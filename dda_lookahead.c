
/** \file
  \brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include "dda_lookahead.h"

#ifdef LOOKAHEAD

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#ifndef SIMULATOR
#include <avr/interrupt.h>
#endif

#include "dda_maths.h"
#include "dda.h"
#include "timer.h"
#include "delay.h"
#include "serial.h"
#include "sermsg.h"
#include "gcode_parse.h"
#include "dda_queue.h"
#include "debug.h"
#include "sersendf.h"
#include "pinio.h"
#include "memory_barrier.h"

extern uint8_t use_lookahead;

uint32_t lookahead_joined = 0;      // Total number of moves joined together
uint32_t lookahead_timeout = 0;     // Moves that did not compute in time to be actually joined

// Used for look-ahead debugging
#ifdef LOOKAHEAD_DEBUG_VERBOSE
  #define serprintf(...) sersendf_P(__VA_ARGS__)
#else
  #define serprintf(...)
#endif

// We also need the inverse: given a ramp length, determine the expected speed
// Note: the calculation is scaled by a factor 10000 to obtain an answer with a smaller
// rounding error.
// Warning: this is an expensive function as it requires a square root to get the result.
//
uint32_t dda_steps_to_velocity(uint32_t steps) {
  // v(t) = a*t, with v in mm/s and a = acceleration in mm/s²
  // s(t) = 1/2*a*t² with s (displacement) in mm
  // Rewriting yields v(s) = sqrt(2*a*s)
  // Rewriting into steps and seperation in constant part and dynamic part:
  // F_steps = sqrt((2000*a)/STEPS_PER_M_X) * 60 * sqrt(steps)
  static uint32_t part = 0;
  if(part == 0)
    part = int_sqrt((uint32_t)((2000.0f*ACCELERATION*3600.0f*10000.0f)/(float)STEPS_PER_M_X));
  uint32_t res = int_sqrt((steps) * 10000) * part;
  return res / 10000;
}

/**
 * Determine the 'jerk' between 2 2D vectors and their speeds. The jerk can be used to obtain an
 * acceptable speed for changing directions between moves.
 * Vector delta is in um, speed is in mm/min.
 * @param x1 x component of first vector
 * @param y1 y component of first vector
 * @param F1 feed rate of first move
 * @param x2 x component of second vector
 * @param y2 y component of second vector
 * @param F2 feed rate of second move
 */
int dda_jerk_size_2d_real(int32_t x1, int32_t y1, uint32_t F1, int32_t x2, int32_t y2, uint32_t F2) {
  const int maxlen = 10000;
  // Normalize vectors so their length will be fixed
  // Note: approx_distance is not precise enough and may result in violent direction changes
  //sersendf_P(PSTR("Input vectors: (%ld, %ld) and (%ld, %ld)\r\n"),x1,y1,x2,y2);
  int32_t len = int_sqrt(x1*x1+y1*y1);
  x1 = (x1 * maxlen) / len;
  y1 = (y1 * maxlen) / len;
  len = int_sqrt(x2*x2+y2*y2);
  x2 = (x2 * maxlen) / len;
  y2 = (y2 * maxlen) / len;

  //sersendf_P(PSTR("Normalized vectors: (%ld, %ld) and (%ld, %ld)\r\n"),x1,y1,x2,y2);

  // Now scale the normalized vectors by their speeds
  x1 *= F1; y1 *= F1; x2 *= F2; y2 *= F2;

  //sersendf_P(PSTR("Speed vectors: (%ld, %ld) and (%ld, %ld)\r\n"),x1,y1,x2,y2);

  // The difference between the vectors actually depicts the jerk
  x1 -= x2; y1 -= y2;

  //sersendf_P(PSTR("Jerk vector: (%ld, %ld)\r\n"),x1,y1);

  return approx_distance(x1,y1) / maxlen;
}

/**
 * Determine the 'jerk' for 2 1D vectors and their speeds. The jerk can be used to obtain an
 * acceptable speed for changing directions between moves.
 * Vector delta is in um, speed is in mm/min.
 * @param x component of 1d vector - used to determine if we go back or forward
 * @param F feed rate
 */
int dda_jerk_size_1d(int32_t x1, uint32_t F1, int32_t x2, uint32_t F2) {
  if(x1 > 0) x1 = F1;
  else x1 = -F1;
  if(x2 > 0) x2 = F2;
  else x2 = -F2;

  // The difference between the vectors actually depicts the jerk
  x1 -= x2;
  if(x1 < 0) x1 = -x1;  // Make sure it remains positive

  //sersendf_P(PSTR("Jerk vector: (%ld, %ld)\r\n"),x1,y1);
  return x1;
}

/**
 * Determine the 'jerk' between 2 vectors and their speeds. The jerk can be used to obtain an
 * acceptable speed for changing directions between moves.
 * Instead of using 2 axis at once, consider the jerk for each axis individually and take the
 * upper limit between both. This ensures that each axis does not changes speed too fast.
 * Vector delta is in um, speed is in mm/min.
 * @param x1 x component of first vector
 * @param y1 y component of first vector
 * @param F1 feed rate of first move
 * @param x2 x component of second vector
 * @param y2 y component of second vector
 * @param F2 feed rate of second move
 */
int dda_jerk_size_2d(int32_t x1, int32_t y1, uint32_t F1, int32_t x2, int32_t y2, uint32_t F2) {
  return MAX(dda_jerk_size_1d(x1,F1,x2,F2),dda_jerk_size_1d(y1,F1,y2,F2));
}

/**
 * Safety procedure: if something goes wrong, for example an opto is triggered during normal movement,
 * we shut down the entire machine.
 * @param msg The reason why the machine did an emergency stop
 */
void dda_emergency_shutdown(PGM_P msg) {
  // Todo: is it smart to enable all interrupts again? e.g. can we create concurrent executions?
  sei();  // Enable interrupts to print the message
  serial_writestr_P(PSTR("error: emergency stop:"));
  if(msg!=NULL) serial_writestr_P(msg);
  serial_writestr_P(PSTR("\r\n"));
  delay_ms(20);   // Delay so the buffer can be flushed - otherwise the message is never sent
  timer_stop();
  queue_flush();
  power_off();
  cli();
  for (;;) { }
}

/**
 * Join 2 moves by removing the full stop between them, where possible.
 * To join the moves, the expected jerk - or force - of the change in direction is calculated.
 * The jerk is used to scale the common feed rate between both moves to obtain an acceptable speed
 * to transition between 'prev' and 'current'.
 *
 * Premise: we currently join the last move in the queue and the one before it (if any).
 * This means the feed rate at the end of the 'current' move is 0.
 *
 * Premise: the 'current' move is not dispatched in the queue: it should remain constant while this
 * function is running.
 *
 * Note: the planner always makes sure the movement can be stopped within the
 * last move (= 'current'); as a result a lot of small moves will still limit the speed.
 */
void dda_join_moves(DDA *prev, DDA *current) {

  // Calculating the look-ahead settings can take a while; before modifying
  // the previous move, we need to locally store any values and write them
  // when we are done (and the previous move is not already active).
  uint32_t prev_F, prev_F_start, prev_F_end, prev_rampup, prev_rampdown, prev_total_steps;
  uint8_t prev_id;
  // Similarly, we only want to modify the current move if we have the results of the calculations;
  // until then, we do not want to touch the current move settings.
  // Note: we assume 'current' will not be dispatched while this function runs, so we do not to
  // back up the move settings: they will remain constant.
  uint32_t this_F_start, this_start, this_rampup, this_rampdown;
  int32_t jerk, jerk_e;       // Expresses the forces if we would change directions at full speed
  static uint32_t la_cnt = 0;     // Counter: how many moves did we join?
  #ifdef LOOKAHEAD_DEBUG
  static uint32_t moveno = 0;     // Debug counter to number the moves - helps while debugging
  moveno++;
  #endif

  // Bail out if there's nothing to join (e.g. G1 F1500).
  if ( ! prev || prev->nullmove)
    return;

  serprintf(PSTR("Current Delta: %ld,%ld,%ld E:%ld Live:%d\r\n"), current->delta.X, current->delta.Y, current->delta.Z, current->delta.E, current->live);
  serprintf(PSTR("Prev    Delta: %ld,%ld,%ld E:%ld Live:%d\r\n"), prev->delta.X, prev->delta.Y, prev->delta.Z, prev->delta.E, prev->live);

  // Look-ahead: attempt to join moves into smooth movements
  // Note: moves are only modified after the calculations are complete.
  // Only prepare for look-ahead if we have 2 available moves to
  // join and the Z axis is unused (for now, Z axis moves are NOT joined).
  if (prev->live == 0 && prev->delta.Z == current->delta.Z) {
    // Calculate the jerk if the previous move and this move would be joined
    // together at full speed.
    jerk = dda_jerk_size_2d(prev->delta.X, prev->delta.Y, prev->endpoint.F,
        current->delta.X, current->delta.Y, current->endpoint.F);
    serprintf(PSTR("Jerk: %lu\r\n"), jerk);
    jerk_e = dda_jerk_size_1d(prev->delta.E, prev->endpoint.F, current->delta.E, current->endpoint.F);
    serprintf(PSTR("Jerk_e: %lu\r\n"), jerk_e);
  } else {
    // Move already executing or Z moved: abort the join
    return;
  }

  // Make sure we have 2 moves and the previous move is not already active
  if (prev->live == 0) {
    // Perform an atomic copy to preserve volatile parameters during the calculations
    ATOMIC_START
      prev_id = prev->id;
      prev_F = prev->endpoint.F;
      prev_F_start = prev->F_start;
      prev_F_end = prev->F_end;
      prev_rampup = prev->rampup_steps;
      prev_rampdown = prev->rampdown_steps;
      prev_total_steps = prev->total_steps;
    ATOMIC_END

    // The initial crossing speed is the minimum between both target speeds
    // Note: this is a given: the start speed and end speed can NEVER be
    // higher than the target speed in a move!
    // Note 2: this provides an upper limit, if needed, the speed is lowered.
    uint32_t crossF = prev_F;
    if(crossF > current->endpoint.F) crossF = current->endpoint.F;

    //sersendf_P(PSTR("j:%lu - XF:%lu"), jerk, crossF);

    // If the XY jerk is too big, scale the proposed cross speed
    if(jerk > LOOKAHEAD_MAX_JERK_XY) {
      serprintf(PSTR("Jerk too big: scale cross speed between moves\r\n"));
      // Get the highest speed between both moves
      if(crossF < prev_F)
        crossF = prev_F;

      // Perform an exponential scaling
      uint32_t ujerk = (uint32_t)jerk;  // Use unsigned to double the range before overflowing
      crossF = (crossF*LOOKAHEAD_MAX_JERK_XY*LOOKAHEAD_MAX_JERK_XY)/(ujerk*ujerk);

      // Optimize: if the crossing speed is zero, there is no join possible between these
      // two (fast) moves. Stop calculating and leave the full stop that is currently between
      // them.
      if(crossF == 0)
        return;

      // Safety: make sure we never exceed the maximum speed of a move
      if(crossF > current->endpoint.F) crossF = current->endpoint.F;
      if(crossF > prev_F) crossF = prev_F;
      sersendf_P(PSTR("=>F:%lu"), crossF);
    }
    // Same to the extruder jerk: make sure we do not yank it
    if(jerk_e > LOOKAHEAD_MAX_JERK_E) {
      sersendf_P(PSTR("Jerk_e too big: scale cross speed between moves\r\n"));
      uint32_t crossF2 = MAX(current->endpoint.F, prev_F);

      // Perform an exponential scaling
      uint32_t ujerk = (uint32_t)jerk_e;  // Use unsigned to double the range before overflowing
      crossF2 = (crossF2*LOOKAHEAD_MAX_JERK_E*LOOKAHEAD_MAX_JERK_E)/(ujerk*ujerk);

      // Only continue with joining if there is a feasible crossing speed
      if(crossF2 == 0) return;

      // Safety: make sure the proposed speed is not higher than the target speeds of each move
      crossF2 = MIN(crossF2, current->endpoint.F);
      crossF2 = MIN(crossF2, prev_F);

      if(crossF2 > crossF) {
        sersendf_P(PSTR("Jerk_e: %lu => crossF: %lu (original: %lu)\r\n"), jerk_e, crossF2, crossF);
      }

      // Pick the crossing speed for these 2 move to be within the jerk limits
      crossF = MIN(crossF, crossF2);
    }

    // Show the proposed crossing speed - this might get adjusted below
    serprintf(PSTR("Initial crossing speed: %lu\r\n"), crossF);

    // Forward check: test if we can actually reach the target speed in the previous move
    // If not: we need to determine the obtainable speed and adjust crossF accordingly.
    // Note: these ramps can be longer than the move: if so we can not reach top speed.
    uint32_t up = ACCELERATE_RAMP_LEN(prev_F) - ACCELERATE_RAMP_LEN(prev_F_start);
    uint32_t down = ACCELERATE_RAMP_LEN(prev_F) - ACCELERATE_RAMP_LEN(crossF);
    // Test if both the ramp up and ramp down fit within the move
    if(up+down > prev_total_steps) {
      // Test if we can reach the crossF rate: if the difference between both ramps is larger
      // than the move itself, there is no ramp up or down from F_start to crossF...
      uint32_t diff = (up>down) ? up-down : down-up;
      if(diff > prev_total_steps) {
        // Cannot reach crossF from F_start, lower crossF and adjust both ramp-up and down
        down = 0;
        // Before we can determine how fast we can go in this move, we need the number of
        // steps needed to reach the entry speed.
        uint32_t prestep = ACCELERATE_RAMP_LEN(prev_F_start);
        // Calculate what feed rate we can reach during this move
        crossF = dda_steps_to_velocity(prestep+prev_total_steps);
        // Make sure we do not exceed the target speeds
        if(crossF > prev_F) crossF = prev_F;
        if(crossF > current->endpoint.F) crossF = current->endpoint.F;
        // The problem with the 'dda_steps_to_velocity' is that it will produce a
        // rounded result. Use it to obtain an exact amount of steps needed to reach
        // that speed and set that as the ramp up; we might stop accelerating for a
        // couple of steps but that is better than introducing errors in the moves.
        up = ACCELERATE_RAMP_LEN(crossF) - prestep;

        #ifdef LOOKAHEAD_DEBUG
        // Sanity check: the ramp up should never exceed the move length
        if(up > prev_total_steps) {
          sersendf_P(PSTR("FATAL ERROR during prev ramp scale, ramp is too long: up:%lu ; len:%lu ; target speed: %lu\r\n"),
            up, prev_total_steps, crossF);
          sersendf_P(PSTR("F_start:%lu ; F:%lu ; crossF:%lu\r\n"),
            prev_F_start, prev_F, crossF);
          dda_emergency_shutdown(PSTR("LA prev ramp scale, ramp is too long"));
        }
        #endif
        // Show the result on the speed on the clipping of the ramp
        serprintf(PSTR("Prev speed & crossing speed truncated to: %lu\r\n"), crossF);
      } else {
        // Can reach crossF; determine the apex between ramp up and ramp down
        // In other words: calculate how long we can accelerate before decelerating to exit at crossF
        // Note: while the number of steps is exponentially proportional to the velocity,
        // the acceleration is linear: we can simply remove the same number of steps of both ramps.
        uint32_t diff = (up + down - prev_total_steps) / 2;
        up -= diff;
        down -= diff;
      }

      #ifdef LOOKAHEAD_DEBUG
      // Sanity check: make sure the speed limits are maintained
      if(prev_F_start > prev_F || crossF > prev_F) {
        serprintf(PSTR("Prev target speed exceeded!: prev_F_start:%lu ; prev_F:%lu ; prev_F_end:%lu\r\n"), prev_F_start, prev_F, crossF);
        dda_emergency_shutdown(PSTR("Prev target speed exceeded"));
      }
      #endif
    }
    // Save the results
    prev_rampup = up;
    prev_rampdown = prev_total_steps - down;
    prev_F_end = crossF;

    #ifdef LOOKAHEAD_DEBUG
    // Sanity check: make sure the speed limits are maintained
    if(crossF > current->endpoint.F) {
      serprintf(PSTR("This target speed exceeded!: F_start:%lu ; F:%lu ; prev_F_end:%lu\r\n"), crossF, current->endpoint.F);
      dda_emergency_shutdown(PSTR("This target speed exceeded"));
    }
    #endif

    // Forward check 2: test if we can actually reach the target speed in this move.
    // If not: determine obtainable speed and adjust crossF accordingly. If that
    // happens, a third (reverse) pass is needed to lower the speeds in the previous move...
    //ramp_scaler = ACCELERATE_SCALER(current->lead); // Use scaler for current leading axis
    up = ACCELERATE_RAMP_LEN(current->endpoint.F) - ACCELERATE_RAMP_LEN(crossF);
    down = ACCELERATE_RAMP_LEN(current->endpoint.F);
    // Test if both the ramp up and ramp down fit within the move
    if(up+down > current->total_steps) {
      // Test if we can reach the crossF rate
      // Note: this is the inverse of the previous move: we need to exit at 0 speed as
      // this is the last move in the queue. Implies that down >= up
      if(down-up > current->total_steps) {
        serprintf(PSTR("This move can not reach crossF - lower it\r\n"));
        // Cannot reach crossF, lower it and adjust ramps
        // Note: after this, the previous move needs to be modified to match crossF.
        up = 0;
        // Calculate what crossing rate we can reach: total/down * F
        crossF = dda_steps_to_velocity(current->total_steps);
        // Speed limit: never exceed the target rate
        if(crossF > current->endpoint.F) crossF = current->endpoint.F;
        // crossF will be conservative: calculate the actual ramp down length
        down = ACCELERATE_RAMP_LEN(crossF);

        #ifdef LOOKAHEAD_DEBUG
        // Make sure we can break to a full stop before the move ends
        if(down > current->total_steps) {
          sersendf_P(PSTR("FATAL ERROR during ramp scale, ramp is too long: down:%lu ; len:%lu ; target speed: %lu\r\n"),
            down, current->total_steps, crossF);
          dda_emergency_shutdown(PSTR("LA current ramp scale, ramp is too long"));
        }
        #endif
      } else {
        serprintf(PSTR("This: crossF is usable but we will not reach Fmax\r\n"));
        // Can reach crossF; determine the apex between ramp up and ramp down
        // In other words: calculate how long we can accelerate before decelerating to start at crossF
        // and end at F = 0
        uint32_t diff = (down + up - current->total_steps) / 2;
        up -= diff;
        down -= diff;
        serprintf(PSTR("Apex: %lu - new up: %lu - new down: %lu\r\n"), diff, up, down);

        // sanity stuff: calculate the speeds for these ramps
        serprintf(PSTR("Ramp up speed: %lu mm/s\r\n"), dda_steps_to_velocity(up+prev->rampup_steps));
        serprintf(PSTR("Ramp down speed: %lu mm/s\r\n"), dda_steps_to_velocity(down));
      }
    }
    // Save the results
    this_rampup = up;
    this_rampdown = current->total_steps - down;
    this_F_start = crossF;
    this_start = ACCELERATE_RAMP_LEN(this_F_start);
    serprintf(PSTR("Actual crossing speed: %lu\r\n"), crossF);

    // Potential reverse processing:
    // Make sure the crossing speed is the same, if its not, we need to slow the previous move to
    // the current crossing speed (note: the crossing speed could only be lowered).
    // This can happen when this move is a short move and the previous move was a larger or faster move:
    // since we need to be able to stop if this is the last move, we lowered the crossing speed
    // between this move and the previous move...
    if(prev_F_end != crossF) {
      // Third reverse pass: slow the previous move to end at the target crossing speed.
      //ramp_scaler = ACCELERATE_SCALER(current->lead); //todo: prev_lead // Use scaler for previous leading axis (again)
      // Note: use signed values so we  can check if results go below zero
      // Note 2: when up2 and/or down2 are below zero from the start, you found a bug in the logic above.
      int32_t up2 = ACCELERATE_RAMP_LEN(prev_F) - ACCELERATE_RAMP_LEN(prev_F_start);
      int32_t down2 = ACCELERATE_RAMP_LEN(prev_F) - ACCELERATE_RAMP_LEN(crossF);

      // Test if both the ramp up and ramp down fit within the move
      if(up2+down2 > prev_total_steps) {
        int32_t diff = (up2 + down2 - (int32_t)prev_total_steps) / 2;
        up2 -= diff;
        down2 -= diff;

        #ifdef LOOKAHEAD_DEBUG
        if(up2 < 0 || down2 < 0) {
          // Cannot reach crossF from prev_F_start - this should not happen!
          sersendf_P(PSTR("FATAL ERROR during reverse pass ramp scale, ramps are too long: up:%ld ; down:%ld; len:%lu ; F_start: %lu ; crossF: %lu\r\n"),
                    up2, down2, prev_total_steps, prev_F_start, crossF);
          sersendf_P(PSTR("Original up: %ld - down %ld (diff=%ld)\r\n"),up2+diff,down2+diff,diff);
          dda_emergency_shutdown(PSTR("reverse pass ramp scale, can not reach F_end from F_start"));
        }
        #endif
      }
      // Assign the results
      prev_rampup = up2;
      prev_rampdown = prev_total_steps - down2;
      prev_F_end = crossF;
    }

    #ifdef LOOKAHEAD_DEBUG
    if(crossF > current->endpoint.F || crossF > prev_F)
      dda_emergency_shutdown(PSTR("Lookahead exceeded speed limits in crossing!"));

    // When debugging, print the 2 moves we joined
    // Legenda: Fs=F_start, len=# of steps, up/down=# steps in ramping, Fe=F_end
    serprintf(PSTR("LA: (%lu) Fs=%lu, len=%lu, up=%lu, down=%lu, Fe=%lu <=> (%lu) Fs=%lu, len=%lu, up=%lu, down=%lu, Fe=0\r\n\r\n"),
      moveno-1, prev->F_start, prev->total_steps, prev->rampup_steps,
      prev->total_steps-prev->rampdown_steps, prev->F_end,
      moveno, current->F_start, current->total_steps, current->rampup_steps,
      current->total_steps - this_rampdown);
    #endif

    uint8_t timeout = 0;

    ATOMIC_START
      // Evaluation: determine how we did...
      lookahead_joined++;

      // Determine if we are fast enough - if not, just leave the moves
      // Note: to test if the previous move was already executed and replaced by a new
      // move, we compare the DDA id.
      if(prev->live == 0 && prev->id == prev_id) {
        prev->F_end = prev_F_end;
        prev->rampup_steps = prev_rampup;
        prev->rampdown_steps = prev_rampdown;
        current->rampup_steps = this_rampup;
        current->rampdown_steps = this_rampdown;
        current->F_end = 0;
        current->F_start = this_F_start;
        current->start_steps = this_start;
        la_cnt++;
      } else
        timeout = 1;
    ATOMIC_END

    // If we were not fast enough, any feedback will happen outside the atomic block:
    if(timeout) {
      sersendf_P(PSTR("Error: look ahead not fast enough\r\n"));
      lookahead_timeout++;
    }
  }
}

#endif /* LOOKAHEAD */
