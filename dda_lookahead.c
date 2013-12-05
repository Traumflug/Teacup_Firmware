
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
// Note: the calculation is scaled by a factor 16384 to obtain an answer with a smaller
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
    part = (uint32_t)sqrtf((2000.0f*3600.0f*ACCELERATION*16384.0f)/(float)STEPS_PER_M_X);
  uint32_t res = int_sqrt((steps) << 14) * part;
  return res >> 14;
}

/**
 * Determine the 'jerk' between 2 2D vectors and their speeds. The jerk can be
 * used to obtain an acceptable speed for changing directions between moves.
 * @param x1 x component of first vector
 * @param y1 y component of first vector
 * @param F1 feed rate of first move
 * @param x2 x component of second vector
 * @param y2 y component of second vector
 * @param F2 feed rate of second move
 */
int dda_jerk_size_2d_real(int32_t x1, int32_t y1, uint32_t F1, int32_t x2, int32_t y2, uint32_t F2) {
  const int maxlen = 16384;
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
 * \brief Find maximum corner speed between two moves.
 * \details Find out how fast we can move around around a corner without
 * exceeding the expected jerk. Worst case this speed is zero, which means a
 * full stop between both moves. Best case it's the lower of the maximum speeds.
 *
 * This function is expected to be called from within dda_start().
 *
 * \param [in] prev is the DDA structure of the move previous to the current one.
 * \param [in] current is the DDA structure of the move currently created.
 *
 * \return dda->crossF
 */
void dda_find_crossing_speed(DDA *prev, DDA *current) {
  uint32_t F, dv, speed_factor, max_speed_factor;
  int32_t prevFx, prevFy, prevFz, prevFe;
  int32_t currFx, currFy, currFz, currFe;

  // Bail out if there's nothing to join (e.g. G1 F1500).
  if ( ! prev || prev->nullmove)
    return;

  // We always look at the smaller of both combined speeds,
  // else we'd interpret intended speed changes as jerk.
  F = prev->endpoint.F;
  if (current->endpoint.F < F)
    F = current->endpoint.F;

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("Distance: %lu, then %lu\n"),
               prev->distance, current->distance);

  // Find individual axis speeds.
  // int32_t muldiv(int32_t multiplicand, uint32_t multiplier, uint32_t divisor)
  prevFx = muldiv(prev->delta_um.X, F, prev->distance);
  prevFy = muldiv(prev->delta_um.Y, F, prev->distance);
  prevFz = muldiv(prev->delta_um.Z, F, prev->distance);
  prevFe = muldiv(prev->delta_um.E, F, prev->distance);

  currFx = muldiv(current->delta_um.X, F, current->distance);
  currFy = muldiv(current->delta_um.Y, F, current->distance);
  currFz = muldiv(current->delta_um.Z, F, current->distance);
  currFe = muldiv(current->delta_um.E, F, current->distance);

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("prevF: %ld  %ld  %ld  %ld\ncurrF: %ld  %ld  %ld  %ld\n"),
               prevFx, prevFy, prevFz, prevFe, currFx, currFy, currFz, currFe);

  /**
   * What we want is (for each axis):
   *
   *   delta velocity = dv = |v1 - v2| < max_jerk
   *
   * In case this isn't satisfied, we can slow down by some factor x until
   * the equitation is satisfied:
   *
   *   x * |v1 - v2| < max_jerk
   *
   * Now computation is pretty straightforward:
   *
   *        max_jerk
   *   x = -----------
   *        |v1 - v2|
   *
   *   if x > 1: continue full speed
   *   if x < 1: v = v_max * x
   *
   * See also: https://github.com/Traumflug/Teacup_Firmware/issues/45
   */
  max_speed_factor = (uint32_t)2 << 8;

  dv = currFx > prevFx ? currFx - prevFx : prevFx - currFx;
  if (dv) {
    speed_factor = ((uint32_t)MAX_JERK_X << 8) / dv;
    if (speed_factor < max_speed_factor)
      max_speed_factor = speed_factor;
    if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
      sersendf_P(PSTR("X: dv %lu of %lu   factor %lu of %lu\n"),
                 dv, (uint32_t)MAX_JERK_X, speed_factor, (uint32_t)1 << 8);
  }

  dv = currFy > prevFy ? currFy - prevFy : prevFy - currFy;
  if (dv) {
    speed_factor = ((uint32_t)MAX_JERK_Y << 8) / dv;
    if (speed_factor < max_speed_factor)
      max_speed_factor = speed_factor;
    if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
      sersendf_P(PSTR("Y: dv %lu of %lu   factor %lu of %lu\n"),
                 dv, (uint32_t)MAX_JERK_Y, speed_factor, (uint32_t)1 << 8);
  }

  dv = currFz > prevFz ? currFz - prevFz : prevFz - currFz;
  if (dv) {
    speed_factor = ((uint32_t)MAX_JERK_Z << 8) / dv;
    if (speed_factor < max_speed_factor)
      max_speed_factor = speed_factor;
    if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
      sersendf_P(PSTR("Z: dv %lu of %lu   factor %lu of %lu\n"),
                 dv, (uint32_t)MAX_JERK_Z, speed_factor, (uint32_t)1 << 8);
  }

  dv = currFe > prevFe ? currFe - prevFe : prevFe - currFe;
  if (dv) {
    speed_factor = ((uint32_t)MAX_JERK_E << 8) / dv;
    if (speed_factor < max_speed_factor)
      max_speed_factor = speed_factor;
    if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
      sersendf_P(PSTR("E: dv %lu of %lu   factor %lu of %lu\n"),
                 dv, (uint32_t)MAX_JERK_E, speed_factor, (uint32_t)1 << 8);
  }

  if (max_speed_factor >= ((uint32_t)1 << 8))
    current->crossF = F;
  else
    current->crossF = (F * max_speed_factor) >> 8;

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("Cross speed reduction from %lu to %lu\n"),
               F, current->crossF);

  return;
}

/**
 * \brief Join 2 moves by removing the full stop between them, where possible.
 * \details To join the moves, the deceleration ramp of the previous move and
 * the acceleration ramp of the current move are shortened, resulting in a
 * non-zero speed at that point. The target speed at the corner is already to
 * be found in dda->crossF. See dda_find_corner_speed().
 *
 * Ideally, both ramps can be reduced to actually have Fcorner at the corner,
 * but the surrounding movements might no be long enough to achieve this speed.
 * Analysing both moves to find the best result is done here.
 *
 * TODO: to achieve better results with short moves (move distance < both ramps),
 *       this function should be able to enhance the corner speed on repeated
 *       calls when reverse-stepping through the movement queue.
 *
 * \param [in] prev is the DDA structure of the move previous to the current one.
 * \param [in] current is the DDA structure of the move currently created.
 *
 * Premise: the 'current' move is not dispatched in the queue: it should remain
 * constant while this function is running.
 *
 * Note: the planner always makes sure the movement can be stopped within the
 * last move (= 'current'); as a result a lot of small moves will still limit the speed.
 */
void dda_join_moves(DDA *prev, DDA *current) {

  // Calculating the look-ahead settings can take a while; before modifying
  // the previous move, we need to locally store any values and write them
  // when we are done (and the previous move is not already active).
  uint32_t prev_F, prev_F_in_steps, prev_F_start_in_steps, prev_F_end_in_steps;
  uint32_t prev_rampup, prev_rampdown, prev_total_steps;
  uint32_t crossF, crossF_in_steps;
  uint8_t prev_id;
  // Similarly, we only want to modify the current move if we have the results of the calculations;
  // until then, we do not want to touch the current move settings.
  // Note: we assume 'current' will not be dispatched while this function runs, so we do not to
  // back up the move settings: they will remain constant.
  uint32_t this_F, this_F_in_steps, this_F_start_in_steps, this_rampup, this_rampdown, this_total_steps;
  uint8_t this_id;
  static uint32_t la_cnt = 0;     // Counter: how many moves did we join?
  #ifdef LOOKAHEAD_DEBUG
  static uint32_t moveno = 0;     // Debug counter to number the moves - helps while debugging
  moveno++;
  #endif

  // Bail out if there's nothing to join (e.g. G1 F1500).
  if ( ! prev || prev->nullmove || current->crossF == 0)
    return;

    // Show the proposed crossing speed - this might get adjusted below.
    if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
      sersendf_P(PSTR("Initial crossing speed: %lu\n"), crossF);

  // Make sure we have 2 moves and the previous move is not already active
  if (prev->live == 0) {
    // Perform an atomic copy to preserve volatile parameters during the calculations
    ATOMIC_START
      prev_id = prev->id;
      prev_F = prev->endpoint.F;
      prev_F_start_in_steps = prev->start_steps;
      prev_F_end_in_steps = prev->end_steps;
      prev_rampup = prev->rampup_steps;
      prev_rampdown = prev->rampdown_steps;
      prev_total_steps = prev->total_steps;
      crossF = current->crossF;
      this_id = current->id;
      this_F = current->endpoint.F;
      this_total_steps = current->total_steps;
    ATOMIC_END

    // Here we have to distinguish between feedrate along the movement
    // direction and feedrate of the fast axis. They can differ by a factor
    // of 2.
    // Along direction: F, crossF.
    // Along fast axis already: start_steps, end_steps.
    //
    // All calculations here are done along the fast axis, so recalculate
    // F and crossF to match this, too.
    prev_F = muldiv(prev->fast_um, prev_F, prev->distance);
    this_F = muldiv(current->fast_um, current->endpoint.F, current->distance);
    crossF = muldiv(current->fast_um, crossF, current->distance);

    prev_F_in_steps = ACCELERATE_RAMP_LEN(prev_F);
    this_F_in_steps = ACCELERATE_RAMP_LEN(this_F);
    crossF_in_steps = ACCELERATE_RAMP_LEN(crossF);

    // Show the proposed crossing speed - this might get adjusted below
    serprintf(PSTR("Initial crossing speed: %lu\r\n"), crossF_in_steps);

    // Compute the maximum speed we can reach for crossing.
    crossF_in_steps = MIN(crossF_in_steps, this_total_steps);
    crossF_in_steps = MIN(crossF_in_steps, prev_total_steps + prev_F_start_in_steps);

    if (crossF_in_steps == 0)
      return;

    // Build ramps for previous move.
    if (crossF_in_steps == prev_F_in_steps) {
      prev_rampup = prev_F_in_steps - prev_F_start_in_steps;
      prev_rampdown = 0;
    }
    else if (crossF_in_steps < prev_F_start_in_steps) {
      uint32_t extra, limit;

      prev_rampup = 0;
      prev_rampdown = prev_F_start_in_steps - crossF_in_steps;
      extra = (prev_total_steps - prev_rampdown) >> 1;
      limit = prev_F_in_steps - prev_F_start_in_steps;
      extra = MIN(extra, limit);

      prev_rampup += extra;
      prev_rampdown += extra;
    }
    else {
      uint32_t extra, limit;

      prev_rampup = crossF_in_steps - prev_F_start_in_steps;
      prev_rampdown = 0;
      extra = (prev_total_steps - prev_rampup) >> 1;
      limit = prev_F_in_steps - crossF_in_steps;
      extra = MIN(extra, limit);

      prev_rampup += extra;
      prev_rampdown += extra;
    }
    prev_rampdown = prev_total_steps - prev_rampdown;
    prev_F_end_in_steps = crossF_in_steps;

    // Build ramps for current move.
    if (crossF_in_steps == this_F_in_steps) {
      this_rampup = 0;
      this_rampdown = crossF_in_steps;
    }
    else {
      this_rampup = 0;
      this_rampdown = crossF_in_steps;

      uint32_t extra = (this_total_steps - this_rampdown) >> 1;
      uint32_t limit = this_F_in_steps - crossF_in_steps;
      extra = MIN(extra, limit);

      this_rampup += extra;
      this_rampdown += extra;
    }
    this_rampdown = this_total_steps - this_rampdown;
    this_F_start_in_steps = crossF_in_steps;

    serprintf(PSTR("prev_F_start: %lu\r\n"), prev_F_start_in_steps);
    serprintf(PSTR("prev_F: %lu\r\n"), prev_F_in_steps);
    serprintf(PSTR("prev_rampup: %lu\r\n"), prev_rampup);
    serprintf(PSTR("prev_rampdown: %lu\r\n"), prev_total_steps - prev_rampdown);
    serprintf(PSTR("crossF: %lu\r\n"), crossF_in_steps);
    serprintf(PSTR("this_rampup: %lu\r\n"), this_rampup);
    serprintf(PSTR("this_rampdown: %lu\r\n"), this_total_steps - this_rampdown);
    serprintf(PSTR("this_F: %lu\r\n"), this_F_in_steps);

    uint8_t timeout = 0;

    ATOMIC_START
      // Evaluation: determine how we did...
      lookahead_joined++;

      // Determine if we are fast enough - if not, just leave the moves
      // Note: to test if the previous move was already executed and replaced by a new
      // move, we compare the DDA id.
      if(prev->live == 0 && prev->id == prev_id && current->live == 0 && current->id == this_id) {
        prev->end_steps = prev_F_end_in_steps;
        prev->rampup_steps = prev_rampup;
        prev->rampdown_steps = prev_rampdown;
        current->rampup_steps = this_rampup;
        current->rampdown_steps = this_rampdown;
        current->end_steps = 0;
        current->start_steps = this_F_start_in_steps;
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
