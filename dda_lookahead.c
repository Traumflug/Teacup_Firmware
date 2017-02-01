
/** \file
  \brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include "dda_lookahead.h"

#ifdef LOOKAHEAD

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>

#include "dda_maths.h"
#include "dda.h"
#include "timer.h"
#include "delay.h"
#include "dda_queue.h"
#include "sersendf.h"
#include "pinio.h"
#include "memory_barrier.h"

#ifdef DEBUG
  // Total number of moves joined together.
  uint32_t lookahead_joined = 0;
  // Moves that did not compute in time to be actually joined.
  uint32_t lookahead_timeout = 0;
#endif

/// \var maximum_jerk_P
/// \brief maximum allowed feedrate jerk on each axis
static const axes_uint32_t PROGMEM maximum_jerk_P = {
  MAX_JERK_X,
  MAX_JERK_Y,
  MAX_JERK_Z,
  MAX_JERK_E
};

/// \var maximum_jerk_steps_P
/// \brief maximum allowed feedrate jerk on each axis in to steps per QUANTUM
///        scaled to q16.16
/// mm/min * 1min/60s * (QUANTUM/F_CPU)s * (steps/m * m/1000mm) = steps / QUANTUM
#define JERK_P_SHIFT 16
static const axes_uint32_t PROGMEM maximum_jerk_steps_P = {
  (uint32_t)((((double)MAX_JERK_X * STEPS_PER_M_X) *(2ULL<<JERK_P_SHIFT)) * QUANTUM / F_CPU / 60 / 1000 + 1)/2,
  (uint32_t)((((double)MAX_JERK_Y * STEPS_PER_M_Y) *(2ULL<<JERK_P_SHIFT)) * QUANTUM / F_CPU / 60 / 1000 + 1)/2,
  (uint32_t)((((double)MAX_JERK_Z * STEPS_PER_M_Z) *(2ULL<<JERK_P_SHIFT)) * QUANTUM / F_CPU / 60 / 1000 + 1)/2,
  (uint32_t)((((double)MAX_JERK_E * STEPS_PER_M_E) *(2ULL<<JERK_P_SHIFT)) * QUANTUM / F_CPU / 60 / 1000 + 1)/2
};



/**
 * \brief Find maximum corner speed between two moves.
 * \details Find out how fast we can move around around a corner without
 * exceeding the expected jerk. Worst case this speed is zero, which means a
 * full stop between both moves. Best case it's the lower of the maximum speeds.
 *
 * This function is expected to be called from within dda_create().
 *
 * \param [in] prev is the DDA structure of the move previous to the current one.
 * \param [in] current is the DDA structure of the move currently created.
 *
 * \return dda->crossF
 */
void dda_find_crossing_speed(DDA *prev, DDA *current) {
  uint32_t F, v, dv, speed_factor, max_speed_factor;
  axes_int32_t prevF, currF;
  axes_int32_t prevV, currV;
  enum axis_e i;

  debug_flags |= DEBUG_DDA;

  // Bail out if there's nothing to join (e.g. first movement after a pause).
  if ( ! prev)
    return;

  // We always look at the smaller of both combined speeds,
  // else we'd interpret intended speed changes as jerk.
  F = prev->endpoint.F;
  if (current->endpoint.F < F)
    F = current->endpoint.F;

  // FIXME: Pre-calc vmax for whole move rather than fixing it up here
  v = MIN(prev->vmax, current->vmax);
  v >>= (ACCEL_P_SHIFT - JERK_P_SHIFT);

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("\nDistance: %lu, then %lu;  F=%lu  v=%lu\n"),
               prev->distance, current->distance, F, v);

  // Find individual axis speeds.
  // TODO: this is eight expensive muldiv()s. It should be possible to store
  //       currF as prevF for the next calculation somehow, to save 4 of
  //       these 8 muldiv()s.
  //       Caveat: bail out condition above and some other non-continuous
  //               situations might need some extra code for handling.
  for (i = X; i < AXIS_COUNT; i++) {
    prevF[i] = muldiv(prev->delta[i], F, prev->total_steps);
    currF[i] = muldiv(current->delta[i], F, current->total_steps);
  }

  for (i = X; i < AXIS_COUNT; i++) {
    prevV[i] = i==prev->fast_axis? v : muldiv(prev->delta[i], v, prev->delta[prev->fast_axis]);
    currV[i] = i==current->fast_axis? v : muldiv(current->delta[i], v, current->delta[current->fast_axis]);
  }

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("prevF: %ld  %ld  %ld  %ld\ncurrF: %ld  %ld  %ld  %ld\n"),
               prevF[X], prevF[Y], prevF[Z], prevF[E],
               currF[X], currF[Y], currF[Z], currF[E]);

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("prevV: %ld  %ld  %ld  %ld\ncurrV: %ld  %ld  %ld  %ld\n"),
            prevV[X], prevV[Y], prevV[Z], prevV[E],
            currV[X], currV[Y], currV[Z], currV[E]);

  /**
   * What we want is (for each axis):
   *
   *   delta velocity = dv = |v1 - v2| < max_jerk
   *
   * In case this isn't satisfied, we can slow down by some factor x until
   * the equation is satisfied:
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
  max_speed_factor = (uint32_t)2 << JERK_P_SHIFT;

  for (i = X; i < AXIS_COUNT; i++) {
    if (get_direction(prev, i) == get_direction(current, i))
      dv = currF[i] > prevF[i] ? currF[i] - prevF[i] : prevF[i] - currF[i];
    else
      dv = currF[i] + prevF[i];

    if (dv) {
      speed_factor = ((uint32_t)pgm_read_dword(&maximum_jerk_P[i]) << JERK_P_SHIFT) / dv;
      if (speed_factor < max_speed_factor)
        max_speed_factor = speed_factor;
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("%c: dv %lu of %lu   factor %lu of %lu\n"),
                   'X' + i, dv, (uint32_t)pgm_read_dword(&maximum_jerk_P[i]),
                   speed_factor, (uint32_t)1 << JERK_P_SHIFT);
    }
  }

  if (max_speed_factor >= ((uint32_t)1 << JERK_P_SHIFT))
    current->crossF = F;
  else
    current->crossF = (F * max_speed_factor) >> JERK_P_SHIFT;

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("CrossF speed reduction from %lu to %lu\n"),
               F, current->crossF);

  max_speed_factor = (uint32_t)2 << JERK_P_SHIFT;

  for (i = X; i < AXIS_COUNT; i++) {
    if (get_direction(prev, i) == get_direction(current, i))
      dv = currV[i] > prevV[i] ? currV[i] - prevV[i] : prevV[i] - currV[i];
    else
      dv = currV[i] + prevV[i];

    if (dv) {
      speed_factor = muldiv((uint32_t)pgm_read_dword(&maximum_jerk_steps_P[i]), 1ULL<<JERK_P_SHIFT, dv);
      if (speed_factor < max_speed_factor)
        max_speed_factor = speed_factor;
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("%c: dv %lu of %lu   factor %lu of %lu\n"),
                   'X' + i, dv, (uint32_t)pgm_read_dword(&maximum_jerk_steps_P[i]),
                   speed_factor, (uint32_t)1 << JERK_P_SHIFT);
    }
  }

  v = MIN(prev->vmax, current->vmax);
  if (max_speed_factor >= ((uint32_t)1 << JERK_P_SHIFT))
    current->v_jerk = v;
  else
    current->v_jerk = muldiv(v , max_speed_factor, 1ULL << JERK_P_SHIFT);

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("Cross speed reduction from %lu to %lu\n"),
            v << (ACCEL_P_SHIFT - JERK_P_SHIFT), current->v_jerk);

  return;
}

/**
 * \brief Join 2 moves by removing the full stop between them, where possible.
 * \details To join the moves, the deceleration ramp of the previous move and
 * the acceleration ramp of the current move are shortened, resulting in a
 * non-zero speed at that point. The target speed at the corner is already to
 * be found in current->crossF. See dda_find_corner_speed().
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
  uint32_t prev_F, prev_F_in_steps, prev_F_start_in_steps;
  uint32_t prev_rampup, prev_rampdown, prev_total_steps;
  uint32_t crossF, crossF_in_steps;
  uint32_t prev_V, prev_V_in_steps, prev_V_start_in_steps;
  uint32_t crossV, crossV_in_steps;
  uint8_t prev_id;
  // Similarly, we only want to modify the current move if we have the results of the calculations;
  // until then, we do not want to touch the current move settings.
  // Note: we assume 'current' will not be dispatched while this function runs, so we do not to
  // back up the move settings: they will remain constant.
  uint32_t this_F, this_F_in_steps, this_rampup, this_rampdown, this_total_steps, this_fast_axis;
  uint32_t this_V, this_V_in_steps;
  uint8_t this_id;
  #ifdef LOOKAHEAD_DEBUG
  static uint32_t la_cnt = 0;     // Counter: how many moves did we join?
  static uint32_t moveno = 0;     // Debug counter to number the moves - helps while debugging
  moveno++;
  #endif

  // Bail out if there's nothing to join.
  if ( ! prev || current->crossF == 0)
    return;

    // Show the proposed crossing speed - this might get adjusted below.
    // if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    //   sersendf_P(PSTR("Initial crossing speed: %lu\n"), current->crossF);
    if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
      sersendf_P(PSTR("Initial crossing speed (v): %lu\n"), current->v_jerk);

  // Make sure we have 2 moves and the previous move is not already active
  if (prev->live == 0) {
    // Copy DDA ids to verify later that nothing changed during calculations
    ATOMIC_START
      prev_id = prev->id;
      this_id = current->id;
    ATOMIC_END

    prev_F = prev->endpoint.F;
    prev_F_start_in_steps = prev->start_steps;
    prev_total_steps = prev->total_steps;
    crossF = current->crossF;
    this_total_steps = current->total_steps;
    this_fast_axis = current->fast_axis;

    this_V = current->vmax;
    prev_V = prev->vmax;
    prev_V_start_in_steps = prev->start_steps;
    crossV = current->v_jerk;

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

    prev_F_in_steps = acc_ramp_len(prev_F, this_fast_axis);
    this_F_in_steps = acc_ramp_len(this_F, this_fast_axis);
    crossF_in_steps = acc_ramp_len(crossF, this_fast_axis);

    // During the movement we will know almost all we need to know, regardless of total_steps
    // But there are two basic conditions we must consider:
    //
    // 1. start speed is less than end speed
    //    In this case we know we can achieve the required delta-v if
    //    we have enough total_steps.  We must reduce end speed if we can not.
    //
    // 2. start speed is greater than or equal to end speed
    //    We know we can achieve this because we previously decided start speed
    //    was ok even though we had to end at zero.  But our planner needs to know
    //    how many extra steps it takes to decelerate to the target speed.

    uint32_t prev_dv, prev_dv_steps, this_dv_steps, this_accel, prev_accel;
    this_accel = pgm_read_dword(&accel_P[this_fast_axis]);
    this_dv_steps = muldiv(crossV>>(ACCEL_P_SHIFT/2), crossV>>(ACCEL_P_SHIFT/2), 2*this_accel);

    uint32_t prev_ratio=0, this_ratio=0;

    if (this_dv_steps > this_total_steps) {
      // We have dx=v^2/2a which is too big.  We must divide v by some ratio > 1
      // to get dx == total_steps.  That is, find r where total_steps = v^2/2a/r.
      // This is just r = v^2/2a/total_steps. But this_dv_steps = v^2/2a, so
      // r = this_dv_steps / total_steps.  Then our adjusted V to satisfy our
      // original equation is V=v/sqrt(r), then total_steps=V^2/2a
      this_ratio = muldiv(this_dv_steps, 65536, this_total_steps); // get r in 16.16 fixed-point
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
        sersendf_P(PSTR("CrossV reduction because this_dx=%lu, prev_total_steps=%lu:  this_ratio=%lu/65536\n"),
               this_dv_steps, this_total_steps, this_ratio);
      }
    }

    prev_accel = pgm_read_dword(&accel_P[prev->fast_axis]);
    prev_dv = prev->v_start > crossV ? prev->v_start - crossV : crossV - prev->v_start ;
    prev_dv_steps = muldiv((prev->v_start + crossV)>>(ACCEL_P_SHIFT/2), prev_dv>>(ACCEL_P_SHIFT/2), 2*prev_accel);

    if (prev->v_start < crossV) {
      if (prev_dv_steps > prev_total_steps) {
        // Must reduce crossV further make prev->v_end sane
        // We have dx=(ve^2-vs^2)/2a which is too big.  We must divide ve by some ratio > 1
        // to get dx == total_steps.  That is, find r where total_steps = (ve^2/r-vs^2)/2a.
        // total_steps = ve^2/2a/r-vs^2/2a
        // total_steps + vs^2/2a = ve^2/2a/r
        // r = ve^2/2a/(total_steps + vs^2/2a)
        //   because prev_ve = this_vs, ve^2/2a = this_dv_steps
        // r = this_dv_steps/(total_steps + vs^2/2a)
        // Then our adjusted V to satisfy our original equation is V=v/sqrt(r)
        uint32_t denom = muldiv(prev->v_start, prev->v_start, 2*prev_accel) + prev->total_steps;
        prev_ratio = muldiv(this_dv_steps, 65536, denom); // get r in 16.16 fixed-point
        if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
          sersendf_P(PSTR("CrossV reduction because prev_dx=%lu, prev_total_steps=%lu:  prev_ratio=%lu/65536\n"),
                 prev_dv_steps, prev_total_steps, prev_ratio);
        }
      }
    }

    if ( prev_ratio || this_ratio) {
      uint32_t ratio = MAX(prev_ratio, this_ratio);
      uint32_t orig = crossV;
      crossV = muldiv(crossV, 256, int_sqrt(ratio));

      prev_dv = prev->v_start > crossV ? prev->v_start - crossV : crossV - prev->v_start ;
      prev_dv_steps = muldiv((prev->v_start + crossV)>>(ACCEL_P_SHIFT/2), prev_dv>>(ACCEL_P_SHIFT/2), 2*prev_accel);
      this_dv_steps = muldiv(crossV>>(ACCEL_P_SHIFT/2), crossV>>(ACCEL_P_SHIFT/2), 2*this_accel);

      if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
        sersendf_P(PSTR("   reduced by ratio %lu/65536 from %lu to %lu\n"),
                ratio, orig, crossV);
        sersendf_P(PSTR("   prev: vs=%lu  ve=%lu  dx=%lu  total_steps=%lu\n"),
                prev->v_start, crossV, prev_dv_steps, prev_total_steps);
        sersendf_P(PSTR("   this: vs=%lu  ve=%lu  dx=%lu  total_steps=%lu\n"),
                crossV, 0, this_dv_steps, this_total_steps);
      }
    }
    int32_t prev_extra_steps = prev->v_start > crossV ? prev_dv_steps : -prev_dv_steps;
    int32_t this_extra_steps = this_dv_steps;

    // // Show the proposed crossing speed - this might get adjusted below
    // if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    //   sersendf_P(PSTR("Initial crossing speed in steps: %lu\n"), crossF_in_steps);
    //
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

    // Build ramps for current move.
    this_rampup = 0;
    this_rampdown = crossF_in_steps;
    if (crossF_in_steps != this_F_in_steps) {
      uint32_t extra = (this_total_steps - this_rampdown) >> 1;
      uint32_t limit = this_F_in_steps - crossF_in_steps;
      extra = MIN(extra, limit);

      this_rampup += extra;
      this_rampdown += extra;
    }
    this_rampdown = this_total_steps - this_rampdown;

    if (DEBUG_DDA && (debug_flags & DEBUG_DDA)) {
      sersendf_P(PSTR("prev_F_start: %lu\n"), prev_F_start_in_steps);
      sersendf_P(PSTR("prev_F: %lu\n"), prev_F_in_steps);
      sersendf_P(PSTR("prev_rampup: %lu\n"), prev_rampup);
      sersendf_P(PSTR("prev_rampdown: %lu\n"),
                 prev_total_steps - prev_rampdown);
      sersendf_P(PSTR("crossF: %lu\n"), crossF_in_steps);
      sersendf_P(PSTR("this_rampup: %lu\n"), this_rampup);
      sersendf_P(PSTR("this_rampdown: %lu\n"),
                 this_total_steps - this_rampdown);
      sersendf_P(PSTR("this_F: %lu\n"), this_F_in_steps);
    }

    #ifdef DEBUG
      uint8_t timeout = 0;
    #endif

    ATOMIC_START
      // Evaluation: determine how we did...
      #ifdef DEBUG
        lookahead_joined++;
      #endif

      // Determine if we are fast enough - if not, just leave the moves
      // Note: to test if the previous move was already executed and replaced by a new
      // move, we compare the DDA id.
      if(prev->live == 0 && prev->id == prev_id && current->live == 0 && current->id == this_id) {
        prev->end_steps = crossF_in_steps;
        prev->rampup_steps = prev_rampup;
        prev->rampdown_steps = prev_rampdown;
        current->rampup_steps = this_rampup;
        current->rampdown_steps = this_rampdown;
        current->end_steps = 0;
        current->start_steps = crossF_in_steps;

        prev->extra_decel_steps = prev_extra_steps;
        prev->v_end = crossV;
        current->v_start = crossV;
        current->extra_decel_steps = this_extra_steps;
        #ifdef LOOKAHEAD_DEBUG
          la_cnt++;
        #endif
      }
      #ifdef DEBUG
        else
          timeout = 1;
      #endif
    ATOMIC_END

    // If we were not fast enough, any feedback will happen outside the atomic block:
    #ifdef DEBUG
      if (timeout) {
        sersendf_P(PSTR("// Notice: look ahead not fast enough\n"));
        lookahead_timeout++;
      }
    #endif
  }
}

#endif /* LOOKAHEAD */
