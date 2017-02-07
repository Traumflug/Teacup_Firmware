#include "dda_planner.h"

/** \file
  \brief Advance planner for the step-times of movements
*/

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "dda.h"
#include "dda_maths.h"
#include "dda_queue.h"
#include "debug.h"
#include "sersendf.h"
#include "memory_barrier.h"


/// \var planner
/// \brief movement planner math which precedes actual movement
MOVE_PLANNER BSS planner;

// Acceleration per QUANTUM.
//           mm/s^2     * (s/QUANTUM)^2            *   steps/m   * 1m/1000mm = steps/QUANTUM^2
// Accel = ACCELERATION * QUANTUM(s)^2 * STEPS_PER_M / 1000
//         ACCELERATION * (QUANTUM/F_CPU)^2 * STEPS_PER_M / 1000
//         ACCELERATION * QUANTUM/F_CPU * QUANTUM/F_CPU * STEPS_PER_M / 1000
//         ACCELERATION * QUANTUM * QUANTUM * STEPS_PER_M / F_CPU / F_CPU / 1000
// Normalized to q8.24; allows up to 2^8=256 in mantissa (steps per quantum)
const axes_uint32_t PROGMEM accel_P = {
  ((((double)ACCELERATION * STEPS_PER_M_X) *(2ULL<<ACCEL_P_SHIFT)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2,
  ((((double)ACCELERATION * STEPS_PER_M_Y) *(2ULL<<ACCEL_P_SHIFT)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2,
  ((((double)ACCELERATION * STEPS_PER_M_Z) *(2ULL<<ACCEL_P_SHIFT)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2,
  ((((double)ACCELERATION * STEPS_PER_M_E) *(2ULL<<ACCEL_P_SHIFT)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2
};


/** Initialize movement planner queue

  It is safe to call this function between moves to reset the planner state.
*/
void planner_init(void)
{
  planner.curr_c = 0;
  planner.end_c = 0;

  planner.position = 0;
  planner.accel_per_tick = 0;
  planner.velocity = 0;
  planner.remainder = 0;

  // queue is empty
  planner.head = 0;
  planner.tail = 0;
  for (int i = 1; i < PLANNER_QUEUE_SIZE; i++)
    planner.next_n[i] = 0;
}

void planner_dump(void)
{
  if (DEBUG_PLANNER && (debug_flags & DEBUG_PLANNER)) {
    // Show the current planner contents
    uint32_t c = planner.curr_c;
    uint32_t total=0, count=0;
    uint8_t q=0;

    int i;
    sersendf_P(PSTR("PLAN dump: %u:%u"), planner.head, planner.tail);

    i=planner.head;
    do {
      int32_t dc = planner.next_dc[i];
      uint32_t n = planner.next_n[i];
      if (!n) break;
      count += n;
      total += ((c+dc)*n + c+dc*(int32_t)n)*n/2;
      c += dc*n;
      sersendf_P(PSTR("  (c:%lu(%ld) n:%lu)"), c, dc, n);
      ++q;
      if (++i == PLANNER_QUEUE_SIZE) i=0;
    } while (i != planner.tail);

    sersendf_P(PSTR("  ==> size:%u  dx:%lu dt:%lu\n"), q, count, total);
  }
}

/** Activate a dda for planning purposes

  \param *dda Pointer to entry in the movement queue to plan

  This function prepares the movement planner to begin following the next dda.
  It does not affect the hardware at all. It includes the dda in the maths
  calculations the planner uses for future movements. The previous dda is no
  longer used for movement planning, but it is still alive for the purposes of
  actual movement tracking. The dda passed in is marked "live" and the motion it
  represents should not be further modified (i.e. by dda lookahead).
*/
void planner_begin_dda(DDA *dda)
{
  SIM_ASSERT(dda->planning == UNPLANNED, "dda not unplanned");
  dda->planning = PLANNING_IN_PROGRESS;

  // TODO: Assert that planner is not still processing previous move

  if (DEBUG_PLANNER && (debug_flags & DEBUG_PLANNER))
    sersendf_P(PSTR("\nPLAN: steps=%lu velocity=%lu\n"),
               dda->total_steps, dda->vmax);

  #ifdef ACCELERATION_RAMPING
    dda->live = 1;

    // This is constant and we could read it directly in dda_clock every time,
    // but we intend to make acceleration a non-constant function someday.
    planner.accel_per_tick = pgm_read_dword(&accel_P[dda->fast_axis]);
    planner.accel = 1;
    planner.position = 0;

    // TODO: Report a warning if JERK is exceeded here; requires compensation for different fast_axis
    planner.velocity = dda->v_start;
  #endif
}

/**
  Get next step time based on queued plan steps.

  \param clip_cruise skip over cruise periods where speed does not change

  Called only at interrupt time from dda_step.

  Use clip_cruise to end constant-velocity cruise regions, for example when
  endstop is triggered during home.
*/
uint32_t planner_get(uint8_t clip_cruise)
{
  // Movement underflow (tragedy!)
  if (planner_empty()) {
    if (DEBUG_PLANNER && (debug_flags & DEBUG_PLANNER))
      sersendf_P(PSTR("\n-- PLAN underflow @ %u\n"), planner.head);
    return 0;
  }

  planner.curr_c += planner.next_dc[planner.head];

  if (--planner.next_n[planner.head] == 0) {
    if (DEBUG_PLANNER && (debug_flags & DEBUG_PLANNER))
      sersendf_P(PSTR("\nPLAN << %u. c=%lu  dc=%ld  n=%lu\n"),
        planner.head, planner.curr_c, planner.next_dc[planner.head], planner.next_n[planner.head]+1);
    if (++planner.head == PLANNER_QUEUE_SIZE)
      planner.head = 0;
  } else {
    // Clip the "cruise" motion if requested
    if (clip_cruise && planner.next_dc[planner.head] == 0)
      planner.next_n[planner.head] = 1;
  }
  return planner.curr_c;
}

/**
  Insert a movement into the planner queue_wait

  \param steps number of steps in this movement
  \param speed (timer-based) velocity at the end of this movement
*/
void planner_put(uint32_t steps, uint32_t speed)
{
  if (!steps || planner_full()) return;

  int32_t dc = speed - planner.end_c + steps/2;
  dc /= (int32_t)steps;
  planner.end_c += dc * steps;
  planner.position += steps;

  if (DEBUG_PLANNER && (debug_flags & DEBUG_PLANNER))
    sersendf_P(PSTR("\nPLAN >> %u. c=%lu  dc=%ld  n=%lu\n"),
      planner.tail, planner.end_c, dc, steps);
  ATOMIC_START
    planner.next_dc[planner.tail] = dc;
    planner.next_n[planner.tail] = steps;
    if (++planner.tail == PLANNER_QUEUE_SIZE) planner.tail = 0;
  ATOMIC_END
}

/**
  Insert step plans into the planner using last dda given and constant acceleration
*/
uint8_t planner_fill_queue(DDA *dda)
{

  SIM_ASSERT(dda->planning == PLANNING_IN_PROGRESS, "dda is not PLANNING_IN_PROGRESS");

  // Plan ahead until the movement queue is full
  while (!planner_full()) {
    SIM_ASSERT(planner.position <= dda->total_steps, "Planner position already exceeds dda->total_steps");

    if (planner.position >= dda->total_steps) {
      dda->planning = PLANNING_DONE;
      break;
    }

    uint32_t move_c;
    uint32_t velocity = planner.velocity;
    uint32_t remainder;
    uint32_t dx = 0;

    remainder = planner.remainder;
    uint32_t step_no = planner.position;
    uint32_t v0 = velocity;

    while ( dx==0 ) {
      // ACCELERATING
      if (planner.accel) {
      //   uint32_t old_rem = remainder;
        remainder += velocity;
        velocity += planner.accel_per_tick;
        dx = remainder >> ACCEL_P_SHIFT ;

        // Almost reached mid-point of move or max velocity; time to cruise
        if (step_no*2 + dx*2 >= dda->total_steps - dda->extra_decel_steps ||
                velocity > dda->vmax) {
          // CRUISING
          planner.accel = 0;
          dx = dda->total_steps - 2*step_no - dda->extra_decel_steps;
          // Undo speed change for this step
          velocity -= planner.accel_per_tick;
          remainder -= velocity;

          // Squash overflow if we went too far
          if (dx+step_no > dda->total_steps) {
            dx = 0;
          }
        }
      }
      else {
        // DECELERATING

        remainder -= velocity;
        remainder &= (1ULL<<ACCEL_P_SHIFT) - 1;

        if (velocity < dda->v_end + planner.accel_per_tick) {
          // We hit our min velocity, so stop decelerating
          dx = dda->total_steps - step_no;
        } else {

          if (velocity > planner.accel_per_tick)
            velocity -= planner.accel_per_tick;

          dx = (remainder + velocity) >> ACCEL_P_SHIFT ;
          while (remainder >= velocity && velocity > planner.accel_per_tick) {
            remainder -= velocity;
            remainder &= (1ULL<<ACCEL_P_SHIFT) - 1;
            velocity -= planner.accel_per_tick;
          }
        }
      }

      if (dx + step_no > dda->total_steps)
        dx = dda->total_steps - step_no;

      step_no += dx;
    }

    // Mask off mantissa
    remainder &= (1ULL<<ACCEL_P_SHIFT) - 1;

    // Average velocity since the last movement (v0).
    if (velocity+v0 >= planner.accel_per_tick)
      move_c = muldiv(QUANTUM , 2*1UL<<ACCEL_P_SHIFT, velocity+v0);
    else
      // Avoid dividing by velocity=0 (should never happen)
      move_c = planner.end_c;

    if (move_c < dda->c_min) {
      move_c = dda->c_min;
    }

    planner_put(dx, move_c);
    if (DEBUG_PLANNER && (debug_flags & DEBUG_PLANNER)) {
      sersendf_P(PSTR("\nPLAN vel=%lu  steps=%lu/%lu\n"),
        velocity, step_no, dda->total_steps);
      planner_dump();
    }

    planner.velocity = velocity;
    planner.remainder = remainder;
  }
  return !planner_full();
}
