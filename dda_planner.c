#include "dda.h"
#include "dda_planner.h"

/** \file
  \brief Advance planner for the step-times of movements
*/

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "dda_maths.h"
#include "preprocessor_math.h"
// #include "dda_kinematics.h"
#include "dda_lookahead.h"
// #include "cpu.h"
// #include	"timer.h"
#include "serial.h"
// #include	"gcode_parse.h"
#include "dda_queue.h"
#include "debug.h"
#include "sersendf.h"
// #include	"pinio.h"
#include "memory_barrier.h"
//#include "graycode.c"


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


//___________________________________________________________________
//                                              CONSTANT ACCELERATION
//
/*
    // Constant acceleration math
    dv = vmax - vstart;
    v = v0 + at;
    vmax = vstart + at;
    vmax - vstart = at;
    dv = at;
    t = dv/a;
    x = x0 + v0*t + a*t^2 / 2
    dx = v0*t + at^2/2
    dx = ( vstart + a*t/2 ) * t
    // For accel ramps, substitute t=dv/a, and we get
    dx = ( vstart + a*(dv/a)/2 ) * (dv/a)
    dx = ( vstart + (vmax-vstart)/2) * (vmax-vstart)/a
    dx = (vstart + vmax/2 - vstart/2) * (vmax - vstart) / a
    dx = (vstart/2 + vmax/2) * (vmax - vstart) / a
    dx = (vstart + vmax) * (vmax - vstart) / 2a
    dx = (vmax^2 - vstart^2) / 2a
    dx = (vmax^2/a - vstart^2/a)/2
    dx = vmax^2/2a - vstart^2/2a

    dx = (vmax^2 - vstart^2) / 2a
    2a*dx = vmax^2 - vstart^2
    2a*dx + vstart^2 = vmax^2
    vmax = sqrt(2a*dx + vstart^2)

    dda->rampup_steps = muldiv(dda->vmax + dda->vstart, dda->vmax - dda->vstart, planner.accel_per_tick*2);

    // // vmax = v0 + a*t
    // // a*t = vmax - v0
    // // t = (vmax - v0)/a
    // const uint32_t v0 = 0; // TODO: real start value
    // uint32_t ts = (dda->vmax - v0 + (1UL<<(ACCEL_P_SHIFT))/2)/dda->accel_per_tick;
    //
    // // x(t) = (v0 + a*t/2)*t
    // uint32_t x = (v0 + dda->accel_per_tick * ts / 2) * ts;
    //
    // // actual vmax is limited to min(dx/2, x(t))
    // if (x*2 > dda->total_steps) {
    //   x = dda->total_steps/2;
    //   // only works for v0=0
    //   // x = a*t*t/2
    //   // t = sqrt(2x/a)
    //   // vmax = a*sqrt(2x/a)
    //   // vmax = sqrt(2xa/a)
    //   dda->vmax = sqrt(2*x*dda->accel_per_tick)
    // }

*/

void planner_dump(void)
{
  // Show the current planner contents
  uint32_t c = planner.curr_c;
  uint32_t total=0, count=0;
  uint8_t q=0;

  int i;
  sersendf_P(PSTR("PLANNER dump: %u:%u"), planner.head, planner.tail);

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
  planner.dda = dda;
  if (!dda) return;

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("Plan: X %lq  Y %lq  Z %lq  F %lu  velocity=%lu\n"),
               dda->endpoint.axis[X], dda->endpoint.axis[Y],
               dda->endpoint.axis[Z], dda->endpoint.F, dda->vmax);

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

  Called at interrupt time from dda_step.  Use clip_cruise to end constant-velocity
  cruise regions, for example when endstop is triggered during home.
*/
uint32_t planner_get(uint8_t clip_cruise)
{
  // Movement underflow (tragedy!)
  if (planner_empty()) {
    sersendf_P(PSTR("\n-- PLANNER underflow @ %u\n"), planner.head);
    return 0;
  }

  planner.curr_c += planner.next_dc[planner.head];

  if (--planner.next_n[planner.head] == 0) {
    sersendf_P(PSTR("\n<< PLANNER %u. c=%lu  dc=%ld  n=%lu\n"),
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

  sersendf_P(PSTR("\n>> PLANNER %u. c=%lu  dc=%ld  n=%lu\n"), planner.tail, planner.end_c, dc, steps);
  ATOMIC_START
    planner.next_dc[planner.tail] = dc;
    planner.next_n[planner.tail] = steps;
    if (++planner.tail == PLANNER_QUEUE_SIZE) planner.tail = 0;
  ATOMIC_END
}

/**
  Insert step plans into the planner using last dda given and constant acceleration
*/
void planner_fill_queue(void)
{
  const DDA *dda = planner.dda;

  // Plan ahead until the movement queue is full
  while (!planner_full()) {
    if (!dda) return;
    if (planner.position >= dda->total_steps) {
      // Finished planning current DDA; go on to next one
      dda = queue_get_next(dda);
      planner_begin_dda((DDA *)dda);
      continue;
    }

    uint32_t move_c;
    uint32_t velocity = planner.velocity;
    uint32_t remainder;
    uint32_t dx = 0;

    remainder = planner.remainder;
    uint32_t step_no = planner.position;
    uint32_t v0 = velocity;
    #ifdef SIMULATOR
      uint32_t i = 0;
    #endif

    while ( dx==0 ) {
      // ACCELERATING
      if (planner.accel) {
      //   uint32_t old_rem = remainder;
        remainder += velocity;
        velocity += planner.accel_per_tick;
        dx = remainder >> ACCEL_P_SHIFT ;

        //   sersendf_P(PSTR(" $$> vel=%lu  rem=%lu (%lu)  dx=%lu (%lu)\n"), velocity, remainder, old_rem, dx, step_no);

        // Almost reached mid-point of move or max velocity; time to cruise
        if (step_no*2 + dx*2 >= dda->total_steps - dda->extra_decel_steps ||
                velocity > dda->vmax) {
          // CRUISING
          planner.accel = 0;
          dx = dda->total_steps - 2*step_no - dda->extra_decel_steps;
          // Undo speed change for this step
          velocity -= planner.accel_per_tick;
          remainder -= velocity;

          // Squash overflow because we went too far
          if (dx+step_no > dda->total_steps) {
            dx = 0;
          }
        }
      }
      else {
        // DECELERATING

        remainder -= velocity;
        remainder &= (1ULL<<ACCEL_P_SHIFT) - 1;

      //   sersendf_P(PSTR(" $$< vel=%lu  rem=%lu (%lu)  dx=%lu (%lu)\n"), velocity, remainder, (remainder + velocity) , dx, step_no);

        if (velocity < dda->v_end + planner.accel_per_tick) {
          // We hit our min velocity, so stop decelerating
          dx = dda->total_steps - step_no;
        } else {

          if (velocity > planner.accel_per_tick)
            velocity -= planner.accel_per_tick;

          dx = (remainder + velocity) >> ACCEL_P_SHIFT ;
          while (remainder >= velocity && velocity > planner.accel_per_tick) {
                #ifdef SIMULATOR
                  sersendf_P(PSTR("   %u. vel=%lu  vmax=%lu  dx=%lu (%lu)  tsteps=%lu  rem=%lu\n"),
                    ++i, velocity, dda->vmax, dx, step_no, dda->total_steps, remainder);
                #endif
            remainder -= velocity;
            remainder &= (1ULL<<ACCEL_P_SHIFT) - 1;
            velocity -= planner.accel_per_tick;
          }
        }
      }

      step_no += dx;
      #ifdef SIMULATOR
        sersendf_P(PSTR("   %u. vel=%lu  vmax=%lu  dx=%lu (%lu)  tsteps=%lu  rem=%lu\n"),
           ++i, velocity, dda->vmax, dx, step_no, dda->total_steps, remainder);
      #endif
    }

    // Mask off mantissa
    remainder &= (1ULL<<ACCEL_P_SHIFT) - 1;

    // Average velocity since the last movement (v0).
    // FIXME: The "mul" is constant here; can't we make this a simple division?
    if (velocity+v0 >= planner.accel_per_tick)
      move_c = muldiv(QUANTUM , 2*1UL<<ACCEL_P_SHIFT, velocity+v0);
    else
      // Avoid dividing by velocity=0 (should never happen)
      move_c = planner.end_c;

    if (move_c < dda->c_min) {
      move_c = dda->c_min;
    }

    planner_put(dx, move_c);

    #ifdef SIMULATOR
      sersendf_P(PSTR("  %lu  S#=%lu, State=%d  vel=%lu (%lu,%lu)  c=%lu  vmax=%lu  dx=%lu (%lu)  tsteps=%lu  rem=%lu\n"),
        move_c, step_no, planner.accel, (v0+velocity)/2, v0,velocity, move_c, dda->vmax, dx, step_no, dda->total_steps, (1000*remainder)>>ACCEL_P_SHIFT);
    #endif

    planner.velocity = velocity;
    planner.remainder = remainder;
  }
}
