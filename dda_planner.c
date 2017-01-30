#include	"dda.h"
#include	"dda_planner.h"

/** \file
  \brief Advance planner for the step-times of movements
*/

#include	<string.h>
#include	<stdlib.h>
#include	<math.h>

#include	"dda_maths.h"
#include "preprocessor_math.h"
// #include "dda_kinematics.h"
#include	"dda_lookahead.h"
// #include "cpu.h"
// #include	"timer.h"
#include	"serial.h"
// #include	"gcode_parse.h"
#include	"dda_queue.h"
#include	"debug.h"
#include	"sersendf.h"
// #include	"pinio.h"
#include "memory_barrier.h"
//#include "graycode.c"


extern MOVE_STATE BSS move_state;

/// \var planner
/// \brief movement planner math which precedes actual movement
MOVE_PLANNER BSS planner;

/// \var c0_P
/// \brief Initialization constant for the ramping algorithm. Timer cycles for
///        first step interval.
// FIXME: Duplicated in dda
static const axes_uint32_t PROGMEM c0_P = {
  (uint32_t)((double)F_CPU / SQRT((double)STEPS_PER_M_X * ACCELERATION / 2000.)),
  (uint32_t)((double)F_CPU / SQRT((double)STEPS_PER_M_Y * ACCELERATION / 2000.)),
  (uint32_t)((double)F_CPU / SQRT((double)STEPS_PER_M_Z * ACCELERATION / 2000.)),
  (uint32_t)((double)F_CPU / SQRT((double)STEPS_PER_M_E * ACCELERATION / 2000.))
};

// Acceleration per QUANTUM.
//           mm/s^2     * (s/QUANTUM)^2            *   steps/m   * 1m/1000mm = steps/QUANTUM^2
// Accel = ACCELERATION * QUANTUM(s)^2 * STEPS_PER_M / 1000
//         ACCELERATION * (QUANTUM/F_CPU)^2 * STEPS_PER_M / 1000
//         ACCELERATION * QUANTUM/F_CPU * QUANTUM/F_CPU * STEPS_PER_M / 1000
//         ACCELERATION * QUANTUM * QUANTUM * STEPS_PER_M / F_CPU / F_CPU / 1000
// Normalized to q8.24; allows up to 2^8=256 in mantissa (steps per quantum)
const axes_uint32_t PROGMEM accel_P = {
  (uint32_t)((((double)ACCELERATION * STEPS_PER_M_X) *(2ULL<<ACCEL_P_SHIFT)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2,
  (uint32_t)((((double)ACCELERATION * STEPS_PER_M_Y) *(2ULL<<ACCEL_P_SHIFT)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2,
  (uint32_t)((((double)ACCELERATION * STEPS_PER_M_Z) *(2ULL<<ACCEL_P_SHIFT)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2,
  (uint32_t)((((double)ACCELERATION * STEPS_PER_M_E) *(2ULL<<ACCEL_P_SHIFT)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2
};


/** Activate a dda for planning purposes

  \param *dda Pointer to entry in the movement queue to plan

  This function prepares the movement planner to begin following the next dda.
  It does not affect the hardware at all. It includes the dda in the maths
  calculations the planner uses for future movements. The previous dda is no
  longer used for movement planning, but it is still alive for the purposes of
  actual movement tracking. The dda passed in is marked "live" and the motion it
  represents should not be further modified (i.e. by dda lookahead).
*/
void dda_plan(DDA *dda) {
  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("Plan: X %lq  Y %lq  Z %lq  F %lu\n"),
               dda->endpoint.axis[X], dda->endpoint.axis[Y],
               dda->endpoint.axis[Z], dda->endpoint.F);

  #ifdef ACCELERATION_RAMPING
    dda->live = 1;

    // This is constant and we could read it directly in dda_clock every time,
    // but we intend to make acceleration a non-constant function someday. This
    // is why we keep the current accel value in the dda.
    planner.accel_per_tick = pgm_read_dword(&accel_P[dda->fast_axis]);

    planner.head = 0;
    planner.tail = 1;
    for (int i = 1; i < SUB_MOVE_QUEUE_SIZE; i++)
      planner.next_n[i] = 0;

    planner.curr_c = dda->c;

    // TODO: Reset this every time or only when we come to a stop?
    planner.remainder = 0;

/*
    // For constant acceleration only
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

*/

    // dda->rampup_steps = muldiv(dda->vmax + dda->vstart, dda->vmax - dda->vstart, planner.accel_per_tick*2);

    // uint32_t accel_time = (dda->vmax - dda->vstart) /
    if (!dda->v_start && !planner.next_n[planner.head]) {
      sersendf_P(PSTR("   planner.velocity  prev=%u  new=%u  STOPPED\n"),
          planner.velocity, planner.accel_per_tick);
      // Calculate velocity at first step: v = v0 + a * t
      planner.velocity = planner.accel_per_tick;
      planner.position = 1;
      planner.next_n[planner.head] = 1;
      planner.next_dc[planner.head] = 1;
    } else {
      sersendf_P(PSTR("   planner.velocity  prev=%u  new=%u\n"),
          planner.velocity, dda->v_start);
      // Calculate velocity at C:  (2 * QUANTUM / C) << ACCEL_P_SHIFT

      // FIXME: This should be unnecessary?
      planner.velocity = dda->v_start;

      // FIXME: Position?  next_n?
      planner.position = 0;

      if (planner.velocity)
        dda->c = muldiv(QUANTUM , 2*1UL<<ACCEL_P_SHIFT, planner.velocity);

      //     planner.next_n[planner.head] = planner.position;
      //     planner.next_dc[planner.head] = 1;  // FIXME: Find actual slope

      // FIXME: Velocity calculated wrong here?  In triangle.gcode we abruptly change direction
      //   Also, we don't compensate for end velocity correctly yet.  Is this what I'm seeing?
      //     printf("dda_start: dda->start_steps=%u\n", dda->start_steps);
    }

    move_state.step_no = 0;
    planner.accel = 1;

    // if (dda->c > QUANTUM) {
    //   uint32_t n = dda->c / QUANTUM;
    //   planner.velocity = planner.accel_per_tick * n;
    //   planner.remainder = planner.accel_per_tick * (n * (n+1) ) / 2;
    //   move_state.step_no = planner.position = dda->steps;
    // }
  #endif
}
