#include	"dda.h"

/** \file
	\brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include	<string.h>
#include	<stdlib.h>
#include	<math.h>

#include	"dda_maths.h"
#include "preprocessor_math.h"
#include "dda_kinematics.h"
#include	"dda_lookahead.h"
#include "cpu.h"
#include	"timer.h"
#include	"serial.h"
#include	"gcode_parse.h"
#include	"dda_queue.h"
#include	"debug.h"
#include	"sersendf.h"
#include	"pinio.h"
#include "memory_barrier.h"
//#include "graycode.c"

#ifdef	DC_EXTRUDER
	#include	"heater.h"
#endif


/*
	position tracking
*/

/// \var startpoint
/// \brief target position of last move in queue
TARGET BSS startpoint;

/// \var startpoint_steps
/// \brief target position of last move in queue, expressed in steps
TARGET BSS startpoint_steps;

/// \var current_position
/// \brief actual position of extruder head
/// \todo make current_position = real_position (from endstops) + offset from G28 and friends
TARGET BSS current_position;

/// \var move_state
/// \brief numbers for tracking the current state of movement
MOVE_STATE BSS move_state;

/// \var maximum_feedrate_P
/// \brief maximum allowed feedrate on each axis
static const axes_uint32_t PROGMEM maximum_feedrate_P = {
  MAXIMUM_FEEDRATE_X,
  MAXIMUM_FEEDRATE_Y,
  MAXIMUM_FEEDRATE_Z,
  MAXIMUM_FEEDRATE_E
};

/// \var c0_P
/// \brief Initialization constant for the ramping algorithm. Timer cycles for
///        first step interval.
static const axes_uint32_t PROGMEM c0_P = {
  (uint32_t)((double)F_CPU / SQRT((double)STEPS_PER_M_X * ACCELERATION / 2000.)),
  (uint32_t)((double)F_CPU / SQRT((double)STEPS_PER_M_Y * ACCELERATION / 2000.)),
  (uint32_t)((double)F_CPU / SQRT((double)STEPS_PER_M_Z * ACCELERATION / 2000.)),
  (uint32_t)((double)F_CPU / SQRT((double)STEPS_PER_M_E * ACCELERATION / 2000.))
};

// Period over which we calculate new velocity; should be at least TICK_TIME/2
#define QUANTUM (TICK_TIME*2)

// Acceleration per QUANTUM.
//           mm/s^2     * (s/QUANTUM)^2            *   steps/m   * 1m/1000mm = steps/QUANTUM^2
// Accel = ACCELERATION * QUANTUM(s)^2 * STEPS_PER_M / 1000
//         ACCELERATION * (QUANTUM/F_CPU)^2 * STEPS_PER_M / 1000
//         ACCELERATION * QUANTUM/F_CPU * QUANTUM/F_CPU * STEPS_PER_M / 1000
//         ACCELERATION * QUANTUM * QUANTUM * STEPS_PER_M / F_CPU / F_CPU / 1000
// Normalized to q10.22; allows up to 2^10=1024 in mantissa (steps per 2ms)
#define ACCEL_P_SHIFT 24
// TODO: ASSERT(rampup_steps_to_vmax < 1<<(32-ACCEL_P_SHIFT))
static const axes_uint32_t PROGMEM accel_P = {
  (uint32_t)((((uint64_t)ACCELERATION * STEPS_PER_M_X) <<(ACCEL_P_SHIFT+1)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2,
  (uint32_t)((((uint64_t)ACCELERATION * STEPS_PER_M_Y) <<(ACCEL_P_SHIFT+1)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2,
  (uint32_t)((((uint64_t)ACCELERATION * STEPS_PER_M_Z) <<(ACCEL_P_SHIFT+1)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2,
  (uint32_t)((((uint64_t)ACCELERATION * STEPS_PER_M_E) <<(ACCEL_P_SHIFT+1)) * QUANTUM / F_CPU * QUANTUM / F_CPU / 1000 + 1)/2
};

/*! Set the direction of the 'n' axis
*/
static void set_direction(DDA *dda, enum axis_e n, int32_t delta) {
  uint8_t dir = (delta >= 0) ? 1 : 0;

  if (n == X)
    dda->x_direction = dir;
  else if (n == Y)
    dda->y_direction = dir;
  else if (n == Z)
    dda->z_direction = dir;
  else if (n == E)
    dda->e_direction = dir;
}

/*! Find the direction of the 'n' axis
*/
int8_t get_direction(DDA *dda, enum axis_e n) {
  if ((n == X && dda->x_direction) ||
      (n == Y && dda->y_direction) ||
      (n == Z && dda->z_direction) ||
      (n == E && dda->e_direction))
    return 1;
  else
    return -1;
}

/*! Inititalise DDA movement structures
*/
void dda_init(void) {
	// set up default feedrate
	if (startpoint.F == 0)
		startpoint.F = next_target.target.F = SEARCH_FEEDRATE_Z;
  if (startpoint.e_multiplier == 0)
    startpoint.e_multiplier = next_target.target.e_multiplier = 256;
  if (startpoint.f_multiplier == 0)
    startpoint.f_multiplier = next_target.target.f_multiplier = 256;
}

/*! Distribute a new startpoint to DDA's internal structures without any movement.

	This is needed for example after homing or a G92. The new location must be in startpoint already.
*/
void dda_new_startpoint(void) {
	axes_um_to_steps(startpoint.axis, startpoint_steps.axis);
  startpoint_steps.axis[E] = um_to_steps(startpoint.axis[E], E);
}

/**
  Create a DDA using startpoint, startpoint_steps and a target, save to passed
  location so we can write directly into the queue.

	\param *dda pointer to a dda_queue entry to overwrite
	\param *target the target position of this move

	\ref startpoint the beginning position of this move

	This function does a /lot/ of math. It works out directions for each axis, distance travelled, the time between the first and second step

	It also pre-fills any data that the selected accleration algorithm needs, and can be pre-computed for the whole move.

  This algorithm is the main limiting factor when queuing movements and can
  become a limitation to print speed if there are lots of tiny, fast movements.

 * Regarding lookahead, we can distinguish everything into these cases:
 *
 * 1. Standard movement. To be joined with the previous move.
 * 2. Movement after a pause. This interrupts lookahead, and invalidates
 *    prev_dda and prev_distance.
 * 3. Non-move, e.g. a wait for temp. This also interrupts lookahead and makes
 *    prev_dda and prev_distance invalid. There might be more such cases in the
 *    future, e.g. when heater or fan changes are queued up, too.
 * 4. Nullmove due to no movement expected, e.g. a pure speed change. This
 *    shouldn't interrupt lookahead and be handled af if the change would come
 *    with the next movement.
 * 5. Nullmove due to movement smaller than a single step. Shouldn't interrupt
 *    lookahead either, but this small distance should be added to the next
 *    movement.
 * 6. Lookahead calculation too slow. This is handled in dda_join_moves()
 *    already.
 */
void dda_create(DDA *dda, const TARGET *target) {
  axes_uint32_t delta_um;
  axes_int32_t steps;
	uint32_t	distance, c_limit, c_limit_calc;
  enum axis_e i;
  #ifdef ACCELERATION_RAMPING
  // Number the moves to identify them; allowed to overflow.
  static uint8_t idcnt = 0;
  #endif
  #ifdef LOOKAHEAD
  static DDA* prev_dda = NULL;

  if (prev_dda && prev_dda->done)
    prev_dda = NULL;
  #endif

  // We end at the passed target.
  memcpy(&(dda->endpoint), target, sizeof(TARGET));

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("\nCreate: X %lq  Y %lq  Z %lq  F %lu\n"),
               dda->endpoint.axis[X], dda->endpoint.axis[Y],
               dda->endpoint.axis[Z], dda->endpoint.F);

  // Apply feedrate multiplier.
  if (dda->endpoint.f_multiplier != 256 && ! dda->endstop_check) {
    dda->endpoint.F *= dda->endpoint.f_multiplier;
    dda->endpoint.F += 128;
    dda->endpoint.F /= 256;
  }

  #ifdef LOOKAHEAD
    // Set the start and stop speeds to zero for now = full stops between
    // moves. Also fallback if lookahead calculations fail to finish in time.
    dda->crossF = 0;
    dda->start_steps = 0;
    dda->end_steps = 0;
  #endif
  #ifdef ACCELERATION_RAMPING
    // Give this move an identifier.
    dda->id = idcnt++;
  #endif

  // Handle bot axes. They're subject to kinematics considerations.
  code_axes_to_stepper_axes(&startpoint, target, delta_um, steps);
  for (i = X; i < E; i++) {
    int32_t delta_steps;

    delta_steps = steps[i] - startpoint_steps.axis[i];
    dda->delta[i] = (uint32_t)labs(delta_steps);
    startpoint_steps.axis[i] = steps[i];

    set_direction(dda, i, delta_steps);
  }

  // Handle extruder axes. They act independently from the bots kinematics
  // type, but are subject to other special handling.
  steps[E] = um_to_steps(target->axis[E], E);

  // Apply extrusion multiplier.
  if (target->e_multiplier != 256) {
    steps[E] *= target->e_multiplier;
    steps[E] += 128;
    steps[E] /= 256;
  }

  if ( ! target->e_relative) {
    int32_t delta_steps;

    delta_um[E] = (uint32_t)labs(target->axis[E] - startpoint.axis[E]);
    delta_steps = steps[E] - startpoint_steps.axis[E];
    dda->delta[E] = (uint32_t)labs(delta_steps);
    startpoint_steps.axis[E] = steps[E];

    set_direction(dda, E, delta_steps);
  }
  else {
    // When we get more extruder axes:
    // for (i = E; i < AXIS_COUNT; i++) { ...
    delta_um[E] = (uint32_t)labs(target->axis[E]);
    dda->delta[E] = (uint32_t)labs(steps[E]);
    dda->e_direction = (target->axis[E] >= 0)?1:0;
	}

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("[%ld,%ld,%ld,%ld]"),
               target->axis[X] - startpoint.axis[X], target->axis[Y] - startpoint.axis[Y],
               target->axis[Z] - startpoint.axis[Z], target->axis[E] - startpoint.axis[E]);

  // Admittedly, this looks like it's overcomplicated. Why store three 32-bit
  // values if storing an axis number would be fully sufficient? Well, I'm not
  // sure, but my feeling says that when we achieve true circles and Beziers,
  // we'll have total_steps which matches neither of X, Y, Z or E. Accordingly,
  // keep it for now. --Traumflug
  for (i = X; i < AXIS_COUNT; i++) {
    if (i == X || dda->delta[i] > dda->total_steps) {
      dda->total_steps = dda->delta[i];
      dda->fast_um = delta_um[i];
      dda->fast_axis = i;
    }
  }

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR(" [ts:%lu"), dda->total_steps);

	if (dda->total_steps == 0) {
		dda->nullmove = 1;
    startpoint.F = dda->endpoint.F;
	}
	else {
		// get steppers ready to go
		power_on();
		stepper_enable();
		x_enable();
		y_enable();
    #ifndef Z_AUTODISABLE
      z_enable();
    // #else Z is enabled in dda_start().
    #endif
		e_enable();

		// since it's unusual to combine X, Y and Z changes in a single move on reprap, check if we can use simpler approximations before trying the full 3d approximation.
		if (delta_um[Z] == 0)
			distance = approx_distance(delta_um[X], delta_um[Y]);
		else if (delta_um[X] == 0 && delta_um[Y] == 0)
			distance = delta_um[Z];
		else
			distance = approx_distance_3(delta_um[X], delta_um[Y], delta_um[Z]);

		if (distance < 1)
			distance = delta_um[E];

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
			sersendf_P(PSTR(",ds:%lu"), distance);

    #ifdef	ACCELERATION_TEMPORAL
      // bracket part of this equation in an attempt to avoid overflow:
      // 60 * 16 MHz * 5 mm is > 32 bits
      uint32_t move_duration, md_candidate;

      move_duration = distance * ((60 * F_CPU) / (dda->endpoint.F * 1000UL));
      for (i = X; i < AXIS_COUNT; i++) {
        md_candidate = dda->delta[i] * ((60 * F_CPU) /
                       (pgm_read_dword(&maximum_feedrate_P[i]) * 1000UL));
        if (md_candidate > move_duration)
          move_duration = md_candidate;
      }
		#else
			// pre-calculate move speed in millimeter microseconds per step minute for less math in interrupt context
			// mm (distance) * 60000000 us/min / step (total_steps) = mm.us per step.min
			//   note: um (distance) * 60000 == mm * 60000000
			// so in the interrupt we must simply calculate
			// mm.us per step.min / mm per min (F) = us per step

			// break this calculation up a bit and lose some precision because 300,000um * 60000 is too big for a uint32
			// calculate this with a uint64 if you need the precision, but it'll take longer so routines with lots of short moves may suffer
			// 2^32/6000 is about 715mm which should be plenty

			// changed * 10 to * (F_CPU / 100000) so we can work in cpu_ticks rather than microseconds.
			// timer.c timer_set() routine altered for same reason

			// changed distance * 6000 .. * F_CPU / 100000 to
			//         distance * 2400 .. * F_CPU / 40000 so we can move a distance of up to 1800mm without overflowing
			uint32_t move_duration = ((distance * 2400) / dda->total_steps) * (F_CPU / 40000);
		#endif

		// similarly, find out how fast we can run our axes.
		// do this for each axis individually, as the combined speed of two or more axes can be higher than the capabilities of a single one.
    // TODO: instead of calculating c_min directly, it's probably more simple
    //       to calculate (maximum) move_duration for each axis, like done for
    //       ACCELERATION_TEMPORAL above. This should make re-calculating the
    //       allowed F easier.
    c_limit = 0;
    for (i = X; i < AXIS_COUNT; i++) {
      c_limit_calc = (delta_um[i] * 2400L) /
                     // dda->total_steps * (F_CPU / 40000) /
                     pgm_read_dword(&maximum_feedrate_P[i]);
      if (c_limit_calc > c_limit)
        c_limit = c_limit_calc;
    }
    c_limit = c_limit / dda->total_steps * (F_CPU / 40000);

		#ifdef ACCELERATION_REPRAP
		// c is initial step time in IOclk ticks
    dda->c = move_duration / startpoint.F;
    if (dda->c < c_limit)
      dda->c = c_limit;
    dda->end_c = move_duration / dda->endpoint.F;
    if (dda->end_c < c_limit)
      dda->end_c = c_limit;

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
      sersendf_P(PSTR(",md:%lu,c:%lu"), move_duration, dda->c);

    if (dda->c != dda->end_c) {
			uint32_t stF = startpoint.F / 4;
			uint32_t enF = dda->endpoint.F / 4;
			// now some constant acceleration stuff, courtesy of http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time
			uint32_t ssq = (stF * stF);
			uint32_t esq = (enF * enF);
			int32_t dsq = (int32_t) (esq - ssq) / 4;

			uint8_t msb_ssq = msbloc(ssq);
			uint8_t msb_tot = msbloc(dda->total_steps);

			// the raw equation WILL overflow at high step rates, but 64 bit math routines take waay too much space
			// at 65536 mm/min (1092mm/s), ssq/esq overflows, and dsq is also close to overflowing if esq/ssq is small
			// but if ssq-esq is small, ssq/dsq is only a few bits
			// we'll have to do it a few different ways depending on the msb locations of each
			if ((msb_tot + msb_ssq) <= 30) {
				// we have room to do all the multiplies first
				if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
					serial_writechar('A');
				dda->n = ((int32_t) (dda->total_steps * ssq) / dsq) + 1;
			}
			else if (msb_tot >= msb_ssq) {
				// total steps has more precision
				if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
					serial_writechar('B');
				dda->n = (((int32_t) dda->total_steps / dsq) * (int32_t) ssq) + 1;
			}
			else {
				// otherwise
				if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
					serial_writechar('C');
				dda->n = (((int32_t) ssq / dsq) * (int32_t) dda->total_steps) + 1;
			}

			if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("\n{DDA:CA end_c:%lu, n:%ld, md:%lu, ssq:%lu, esq:%lu, dsq:%lu, msbssq:%u, msbtot:%u}\n"), dda->end_c, dda->n, move_duration, ssq, esq, dsq, msb_ssq, msb_tot);

			dda->accel = 1;
		}
		else
			dda->accel = 0;
		#elif defined ACCELERATION_RAMPING
      dda->c_min = move_duration / dda->endpoint.F;
      if (dda->c_min < c_limit) {
        dda->c_min = c_limit;
        dda->endpoint.F = move_duration / dda->c_min;
      }
      dda->vmax = muldiv(QUANTUM, 1UL << ACCEL_P_SHIFT, dda->c_min);

      // Lookahead can deal with 16 bits ( = 1092 mm/s), only.
      if (dda->endpoint.F > 65535)
        dda->endpoint.F = 65535;

      // Acceleration ramps are based on the fast axis, not the combined speed.
      dda->rampup_steps =
        acc_ramp_len(muldiv(dda->fast_um, dda->endpoint.F, distance),
                     dda->fast_axis);

      if (dda->rampup_steps > dda->total_steps / 2)
        dda->rampup_steps = dda->total_steps / 2;
      dda->rampdown_steps = dda->total_steps - dda->rampup_steps;

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

      #ifdef LOOKAHEAD
        dda->distance = distance;
        dda_find_crossing_speed(prev_dda, dda);
        // TODO: this should become a reverse-stepping through the existing
        //       movement queue to allow higher speeds for short moves.
        //       dda_find_crossing_speed() is required only once.
        dda_join_moves(prev_dda, dda);
        dda->n = dda->start_steps;
        if (dda->n == 0)
          dda->c = pgm_read_dword(&c0_P[dda->fast_axis]);
        else
          dda->c = ((pgm_read_dword(&c0_P[dda->fast_axis])>>2) *
                    int_inv_sqrt(dda->n)) >> 11;
        if (dda->c < dda->c_min)
          dda->c = dda->c_min;
      #else
        dda->n = 0;
        dda->c = pgm_read_dword(&c0_P[dda->fast_axis]);
        dda->steps = 1;
      #endif

		#elif defined ACCELERATION_TEMPORAL
			// TODO: calculate acceleration/deceleration for each axis
      for (i = X; i < AXIS_COUNT; i++) {
        dda->step_interval[i] = 0xFFFFFFFF;
        if (dda->delta[i])
          dda->step_interval[i] = move_duration / dda->delta[i];
      }

      dda->c = 0xFFFFFFFF;
      dda->axis_to_step = X; // Safety value
      for (i = X; i < AXIS_COUNT; i++) {
        if (dda->step_interval[i] < dda->c) {
          dda->axis_to_step = i;
          dda->c = dda->step_interval[i];
        }
      }

		#else
      dda->c = move_duration / dda->endpoint.F;
      if (dda->c < c_limit)
        dda->c = c_limit;
		#endif

    // next dda starts where we finish
    memcpy(&startpoint, &dda->endpoint, sizeof(TARGET));
    #ifdef LOOKAHEAD
      prev_dda = dda;
    #endif
	} /* ! dda->total_steps == 0 */

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("] }\n"));
}

/** Start a prepared DDA

  \param *dda Pointer to entry in the movement queue to start.

  This function actually begins the move described by the passed DDA entry.
  Called from both, inside and outside of interrupts.
*/
void dda_start(DDA *dda) {

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("Start: X %lq  Y %lq  Z %lq  F %lu\n"),
               dda->endpoint.axis[X], dda->endpoint.axis[Y],
               dda->endpoint.axis[Z], dda->endpoint.F);

  // Get ready to go.
  psu_timeout = 0;
  #ifdef Z_AUTODISABLE
    if (dda->delta[Z])
      z_enable();
  #endif
  if (dda->endstop_check)
    endstops_on();

  // Set direction outputs.
  x_direction(dda->x_direction);
  y_direction(dda->y_direction);
  z_direction(dda->z_direction);
  e_direction(dda->e_direction);

  #ifdef DC_EXTRUDER
    if (dda->delta[E])
      heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
  #endif

  // Initialise state variables.
  move_state.counter[X] = move_state.counter[Y] = move_state.counter[Z] = \
    move_state.counter[E] = -(dda->total_steps >> 1);
  move_state.endstop_stop = 0;
  memcpy(&move_state.steps[X], &dda->delta[X], sizeof(uint32_t) * 4);
  #ifdef ACCELERATION_RAMPING
    // This is constant and we could read it directly in dda_clock every time,
    // but we intend to make acceleration a non-constant function someday. This
    // is why we keep the current accel value in the dda.
    move_state.accel_per_tick = pgm_read_dword(&accel_P[dda->fast_axis]);

    move_state.head = move_state.tail = 0;
    for (int i = 0; i < SUB_MOVE_QUEUE_SIZE; i++)
      move_state.next_n[i] = 0;
    move_state.curr_c = dda->c;

    // Biasing the remainder causes us to step when the integral rounds-up to the next step.  This follows
    // the method used in the original stepper-motor step estimation article.  But is it reasonable to do
    // this or should we wait until something closer to the actual step-time occurs? This seems to work
    // and is least disruptive, so keep it for now.  But set this remainder to something smaller (like 0)
    // for better step-time accuracy.

    //#define USE_BIAS
    #ifdef USE_BIAS
    move_state.remainder = (1ULL<<ACCEL_P_SHIFT)/2;
    #else
    move_state.remainder = (1ULL<<ACCEL_P_SHIFT);
    #endif

    // Biasing the velocity by 1/2 acceleration lets us easily represent the average velocity for each
    // QUANTUM without any extra math.
    #ifdef USE_BIAS
    move_state.velocity = move_state.accel_per_tick/2;  // TODO: Use dda->start_velocity here
    #else
    // Calculate velocity at first step: v = v0 + a * t
    move_state.velocity = muldiv(move_state.accel_per_tick, dda->c, QUANTUM);
    #endif
    dda->n = move_state.position = 1;

    move_state.step_no = 0;
    move_state.accel = 1;

    // if (dda->c > QUANTUM) {
    //   uint32_t n = dda->c / QUANTUM;
    //   move_state.velocity = move_state.accel_per_tick * n;
    //   move_state.remainder = move_state.accel_per_tick * (n * (n+1) ) / 2;
    //   move_state.step_no = move_state.position = dda->steps;
    // }
  #endif
  #ifdef ACCELERATION_TEMPORAL
    move_state.time[X] = move_state.time[Y] = \
      move_state.time[Z] = move_state.time[E] = 0UL;
  #endif

  // Ensure this DDA starts.
  dda->live = 1;

  // Set timeout for first step.
  timer_set(dda->c, 0);
}

/**
  \brief Do per-step movement maintenance.

  \param *dda the current move

  \details Most important task here is to update the Bresenham algorithm and
  to generate step pulses accordingly, this guarantees geometrical accuracy
  of the movement. Other tasks, like acceleration calculations, are moved
  into dda_clock() as much as possible.

  This is called from our timer interrupt every time a step needs to occur.
  Keep it as simple and fast as possible, this is most critical for the
  achievable step frequency.

  Note: it was tried to do this in loops instead of straight, repeating code.
        However, this resulted in at least 16% performance loss, no matter
        how it was done. On how to measure, see commit "testcases: Add
        config.h". On the various tries and measurement results, see commits
        starting with "DDA: Move axis calculations into loops, part 6".
*/
void dda_step(DDA *dda) {

  // sersendf_P(PSTR("   DDA_STEP   head=%u  tail=%u   id=%u\n"), move_state.head, move_state.tail, dda->id);
  #if ! defined ACCELERATION_TEMPORAL
    if (move_state.steps[X]) {
      move_state.counter[X] -= dda->delta[X];
      if (move_state.counter[X] < 0) {
        move_state.counter[X] += dda->total_steps;
        x_step();
        move_state.steps[X]--;
      }
    }
    if (move_state.steps[Y]) {
      move_state.counter[Y] -= dda->delta[Y];
      if (move_state.counter[Y] < 0) {
        move_state.counter[Y] += dda->total_steps;
        y_step();
        move_state.steps[Y]--;
      }
    }
    if (move_state.steps[Z]) {
      move_state.counter[Z] -= dda->delta[Z];
      if (move_state.counter[Z] < 0) {
        move_state.counter[Z] += dda->total_steps;
        z_step();
        move_state.steps[Z]--;
      }
    }
    if (move_state.steps[E]) {
      move_state.counter[E] -= dda->delta[E];
      if (move_state.counter[E] < 0) {
        move_state.counter[E] += dda->total_steps;
        e_step();
        move_state.steps[E]--;
      }
    }
    move_state.step_no++;
    if (--dda->steps == 0) {
      dda->steps = move_state.next_n[move_state.head];
      if (dda->steps) {
        dda->dc = move_state.next_dc[move_state.head];
        // sersendf_P(PSTR("\n<< DDA %u. c=%lu  dc=%ld  n=%lu\n"), move_state.head, dda->c, dda->dc, dda->steps);
        move_state.next_n[move_state.head++] = 0;
        if (move_state.head == SUB_MOVE_QUEUE_SIZE)
          move_state.head = 0;
      } else {
        // This is bad.  It means we will idle at the last commanded speed and send more steps than commanded.
        // sersendf_P(PSTR("\n-- DDA underflow with %lu steps remaining\n"), dda->total_steps - move_state.  step_no);
        dda->steps++;
      }
    }
    dda->c += dda->dc;
  #endif

	#ifdef ACCELERATION_REPRAP
		// linear acceleration magic, courtesy of http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time
		if (dda->accel) {
      if ((dda->c > dda->end_c) && (dda->n > 0)) {
				uint32_t new_c = dda->c - (dda->c * 2) / dda->n;
        if (new_c <= dda->c && new_c > dda->end_c) {
					dda->c = new_c;
					dda->n += 4;
				}
				else
          dda->c = dda->end_c;
			}
      else if ((dda->c < dda->end_c) && (dda->n < 0)) {
				uint32_t new_c = dda->c + ((dda->c * 2) / -dda->n);
        if (new_c >= dda->c && new_c < dda->end_c) {
					dda->c = new_c;
					dda->n += 4;
				}
				else
          dda->c = dda->end_c;
			}
      else if (dda->c != dda->end_c) {
        dda->c = dda->end_c;
			}
			// else we are already at target speed
		}
	#endif

  #ifdef ACCELERATION_TEMPORAL
    /** How is this ACCELERATION TEMPORAL expected to work?

      All axes work independently of each other, as if they were on four
      different, synchronized timers. As we have not enough suitable timers,
      we have to share one for all axes.

      To do this, each axis maintains the time of its last step in
      move_state.time[]. This time is updated as the step is done, see early
      in dda_step(). To find out which axis is the next one to step, the time
      of each axis' next step is compared to the time of the step just done.
      Zero means this actually is the axis just stepped, the smallest value > 0
      wins.

      One problem undoubtedly arising is, steps should sometimes be done at
      {almost,exactly} the same time. We trust the timer to deal properly with
      very short or even zero periods. If a step can't be done in time, the
      timer shall do the step as soon as possible and compensate for the delay
      later. In turn we promise here to send a maximum of four such
      short-delays consecutively and to give sufficient time on average.
    */
    // This is the time which led to this call of dda_step().
    move_state.last_time = move_state.time[dda->axis_to_step] +
                           dda->step_interval[dda->axis_to_step];

    do {
      uint32_t c_candidate;
      enum axis_e i;

      if (dda->axis_to_step == X) {
        x_step();
        move_state.steps[X]--;
        move_state.time[X] += dda->step_interval[X];
      }
      if (dda->axis_to_step == Y) {
        y_step();
        move_state.steps[Y]--;
        move_state.time[Y] += dda->step_interval[Y];
      }
      if (dda->axis_to_step == Z) {
        z_step();
        move_state.steps[Z]--;
        move_state.time[Z] += dda->step_interval[Z];
      }
      if (dda->axis_to_step == E) {
        e_step();
        move_state.steps[E]--;
        move_state.time[E] += dda->step_interval[E];
      }
      unstep();

      // Find the next stepper to step.
      dda->c = 0xFFFFFFFF;
      for (i = X; i < AXIS_COUNT; i++) {
        if (move_state.steps[i]) {
          c_candidate = move_state.time[i] + dda->step_interval[i] -
                        move_state.last_time;
          if (c_candidate < dda->c) {
            dda->axis_to_step = i;
            dda->c = c_candidate;
          }
        }
      }

      // No stepper to step found? Then we're done.
      if (dda->c == 0xFFFFFFFF) {
        dda->live = 0;
        dda->done = 1;
        break;
      }
    } while (timer_set(dda->c, 1));

  #endif /* ACCELERATION_TEMPORAL */

  // If there are no steps left or an endstop stop happened, we have finished.
  //
  // TODO: with ACCELERATION_TEMPORAL this duplicates some code. See where
  //       dda->live is zero'd, about 10 lines above.
  #if ! defined ACCELERATION_TEMPORAL
    if (move_state.step_no >= dda->total_steps ||
        (move_state.endstop_stop && dda->n <= 0))
  #else
    if (move_state.steps[X] == 0 && move_state.steps[Y] == 0 &&
        move_state.steps[Z] == 0 && move_state.steps[E] == 0)
  #endif
  {
		dda->live = 0;
    dda->done = 1;
    if (dda->steps) {
      // Note: currently we expect to have leftover steps because the feeder never stops feeding.
      // Maybe we should fix that, but it is no harm for now.
      // sersendf_P(PSTR("\n-- DDA has %lu leftover steps in its todo list.\n"), dda->steps);
    }
    #ifdef LOOKAHEAD
    // If look-ahead was using this move, it could have missed our activation:
    // make sure the ids do not match.
    dda->id--;
    #endif
		#ifdef	DC_EXTRUDER
			heater_set(DC_EXTRUDER, 0);
		#endif
    #ifdef Z_AUTODISABLE
      // Z stepper is only enabled while moving.
      z_disable();
    #endif

    // No need to restart timer here.
    // After having finished, dda_start() will do it.
	}
  else {
		psu_timeout = 0;
    #ifndef ACCELERATION_TEMPORAL
      timer_set(dda->c, 0);
    #endif
  }

	// turn off step outputs, hopefully they've been on long enough by now to register with the drivers
	// if not, too bad. or insert a (very!) small delay here, or fire up a spare timer or something.
	// we also hope that we don't step before the drivers register the low- limit maximum speed if you think this is a problem.
	unstep();
}

/*! Do regular movement maintenance.

  This should be called pretty often, like once every 1 or 2 milliseconds.

  Currently, this is checking the endstops and doing acceleration maths. These
  don't need to be checked/recalculated on every single step, so this code
  can be moved out of the highly time critical dda_step(). At high precision
  (slow) searches of the endstop, this function is called more often than
  dda_step() anyways.

  In the future, arc movement calculations might go here, too. Updating
  movement direction 500 times a second is easily enough for smooth and
  accurate curves!
*/
void dda_clock() {
  DDA *dda;
  static DDA *last_dda = NULL;
  uint8_t endstop_trigger = 0;
  #ifdef ACCELERATION_RAMPING
  uint8_t current_id ;
  #endif

  ATOMIC_START
    dda = mb_tail_dda;
  ATOMIC_END
  if (dda != last_dda) {
    move_state.debounce_count_x =
    move_state.debounce_count_z =
    move_state.debounce_count_y = 0;
    last_dda = dda;
  }

  if (dda == NULL)
    return;

  // Caution: we mangle step counters here without locking interrupts. This
  //          means, we trust dda isn't changed behind our back, which could
  //          in principle (but rarely) happen if endstops are checked not as
  //          endstop search, but as part of normal operations.
  if (dda->endstop_check && ! move_state.endstop_stop) {
    #ifdef X_MIN_PIN
    if (dda->endstop_check & 0x01) {
      if (x_min() == dda->endstop_stop_cond)
        move_state.debounce_count_x++;
      else
        move_state.debounce_count_x = 0;
      endstop_trigger = move_state.debounce_count_x >= ENDSTOP_STEPS;
    }
    #endif
    #ifdef X_MAX_PIN
    if (dda->endstop_check & 0x02) {
      if (x_max() == dda->endstop_stop_cond)
        move_state.debounce_count_x++;
      else
        move_state.debounce_count_x = 0;
      endstop_trigger = move_state.debounce_count_x >= ENDSTOP_STEPS;
    }
    #endif

    #ifdef Y_MIN_PIN
    if (dda->endstop_check & 0x04) {
      if (y_min() == dda->endstop_stop_cond)
        move_state.debounce_count_y++;
      else
        move_state.debounce_count_y = 0;
      endstop_trigger = move_state.debounce_count_y >= ENDSTOP_STEPS;
    }
    #endif
    #ifdef Y_MAX_PIN
    if (dda->endstop_check & 0x08) {
      if (y_max() == dda->endstop_stop_cond)
        move_state.debounce_count_y++;
      else
        move_state.debounce_count_y = 0;
      endstop_trigger = move_state.debounce_count_y >= ENDSTOP_STEPS;
    }
    #endif

    #ifdef Z_MIN_PIN
    if (dda->endstop_check & 0x10) {
      if (z_min() == dda->endstop_stop_cond)
        move_state.debounce_count_z++;
      else
        move_state.debounce_count_z = 0;
      endstop_trigger = move_state.debounce_count_z >= ENDSTOP_STEPS;
    }
    #endif
    #ifdef Z_MAX_PIN
    if (dda->endstop_check & 0x20) {
      if (z_max() == dda->endstop_stop_cond)
        move_state.debounce_count_z++;
      else
        move_state.debounce_count_z = 0;
      endstop_trigger = move_state.debounce_count_z >= ENDSTOP_STEPS;
    }
    #endif

    // If an endstop is definitely triggered, stop the movement.
    if (endstop_trigger) {
      #ifdef ACCELERATION_RAMPING
        // For always smooth operations, don't halt apruptly,
        // but start deceleration here.
        ATOMIC_START
          move_state.endstop_stop = 1;
          if (move_state.step_no < dda->rampup_steps)  // still accelerating
            dda->total_steps = move_state.step_no * 2;
          else
            // A "-=" would overflow earlier.
            dda->total_steps = dda->total_steps - dda->rampdown_steps +
                               move_state.step_no;
          dda->rampdown_steps = move_state.step_no;
        ATOMIC_END
        // Not atomic, because not used in dda_step().
        dda->rampup_steps = 0; // in case we're still accelerating
      #else
        dda->live = 0;
      #endif

      endstops_off();
    }
  } /* ! move_state.endstop_stop */

  #ifdef ACCELERATION_RAMPING

    // For maths about stepper speed profiles, see
    // http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time
    // and http://www.atmel.com/images/doc8017.pdf (Atmel app note AVR446)
    ATOMIC_START
      current_id = dda->id;
      // All other variables are read-only or unused in dda_step(),
      // so no need for atomic operations.
    ATOMIC_END

    // Plan ahead until the movement queue is full
    while (move_state.position < dda->total_steps &&
           current_id == dda->id &&
           move_state.next_n[move_state.tail] == 0) {
      uint8_t tail = move_state.tail;
      uint32_t move_c, curr_c;
      int32_t move_dc;
      uint32_t velocity = move_state.velocity;
      uint32_t remainder;
      uint32_t dx = 0;

      remainder = move_state.remainder;
      uint32_t step_no = move_state.position;
      uint32_t v0 = velocity;
      #ifdef SIMULATOR
        uint32_t i = 0;
      #endif

      while ( dx==0 ) {

        if (move_state.accel) {
        //   uint32_t old_rem = remainder;
          remainder += velocity;
          dx = remainder >> ACCEL_P_SHIFT ;
          step_no += dx;

        //   sersendf_P(PSTR(" $$> vel=%lu  rem=%lu (%lu)  dx=%lu (%lu)\n"), velocity, remainder, old_rem, dx, step_no);

          uint32_t vnext = velocity + move_state.accel_per_tick;
          // Almost reached mid-point of move or max velocity; time to cruise
          if (step_no*2 >= dda->total_steps || vnext > dda->vmax) {
            move_state.accel = 0;
            dx = dda->total_steps - step_no*2 + dx;
          } else {
            velocity = vnext;
          }
        }
        else {//if (move_state.phase >= PHASE_CRUISE) {
          remainder -= velocity;
          remainder &= (1ULL<<ACCEL_P_SHIFT) - 1;

          dx = (remainder + velocity) >> ACCEL_P_SHIFT ;
          step_no += dx;

        //   sersendf_P(PSTR(" $$< vel=%lu  rem=%lu (%lu)  dx=%lu (%lu)\n"), velocity, remainder, (remainder + velocity) , dx, step_no);

          if (velocity > move_state.accel_per_tick)
            velocity -= move_state.accel_per_tick;

          while (remainder >= velocity && velocity > move_state.accel_per_tick) {
                  #ifdef SIMULATOR
                    sersendf_P(PSTR("   %u. vel=%lu  vmax=%lu  dx=%lu (%lu)  tsteps=%lu  rem=%lu\n"),
                      ++i, velocity, dda->vmax, dx, step_no, dda->total_steps, remainder);
                  #endif
            remainder -= velocity;
            remainder &= (1ULL<<ACCEL_P_SHIFT) - 1;
            velocity -= move_state.accel_per_tick;
          }
        }

        #ifdef SIMULATOR
          sersendf_P(PSTR("   %u. vel=%lu  vmax=%lu  dx=%lu (%lu)  tsteps=%lu  rem=%lu\n"),
             ++i, velocity, dda->vmax, dx, step_no, dda->total_steps, remainder);
        #endif
      }

      // Mask off mantissa
      remainder &= (1ULL<<ACCEL_P_SHIFT) - 1;

      // if (recalc_speed)
      {
        // If just 1 or 2 steps per quantum, use average velocity since the last movement (v0).
        // Otherwise, use the current velocity at the end of this quantum for the many steps expected.
        // TODO: Remove this distinction; it seems like we can always use the average without issue.
        // if (dx < 3)
          move_c = muldiv(QUANTUM , 2*1UL<<ACCEL_P_SHIFT, velocity+v0);
        // else
        //   move_c = muldiv(QUANTUM , 1UL<<ACCEL_P_SHIFT, velocity);
        // move_c = (QUANTUM << 16) / (velocity >> (ACCEL_P_SHIFT - 16));

        // TODO: This shouldn't happen, ideally.  How can we prevent it?
        if (move_c > pgm_read_dword(&c0_P[dda->fast_axis]))
          move_c = pgm_read_dword(&c0_P[dda->fast_axis]);

        if (move_c < dda->c_min) {
          move_c = dda->c_min;
        }
        move_dc = move_c - move_state.curr_c + dx/2;
        move_dc /= (int32_t)dx;
        curr_c = move_state.curr_c + move_dc * dx;
      }

      #ifdef SIMULATOR
        sersendf_P(PSTR("  %lu  S#=%lu, q=%u  State=%d  vel=%lu,%lu  c=%lu  vmax=%lu  dx=%lu (%lu)  tsteps=%lu  rem=%lu\n"),
          move_c, step_no, tail, move_state.accel, v0,velocity, move_c, dda->vmax, dx, step_no, dda->total_steps, (1000*remainder)>>ACCEL_P_SHIFT);
      // sersendf_P(PSTR("   %u. vel=%lu  vmax=%lu  dx=%lu (%lu)  tsteps=%lu  rem=%lu\n"),
      //    ++i, velocity, dda->vmax, dx, move_step_no+dx, dda->total_steps, remainder);
      #endif
      // sersendf_P(PSTR("  %lu  S#=%lu, q=%u  State=%d  elapsed=%lu  vel=%lu,%lu  c=%lu  vmax=%lu  dx=%lu (%lu)  tsteps=%lu  rem=%lu\n"),
      //   move_c, step_no, tail, move_state.phase, elapsed , v0,velocity, move_c, dda->vmax, dx, move_step_no, dda->total_steps, (1000*remainder)>>ACCEL_P_SHIFT);

      // Write results.
      ATOMIC_START
        /**
          Apply new n & c values only if dda didn't change underneath us. It
          is possible for dda to be modified since fetching values in the
          ATOMIC above, e.g. when a new dda becomes live.

          In case such a change happened, values in the new dda are more
          recent than our calculation here, anyways.
        */
        if (current_id == dda->id) {
          move_state.next_dc[tail] = move_dc;
          move_state.next_n[tail] = dx;
          move_state.curr_c = curr_c;
          // sersendf_P(PSTR("\n>> DDA %u. c=%lu  dc=%ld  n=%lu\n"), tail, curr_c, move_dc, dx);
          move_state.velocity = velocity;

          move_state.remainder = remainder;
          move_state.position += dx;
        }

      ATOMIC_END

      if (++tail == SUB_MOVE_QUEUE_SIZE) tail = 0;
      move_state.tail = tail;
    }
  #endif
}

/// update global current_position struct
void update_current_position() {
  DDA *dda = mb_tail_dda;
  enum axis_e i;

  static const axes_uint32_t PROGMEM steps_per_m_P = {
    STEPS_PER_M_X,
    STEPS_PER_M_Y,
    STEPS_PER_M_Z,
    STEPS_PER_M_E
  };

  if (dda != NULL) {
    uint32_t axis_steps, axis_um;

    for (i = X; i < AXIS_COUNT; i++) {
      #if ! defined ACCELERATION_TEMPORAL
        axis_steps = muldiv(dda->total_steps - move_state.step_no,
                            dda->delta[i], dda->total_steps);
      #else
        axis_steps = move_state.steps[i];
      #endif
      axis_um = muldiv(axis_steps, 1000000, pgm_read_dword(&steps_per_m_P[i]));
      current_position.axis[i] =
        dda->endpoint.axis[i] - (int32_t)get_direction(dda, i) * axis_um;
    }

    if (dda->endpoint.e_relative) {
      // We support only one extruder, so axis_um is already valid.
      current_position.axis[E] = axis_um;
    }

    current_position.F = dda->endpoint.F;
	}
  else {
    memcpy(&current_position, &startpoint, sizeof(TARGET));
	}
}
