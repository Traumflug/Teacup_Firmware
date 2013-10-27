#include	"dda.h"

/** \file
	\brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include	<string.h>
#include	<stdlib.h>
#include	<math.h>
#include	<avr/interrupt.h>

#include	"dda_maths.h"
#include	"dda_lookahead.h"
#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"
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
#include	"delay.h"

#if defined ACCELERATION_RAMPING && defined ACCELERATION_CLOCK
  #error Cant define ACCELERATION_RAMPING and ACCELERATION_CLOCK at the same time.
#endif

/*
	position tracking
*/

/// \var startpoint
/// \brief target position of last move in queue
TARGET startpoint __attribute__ ((__section__ (".bss")));

/// \var startpoint_steps
/// \brief target position of last move in queue, expressed in steps
TARGET startpoint_steps __attribute__ ((__section__ (".bss")));

/// \var current_position
/// \brief actual position of extruder head
/// \todo make current_position = real_position (from endstops) + offset from G28 and friends
TARGET current_position __attribute__ ((__section__ (".bss")));

/// \var move_state
/// \brief numbers for tracking the current state of movement
MOVE_STATE move_state __attribute__ ((__section__ (".bss")));

/*! Inititalise DDA movement structures
*/
void dda_init(void) {
	// set up default feedrate
	if (startpoint.F == 0)
		startpoint.F = next_target.target.F = SEARCH_FEEDRATE_Z;

	#ifdef ACCELERATION_RAMPING
		move_state.n = 1;
		move_state.c = ((uint32_t)((double)F_CPU / sqrt((double)(STEPS_PER_M_X * ACCELERATION / 1000.)))) << 8;
	#endif
}

/*! Distribute a new startpoint to DDA's internal structures without any movement.

	This is needed for example after homing or a G92. The new location must be in startpoint already.
*/
void dda_new_startpoint(void) {
	startpoint_steps.X = um_to_steps_x(startpoint.X);
	startpoint_steps.Y = um_to_steps_y(startpoint.Y);
	startpoint_steps.Z = um_to_steps_z(startpoint.Z);
	startpoint_steps.E = um_to_steps_e(startpoint.E);
}

/*! CREATE a dda given current_position and a target, save to passed location so we can write directly into the queue
	\param *dda pointer to a dda_queue entry to overwrite
	\param *target the target position of this move

	\ref startpoint the beginning position of this move

	This function does a /lot/ of math. It works out directions for each axis, distance travelled, the time between the first and second step

	It also pre-fills any data that the selected accleration algorithm needs, and can be pre-computed for the whole move.

	This algorithm is probably the main limiting factor to print speed in terms of firmware limitations
*/
void dda_create(DDA *dda, TARGET *target, DDA *prev_dda) {
	uint32_t	steps, x_delta_um, y_delta_um, z_delta_um, e_delta_um;
	uint32_t	distance, c_limit, c_limit_calc;
  #ifdef LOOKAHEAD
  // Number the moves to identify them; allowed to overflow.
  static uint8_t idcnt = 0;
  #endif

	// initialise DDA to a known state
	dda->allflags = 0;

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("\n{DDA_CREATE: ["));

	// we end at the passed target
	memcpy(&(dda->endpoint), target, sizeof(TARGET));

  #ifdef LOOKAHEAD
  // Set the start and stop speeds to zero for now = full stops between
  // moves. Also fallback if lookahead calculations fail to finish in time.
  dda->F_start = 0;
  dda->F_end = 0;
  // Give this move an identifier.
  dda->id = idcnt++;
  #endif

// TODO TODO: We should really make up a loop for all axes.
//            Think of what happens when a sixth axis (multi colour extruder)
//            appears?
	x_delta_um = (uint32_t)labs(target->X - startpoint.X);
	y_delta_um = (uint32_t)labs(target->Y - startpoint.Y);
	z_delta_um = (uint32_t)labs(target->Z - startpoint.Z);

	steps = um_to_steps_x(target->X);
	dda->x_delta = labs(steps - startpoint_steps.X);
	startpoint_steps.X = steps;
	steps = um_to_steps_y(target->Y);
	dda->y_delta = labs(steps - startpoint_steps.Y);
	startpoint_steps.Y = steps;
	steps = um_to_steps_z(target->Z);
	dda->z_delta = labs(steps - startpoint_steps.Z);
	startpoint_steps.Z = steps;

	dda->x_direction = (target->X >= startpoint.X)?1:0;
	dda->y_direction = (target->Y >= startpoint.Y)?1:0;
	dda->z_direction = (target->Z >= startpoint.Z)?1:0;

	if (target->e_relative) {
		e_delta_um = labs(target->E);
		dda->e_delta = labs(um_to_steps_e(target->E));
		dda->e_direction = (target->E >= 0)?1:0;
	}
	else {
		e_delta_um = (uint32_t)labs(target->E - startpoint.E);
		steps = um_to_steps_e(target->E);
		dda->e_delta = labs(steps - startpoint_steps.E);
		startpoint_steps.E = steps;
		dda->e_direction = (target->E >= startpoint.E)?1:0;
	}

  #ifdef LOOKAHEAD
  // Also displacements in micrometers, but for the lookahead alogrithms.
  dda->delta.X = target->X - startpoint.X;
  dda->delta.Y = target->Y - startpoint.Y;
  dda->delta.Z = target->Z - startpoint.Z;
  dda->delta.E = target->e_relative ? target->E : target->E - startpoint.E;
  #endif

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		sersendf_P(PSTR("%ld,%ld,%ld,%ld] ["), target->X - startpoint.X, target->Y - startpoint.Y, target->Z - startpoint.Z, target->E - startpoint.E);

	dda->total_steps = dda->x_delta;
	if (dda->y_delta > dda->total_steps)
		dda->total_steps = dda->y_delta;
	if (dda->z_delta > dda->total_steps)
		dda->total_steps = dda->z_delta;
	if (dda->e_delta > dda->total_steps)
		dda->total_steps = dda->e_delta;

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		sersendf_P(PSTR("ts:%lu"), dda->total_steps);

	if (dda->total_steps == 0) {
		dda->nullmove = 1;
	}
	else {
		// get steppers ready to go
		power_on();
		stepper_enable();
		x_enable();
		y_enable();
		// Z is enabled in dda_start()
		e_enable();

		// since it's unusual to combine X, Y and Z changes in a single move on reprap, check if we can use simpler approximations before trying the full 3d approximation.
		if (z_delta_um == 0)
			distance = approx_distance(x_delta_um, y_delta_um);
		else if (x_delta_um == 0 && y_delta_um == 0)
			distance = z_delta_um;
		else
			distance = approx_distance_3(x_delta_um, y_delta_um, z_delta_um);

		if (distance < 2)
			distance = e_delta_um;

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
			sersendf_P(PSTR(",ds:%lu"), distance);

		#ifdef	ACCELERATION_TEMPORAL
			// bracket part of this equation in an attempt to avoid overflow: 60 * 16MHz * 5mm is >32 bits
			uint32_t move_duration, md_candidate;

			move_duration = distance * ((60 * F_CPU) / (target->F * 1000UL));
			md_candidate = dda->x_delta * ((60 * F_CPU) / (MAXIMUM_FEEDRATE_X * 1000UL));
			if (md_candidate > move_duration)
				move_duration = md_candidate;
			md_candidate = dda->y_delta * ((60 * F_CPU) / (MAXIMUM_FEEDRATE_Y * 1000UL));
			if (md_candidate > move_duration)
				move_duration = md_candidate;
			md_candidate = dda->z_delta * ((60 * F_CPU) / (MAXIMUM_FEEDRATE_Z * 1000UL));
			if (md_candidate > move_duration)
				move_duration = md_candidate;
			md_candidate = dda->e_delta * ((60 * F_CPU) / (MAXIMUM_FEEDRATE_E * 1000UL));
			if (md_candidate > move_duration)
				move_duration = md_candidate;
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
			// timer.c setTimer() routine altered for same reason

			// changed distance * 6000 .. * F_CPU / 100000 to
			//         distance * 2400 .. * F_CPU / 40000 so we can move a distance of up to 1800mm without overflowing
			uint32_t move_duration = ((distance * 2400) / dda->total_steps) * (F_CPU / 40000);
		#endif

		// similarly, find out how fast we can run our axes.
		// do this for each axis individually, as the combined speed of two or more axes can be higher than the capabilities of a single one.
		c_limit = 0;
		// check X axis
		c_limit_calc = ((x_delta_um * 2400L) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_X) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check Y axis
		c_limit_calc = ((y_delta_um * 2400L) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_Y) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check Z axis
		c_limit_calc = ((z_delta_um * 2400L) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_Z) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check E axis
		c_limit_calc = ((e_delta_um * 2400L) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_E) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;

		#ifdef ACCELERATION_REPRAP
		// c is initial step time in IOclk ticks
		dda->c = (move_duration / startpoint.F) << 8;
		if (dda->c < c_limit)
			dda->c = c_limit;
		dda->end_c = (move_duration / target->F) << 8;
		if (dda->end_c < c_limit)
			dda->end_c = c_limit;

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
			sersendf_P(PSTR(",md:%lu,c:%lu"), move_duration, dda->c >> 8);

		if (dda->c != dda->end_c) {
			uint32_t stF = startpoint.F / 4;
			uint32_t enF = target->F / 4;
			// now some constant acceleration stuff, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
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
				sersendf_P(PSTR("\n{DDA:CA end_c:%lu, n:%ld, md:%lu, ssq:%lu, esq:%lu, dsq:%lu, msbssq:%u, msbtot:%u}\n"), dda->end_c >> 8, dda->n, move_duration, ssq, esq, dsq, msb_ssq, msb_tot);

			dda->accel = 1;
		}
		else
			dda->accel = 0;
		#elif defined ACCELERATION_RAMPING
			// yes, this assumes always the x axis as the critical one regarding acceleration. If we want to implement per-axis acceleration, things get tricky ...
			dda->c_min = (move_duration / target->F) << 8;
			if (dda->c_min < c_limit)
				dda->c_min = c_limit;

      /**
        Assuming: F is in mm/min, STEPS_PER_M_X is in steps/m, ACCELERATION is in mm/s²
        Given:
         - Velocity v at time t given acceleration a: v(t) = a*t
         - Displacement s at time t given acceleration a: s(t) = 1/2 * a * t²
         - Displacement until reaching target velocity v: s = 1/2 * (v² / a)
         - Final result: steps needed to reach velocity v given acceleration a:
         steps = (STEPS_PER_M_X * F^2) / (7200000 * ACCELERATION)
         To keep precision, break up in floating point and integer part:
           F^2 * (int)(STEPS_PER_M_X / (7200000 * ACCELERATION))
         Note: the floating point part is static so its calculated during compilation.
         Note 2: the floating point part will be smaller than one, invert it:
                   steps = F^2 / (int)((7200000 * ACCELERATION) / STEPS_PER_M_X)
         Note 3: As mentioned, setting F to 65535 or larger will overflow the
                 calculation. Make sure this does not happen.
         Note 4: Anyone trying to run their machine at 65535 mm/min > 1m/s is nuts
       */
      if (target->F > 65534) target->F = 65534;
      // Note: this is inaccurate for several reasons:
      // - target->F isn't reverse-calculated from c_limit, so speed
      //   reductions due to slow axes are ignored.
      // - target->F means the speed of all axes combined, not the speed
      //   of the fast axis, which is taken into account here.
      // The good thing: taking target->F means rampup_steps is always
      // equal or larger than the number of steps required for acceleration,
      // so we can use it when also limiting max speed according to c_limit.
      dda->rampup_steps = ACCELERATE_RAMP_LEN(target->F);
      // Quick hack: we do not do Z move joins as jerk on the Z axis is undesirable;
      // as the ramp length is calculated for XY, its incorrect for Z: apply the original
      // 'fix' to simply specify a large enough ramp for any speed.
      if (x_delta_um == 0 && y_delta_um == 0) {
        dda->rampup_steps = 100000; // replace mis-calculation by a safe value
      }

      if (dda->rampup_steps > dda->total_steps / 2)
        dda->rampup_steps = dda->total_steps / 2;
      dda->rampdown_steps = dda->total_steps - dda->rampup_steps;

      #ifdef LOOKAHEAD
        dda_join_moves(prev_dda, dda);
      #endif
    #elif defined ACCELERATION_CLOCK
      uint16_t candidate;

      // Total time of the unaccelerated move.
      // 1 um/ms = 1 mm/s = 60 mm/min
      dda->time_total = distance * (60 / TICK_TIME_MS) / target->F;

      // To avoid overspeeding an axis, movement takes at least as
      // long as the slowest axis requires.
      candidate = x_delta_um * (60 / TICK_TIME_MS) / MAXIMUM_FEEDRATE_X;
      if (candidate > dda->time_total)
        dda->time_total = candidate;
      candidate = y_delta_um * (60 / TICK_TIME_MS) / MAXIMUM_FEEDRATE_Y;
      if (candidate > dda->time_total)
        dda->time_total = candidate;
      candidate = z_delta_um * (60 / TICK_TIME_MS) / MAXIMUM_FEEDRATE_Z;
      if (candidate > dda->time_total)
        dda->time_total = candidate;
      candidate = e_delta_um * (60 / TICK_TIME_MS) / MAXIMUM_FEEDRATE_E;
      if (candidate > dda->time_total)
        dda->time_total = candidate;
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("time total %u\n"), dda->time_total);

      // Re-calculate speeds, as they might have changed.
      dda->F_start = 0;
      dda->F_end = 0;
      dda->F_max = distance * (60 / TICK_TIME_MS) / dda->time_total;
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("corrected F_max %u\n"), dda->F_max);

      // Time in clock ticks required for acceleration.
      dda->time_accel = ((uint32_t)(dda->F_max - dda->F_start)) *
                        ((uint32_t)(1000 / TICK_TIME_MS)) /
                        ((uint32_t)(60 * ACCELERATION));
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("time accel %u\n"), dda->time_accel);

      // Time in clock ticks required for deceleration.
      dda->time_decel = ((uint32_t)(dda->F_max - dda->F_end)) *
                        ((uint32_t)(1000 / TICK_TIME_MS)) /
                        ((uint32_t)(60 * ACCELERATION));
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("time decel %u\n"), dda->time_decel);

      // Add time required for acceleration / deceleration.
      dda->time_total += dda->time_accel / 2 + dda->time_decel / 2;
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("time total w. accel %u\n"), dda->time_total);
      dda->time_decel = dda->time_total - dda->time_decel;
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("time decel2 %u\n"), dda->time_decel);

      // UGLY HACK: to compensate for inaccurate ac- and deceleration
      //            time calculations, add a margin here:
//      dda->time_decel += (dda->time_accel >> 3);
//sersendf_P(PSTR("time decel hacked %u\n"), dda->time_decel); delay_ms(10);

      // This is the ratio between F (in mm/min) and c (in CPU clock ticks)
      // and is constant during the entire move, even on curved movements.
      // Essentially, it's the step rate of the fastest stepping stepper
      // of the entire move.
      // For linear movements it's simple:
      dda->f_to_c = (distance * 2400L) / dda->total_steps * (F_CPU / 40000) << 8;
      if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
        sersendf_P(PSTR("f_to_c %lu\n"), dda->f_to_c);

      // Initial step delays. As we can't start with zero speed, advance
      // all calculations by half a clock tick.
      // v = a * t; c = 1 / v;
// Don't forget F_start!
      dda->c = dda->f_to_c / ((uint32_t)ACCELERATION * 60UL * TICK_TIME_MS / 1000UL);
      
sersendf_P(PSTR("c_min new %lu\n"), dda->f_to_c / target->F); delay_ms(10);
sersendf_P(PSTR("c_min trad %lu\n"), (move_duration / target->F) << 8); delay_ms(10);

		#elif defined ACCELERATION_TEMPORAL
			// TODO: limit speed of individual axes to MAXIMUM_FEEDRATE
			// TODO: calculate acceleration/deceleration for each axis
			dda->x_step_interval = dda->y_step_interval = \
				dda->z_step_interval = dda->e_step_interval = 0xFFFFFFFF;
			if (dda->x_delta)
				dda->x_step_interval = move_duration / dda->x_delta;
			if (dda->y_delta)
				dda->y_step_interval = move_duration / dda->y_delta;
			if (dda->z_delta)
				dda->z_step_interval = move_duration / dda->z_delta;
			if (dda->e_delta)
				dda->e_step_interval = move_duration / dda->e_delta;

			dda->axis_to_step = 'x';
			dda->c = dda->x_step_interval;
			if (dda->y_step_interval < dda->c) {
				dda->axis_to_step = 'y';
				dda->c = dda->y_step_interval;
			}
			if (dda->z_step_interval < dda->c) {
				dda->axis_to_step = 'z';
				dda->c = dda->z_step_interval;
			}
			if (dda->e_step_interval < dda->c) {
				dda->axis_to_step = 'e';
				dda->c = dda->e_step_interval;
			}

			dda->c <<= 8;
		#else
			dda->c = (move_duration / target->F) << 8;
			if (dda->c < c_limit)
				dda->c = c_limit;
		#endif
	} /* ! dda->total_steps == 0 */

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("] }\n"));

	// next dda starts where we finish
	memcpy(&startpoint, target, sizeof(TARGET));
}

/*! Start a prepared DDA
	\param *dda pointer to entry in dda_queue to start

	This function actually begins the move described by the passed DDA entry.

	We set direction and enable outputs, and set the timer for the first step from the precalculated value.

	We also mark this DDA as running, so other parts of the firmware know that something is happening

	Called both inside and outside of interrupts.
*/
void dda_start(DDA *dda) {
	// called from interrupt context: keep it simple!
	if ( ! dda->nullmove) {
		// get ready to go
		psu_timeout = 0;
		if (dda->z_delta)
			z_enable();
		if (dda->endstop_check)
			endstops_on();

		// set direction outputs
		x_direction(dda->x_direction);
		y_direction(dda->y_direction);
		z_direction(dda->z_direction);
		e_direction(dda->e_direction);

		#ifdef	DC_EXTRUDER
		if (dda->e_delta)
			heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
		#endif

		// initialise state variable
		move_state.x_counter = move_state.y_counter = move_state.z_counter = \
			move_state.e_counter = -(dda->total_steps >> 1);
		memcpy(&move_state.x_steps, &dda->x_delta, sizeof(uint32_t) * 4);
		#ifdef ACCELERATION_RAMPING
			move_state.step_no = 0;
		#endif
		#ifdef ACCELERATION_TEMPORAL
		move_state.x_time = move_state.y_time = \
			move_state.z_time = move_state.e_time = 0UL;
		#endif

		// ensure this dda starts
		dda->live = 1;

		// set timeout for first step
		#ifdef ACCELERATION_RAMPING
		if (dda->c_min > move_state.c) // can be true when look-ahead removed all deceleration steps
			setTimer(dda->c_min >> 8);
		else
			setTimer(move_state.c >> 8);
		#else
      #ifdef ACCELERATION_CLOCK
      move_state.time_current = 0;
      move_state.ticks_since_step = 0;
      #endif
		setTimer(dda->c >> 8);
		#endif
	}
	// else just a speed change, keep dda->live = 0

	current_position.F = dda->endpoint.F;
}

/*! STEP
	\param *dda the current move

	This is called from our timer interrupt every time a step needs to occur. Keep it as simple as possible!
	We first work out which axes need to step, and generate step pulses for them
	Then we re-enable global interrupts so serial data reception and other important things can occur while we do some math.
	Next, we work out how long until our next step using the selected acceleration algorithm and set the timer.
	Then we decide if this was the last step for this move, and if so mark this dda as dead so next timer interrupt we can start a new one.
	Finally we de-assert any asserted step pins.
*/
void dda_step(DDA *dda) {

#if ! defined ACCELERATION_TEMPORAL
	if (move_state.x_steps) {
		move_state.x_counter -= dda->x_delta;
		if (move_state.x_counter < 0) {
			x_step();
			move_state.x_steps--;
			move_state.x_counter += dda->total_steps;
		}
	}
#else	// ACCELERATION_TEMPORAL
	if (dda->axis_to_step == 'x') {
		x_step();
		move_state.x_steps--;
		move_state.x_time += dda->x_step_interval;
		move_state.all_time = move_state.x_time;
	}
#endif

#if ! defined ACCELERATION_TEMPORAL
	if (move_state.y_steps) {
		move_state.y_counter -= dda->y_delta;
		if (move_state.y_counter < 0) {
			y_step();
			move_state.y_steps--;
			move_state.y_counter += dda->total_steps;
		}
	}
#else	// ACCELERATION_TEMPORAL
	if ((dda->axis_to_step == 'y') && ! endstop_stop) {
		y_step();
		move_state.y_steps--;
		move_state.y_time += dda->y_step_interval;
		move_state.all_time = move_state.y_time;
	}
#endif

#if ! defined ACCELERATION_TEMPORAL
	if (move_state.z_steps) {
		move_state.z_counter -= dda->z_delta;
		if (move_state.z_counter < 0) {
			z_step();
			move_state.z_steps--;
			move_state.z_counter += dda->total_steps;
		}
	}
#else	// ACCELERATION_TEMPORAL
	if (dda->axis_to_step == 'z') {
		z_step();
		move_state.z_steps--;
		move_state.z_time += dda->z_step_interval;
		move_state.all_time = move_state.z_time;
	}
#endif

#if ! defined ACCELERATION_TEMPORAL
	if (move_state.e_steps) {
		move_state.e_counter -= dda->e_delta;
		if (move_state.e_counter < 0) {
			e_step();
			move_state.e_steps--;
			move_state.e_counter += dda->total_steps;
		}
	}
#else	// ACCELERATION_TEMPORAL
	if (dda->axis_to_step == 'e') {
		e_step();
		move_state.e_steps--;
		move_state.e_time += dda->e_step_interval;
		move_state.all_time = move_state.e_time;
	}
#endif

  #if defined STEP_INTERRUPT_INTERRUPTIBLE && ! defined ACCELERATION_CLOCK
		// Since we have sent steps to all the motors that will be stepping
		// and the rest of this function isn't so time critical, this interrupt
		// can now be interruptible by other interrupts.
		// The step interrupt is disabled before entering dda_step() to ensure
		// that we don't step again while computing the below.
		sei();
	#endif

	#ifdef ACCELERATION_REPRAP
		// linear acceleration magic, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
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
	#ifdef ACCELERATION_RAMPING
		// - algorithm courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		// - precalculate ramp lengths instead of counting them, see AVR446 tech note
		uint8_t recalc_speed;

		// debug ramping algorithm
		//if (move_state.step_no == 0) {
		//	sersendf_P(PSTR("\r\nc %lu  c_min %lu  n %d"), dda->c, dda->c_min, move_state.n);
		//}
		recalc_speed = 0;
		if (move_state.step_no < dda->rampup_steps) {
			if (move_state.n < 0) // wrong ramp direction
				move_state.n = -((int32_t)2) - move_state.n;
			recalc_speed = 1;
		}
		else if (move_state.step_no >= dda->rampdown_steps) {
			if (move_state.n > 0) // wrong ramp direction
				move_state.n = -((int32_t)2) - move_state.n;
			recalc_speed = 1;
		}
		if (recalc_speed) {
			move_state.n += 4;
			// be careful of signedness!
			move_state.c = (int32_t)move_state.c - ((int32_t)(move_state.c * 2) / (int32_t)move_state.n);
      //sersendf_P(PSTR("n:%ld; c:%ld; steps: %ld / %lu\n"), move_state.n,
      //           move_state.c, move_state.step_no, move_state.y_steps);
		}
		move_state.step_no++;

    #ifdef ACCELERATION_RAMPING
    // This is a hack which deals with movements with an unknown number of
    // acceleration steps. dda_create() sets a very high number, then.
    if (move_state.c < dda->c_min &&
        dda->rampup_steps > move_state.step_no + 5) {
      dda->rampup_steps = move_state.step_no;
      dda->rampdown_steps = dda->total_steps - dda->rampup_steps;
    }
    #endif

// Print the number of steps actually needed for ramping up
// Needed for comparing the number with the one calculated in dda_create()
//static char printed = 0;
//if (printed == 0 && dda->c_min >= move_state.c) {
//  sersendf_P(PSTR("speedup %lu steps\n"), move_state.step_no);
//  printed = 1;
//}
//if (move_state.step_no < 3) printed = 0;

		// debug ramping algorithm
		// raise this 10 for higher speeds to avoid flooding the serial line
		//if (move_state.step_no % 10 /* 10, 50, 100, ...*/ == 0)
		//	sersendf_P(PSTR("\r\nc %lu  c_min %lu  n %ld"),
		//	           move_state.c, dda->c_min, move_state.n);
	#endif

	#ifdef ACCELERATION_TEMPORAL
		/** How is this ACCELERATION TEMPORAL expected to work?

			All axes work independently of each other, as if they were on four different, synchronized timers. As we have not enough suitable timers, we have to share one for all axes.

			To do this, each axis maintains the time of its last step in move_state.{xyze}_time. This time is updated as the step is done, see early in dda_step(). To find out which axis is the next one to step, the time of each axis' next step is compared to the time of the step just done. Zero means this actually is the axis just stepped, the smallest value > 0 wins.

			One problem undoubtly arising is, steps should sometimes be done at {almost,exactly} the same time. We trust the timer to deal properly with very short or even zero periods. If a step can't be done in time, the timer shall do the step as soon as possible and compensate for the delay later. In turn we promise here to send a maximum of four such short-delays consecutively and to give sufficient time on average.
		*/
		uint32_t c_candidate;

		dda->c = 0xFFFFFFFF;
		if (move_state.x_steps) {
			c_candidate = move_state.x_time + dda->x_step_interval - move_state.all_time;
			dda->axis_to_step = 'x';
			dda->c = c_candidate;
		}
		if (move_state.y_steps) {
			c_candidate = move_state.y_time + dda->y_step_interval - move_state.all_time;
			if (c_candidate < dda->c) {
				dda->axis_to_step = 'y';
				dda->c = c_candidate;
			}
		}
		if (move_state.z_steps) {
			c_candidate = move_state.z_time + dda->z_step_interval - move_state.all_time;
			if (c_candidate < dda->c) {
				dda->axis_to_step = 'z';
				dda->c = c_candidate;
			}
		}
		if (move_state.e_steps) {
			c_candidate = move_state.e_time + dda->e_step_interval - move_state.all_time;
			if (c_candidate < dda->c) {
				dda->axis_to_step = 'e';
				dda->c = c_candidate;
			}
		}
		dda->c <<= 8;
	#endif

  // If there are no steps left or an endstop stop happened, we have finished.
  if ((move_state.x_steps == 0 && move_state.y_steps == 0 &&
       move_state.z_steps == 0 && move_state.e_steps == 0)
    #ifdef ACCELERATION_RAMPING
      || (dda->endstop_check && move_state.n == -3)
    #endif
      ) {
		dda->live = 0;
    #ifdef ACCELERATION_CLOCK
    if (dda->time_total - move_state.time_current > 1)
      sersendf_P(PSTR("undershoot by %u ticks\n"),
                 dda->time_total - move_state.time_current);
    #endif
    #ifdef LOOKAHEAD
    // If look-ahead was using this move, it could have missed our activation:
    // make sure the ids do not match.
    dda->id--;
    #endif
		#ifdef	DC_EXTRUDER
			heater_set(DC_EXTRUDER, 0);
		#endif
		// z stepper is only enabled while moving
		z_disable();
	}
	else
		psu_timeout = 0;

	#ifdef ACCELERATION_RAMPING
		// we don't hit maximum speed exactly with acceleration calculation, so limit it here
		// the nice thing about _not_ setting dda->c to dda->c_min is, the move stops at the exact same c as it started, so we have to calculate c only once for the time being
		// TODO: set timer only if dda->c has changed
		if (dda->c_min > move_state.c)
			setTimer(dda->c_min >> 8);
		else
			setTimer(move_state.c >> 8);
	#else
    #ifdef ACCELERATION_CLOCK
    move_state.ticks_since_step = 0;
    #endif
		setTimer(dda->c >> 8);
	#endif

	// turn off step outputs, hopefully they've been on long enough by now to register with the drivers
	// if not, too bad. or insert a (very!) small delay here, or fire up a spare timer or something.
	// we also hope that we don't step before the drivers register the low- limit maximum speed if you think this is a problem.
	unstep();
}

/*! Do regular movement maintenance.

  This should be called pretty often, like once every 1 ot 2 milliseconds.

  Currently, this is checking the endstops. These don't need to be checked on
  every single step, so this code can be moved out of the highly time critical
  dda_step(). At high precision (slow) searches of the endstop, this function
  is called more often than dda_step() anyways.

  In the future, acceleration and arc movement calculations might go here, too.
  Updating speed 500 times a second is easily enough for smooth acceleration!
*/
void dda_clock() {
  static volatile uint8_t busy = 0;
  DDA *dda;
  static DDA *last_dda = NULL;
  static uint8_t endstop_stop = 0; ///< Stop due to endstop trigger

  move_state.time_current++;

  dda = queue_current_movement();
  if (dda != last_dda) {
    move_state.debounce_count_xmin = move_state.debounce_count_ymin =
    move_state.debounce_count_zmin = move_state.debounce_count_xmax =
    move_state.debounce_count_ymax = move_state.debounce_count_zmax = 0;
    endstop_stop = 0;
    last_dda = dda;
  }

  if (dda == NULL)
    return;

  // Lengthy calculations ahead!
  // Make sure we didn't re-enter, then allow nested interrupts.
  if (busy) {
    if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
      serial_writechar('B');
    return;
  }
  busy = 1;
  sei();

  // Caution: we mangle step counters here without locking interrupts. This
  //          means, we trust dda isn't changed behind our back, which could
  //          in principle (but rarely) happen if endstops are checked not as
  //          endstop search, but as part of normal operations.
  if (endstop_stop == 0) {
    #if defined X_MIN_PIN || defined X_MAX_PIN
    if (dda->endstop_check & 0x1) {
      #if defined X_MIN_PIN
      if (x_min() == dda->endstop_stop_cond)
        move_state.debounce_count_xmin++;
      else
        move_state.debounce_count_xmin = 0;
      #endif
      #if defined X_MAX_PIN
      if (x_max() == dda->endstop_stop_cond)
        move_state.debounce_count_xmax++;
      else
        move_state.debounce_count_xmax = 0;
      #endif
      endstop_stop = move_state.debounce_count_xmin >= ENDSTOP_STEPS ||
                     move_state.debounce_count_xmax >= ENDSTOP_STEPS;
    }
    #endif

    #if defined Y_MIN_PIN || defined Y_MAX_PIN
    if (dda->endstop_check & 0x2) {
      #if defined Y_MIN_PIN
      if (y_min() == dda->endstop_stop_cond)
        move_state.debounce_count_ymin++;
      else
        move_state.debounce_count_ymin = 0;
      #endif
      #if defined Y_MAX_PIN
      if (y_max() == dda->endstop_stop_cond)
        move_state.debounce_count_ymax++;
      else
        move_state.debounce_count_ymax = 0;
      #endif
      endstop_stop = move_state.debounce_count_ymin >= ENDSTOP_STEPS ||
                     move_state.debounce_count_ymax >= ENDSTOP_STEPS;
    }
    #endif

    #if defined Z_MIN_PIN || defined Z_MAX_PIN
    if (dda->endstop_check & 0x4) {
      #if defined Z_MIN_PIN
      if (z_min() == dda->endstop_stop_cond)
        move_state.debounce_count_zmin++;
      else
        move_state.debounce_count_zmin = 0;
      #endif
      #if defined Z_MAX_PIN
      if (z_max() == dda->endstop_stop_cond)
        move_state.debounce_count_zmax++;
      else
        move_state.debounce_count_zmax = 0;
      #endif
      endstop_stop = move_state.debounce_count_zmin >= ENDSTOP_STEPS ||
                     move_state.debounce_count_zmax >= ENDSTOP_STEPS;
    }
    #endif

    // If an endstop is definitely triggered, stop the movement.
    if (endstop_stop) {
      #ifdef ACCELERATION_RAMPING
      // For always smooth operations, don't halt apruptly,
      // but start deceleration here.
      ATOMIC_START
        dda->rampdown_steps = move_state.step_no;
        dda->rampup_steps = 0; // in case we're still accelerating
      ATOMIC_END
      #else
      dda->live = 0;
      #endif

      endstops_off();
    }
  } /* if (endstop_stop == 0) */

  #ifdef ACCELERATION_CLOCK
  uint32_t new_c = 0;
  static uint8_t plateau_done = 0;

  // Overtime?
  if (move_state.time_current > dda->time_total) {
    // Keep it short to have at least a chance to get it sent.
    #warning Das hier hört nicht auf.
    //serial_writestr_P(PSTR("ot"));
    move_state.time_current = dda->time_total;
  }
  // Acceleration time.
  else if (move_state.time_current < dda->time_accel) {
    // v = a * t; c = 1 / v;
    new_c = dda->f_to_c / ((uint32_t)ACCELERATION * 60UL * (uint32_t)move_state.time_current * TICK_TIME_MS / 1000UL);
    plateau_done = 0;
serial_writechar('a');
  }
  else if (move_state.time_current > dda->time_decel) {
    uint32_t dt = (uint32_t)dda->time_total - (uint32_t)move_state.time_current;

    if (dt < 1) // we undershot *sigh*
      dt = 1;
    // v = a * t; c = 1 / v;
    new_c = dda->f_to_c / ((uint32_t)ACCELERATION * 60UL * dt * TICK_TIME_MS / 1000UL);
serial_writechar('d');
    plateau_done = 0;

  }
  // Plateau time.
  else if (plateau_done == 0) {
    new_c = dda->f_to_c / dda->F_max;
    plateau_done = 1;
serial_writechar('r');
  }
  else
serial_writechar('.');

  if (new_c) {
    ATOMIC_START
      dda->c = new_c;    
    ATOMIC_END
  }

  // Set up or readjust the timer if actual steps happen too slowly. dda_step()
  // resets ticks_since_step to zero, while we increment it here, so we have an
  // idea on how much time is gone since the last actual step.
  // 300 = minimum time setTimer requires.
if (dda->time_total == move_state.time_current)
sersendf_P(PSTR("overshoot by %lu steps\n"), move_state.x_steps);
  if (move_state.ticks_since_step) {
    if ((dda->c >> 8) < ((uint32_t)move_state.ticks_since_step * TICK_TIME) + 300UL) {
      // We're too late already, go as quick as possbile.
      setTimer(300UL);
serial_writechar('-');
    }
    else {
      // TODO: we ignore the time taken until we get here.
      setTimer((dda->c >> 8) - ((uint32_t)move_state.ticks_since_step * TICK_TIME));
serial_writechar('+');
    }
  }
  sei(); // setTimer locks interrupts
  move_state.ticks_since_step++;
  #endif

  cli(); // Compensate sei() above.
  busy = 0;
}

/// update global current_position struct
void update_current_position() {
	DDA *dda = &movebuffer[mb_tail];

	if (queue_empty()) {
		current_position.X = startpoint.X;
		current_position.Y = startpoint.Y;
		current_position.Z = startpoint.Z;
		current_position.E = startpoint.E;
	}
	else if (dda->live) {
		if (dda->x_direction)
			// (STEPS_PER_M_X / 1000) is a bit inaccurate for low STEPS_PER_M numbers
			current_position.X = dda->endpoint.X -
			                     // should be: move_state.x_steps * 1000000 / STEPS_PER_M_X)
			                     // but x_steps can be like 1000000 already, so we'd overflow
			                     move_state.x_steps * 1000 / ((STEPS_PER_M_X + 500) / 1000);
		else
			current_position.X = dda->endpoint.X +
			                     move_state.x_steps * 1000 / ((STEPS_PER_M_X + 500) / 1000);

		if (dda->y_direction)
			current_position.Y = dda->endpoint.Y -
			                     move_state.y_steps * 1000 / ((STEPS_PER_M_Y + 500) / 1000);
		else
			current_position.Y = dda->endpoint.Y +
			                     move_state.y_steps * 1000 / ((STEPS_PER_M_Y + 500) / 1000);

		if (dda->z_direction)
			current_position.Z = dda->endpoint.Z -
			                     move_state.z_steps * 1000 / ((STEPS_PER_M_Z + 500) / 1000);
		else
			current_position.Z = dda->endpoint.Z +
			                     move_state.z_steps * 1000 / ((STEPS_PER_M_Z + 500) / 1000);

		if (dda->endpoint.e_relative) {
			current_position.E = move_state.e_steps * 1000 / ((STEPS_PER_M_E + 500) / 1000);
		}
		else {
			if (dda->e_direction)
				current_position.E = dda->endpoint.E -
				                     move_state.e_steps * 1000 / ((STEPS_PER_M_E + 500) / 1000);
			else
				current_position.E = dda->endpoint.E +
				                     move_state.e_steps * 1000 / ((STEPS_PER_M_E + 500) / 1000);
		}

		// current_position.F is updated in dda_start()
	}
}
