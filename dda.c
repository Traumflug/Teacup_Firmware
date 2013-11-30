#include	"dda.h"

/** \file
	\brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include	<string.h>
#include	<stdlib.h>
#include	<math.h>
#ifndef SIMULATOR
#include	<avr/interrupt.h>
#endif

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

/* 32-bit-specific abs fn coerces argument to 32-bit signed value first */
#define abs32(x) labs((int32_t)(x))

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
 *
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
void dda_create(DDA *dda, TARGET *target) {
	uint32_t	steps, x_delta_um, y_delta_um, z_delta_um, e_delta_um;
	uint32_t	distance, c_limit, c_limit_calc;
  #ifdef LOOKAHEAD
  // Number the moves to identify them; allowed to overflow.
  static uint8_t idcnt = 0;
  static DDA* prev_dda = NULL;

  if ((prev_dda && prev_dda->done) || dda->waitfor_temp)
    prev_dda = NULL;
  #endif

  if (dda->waitfor_temp)
    return;

  // Initialise DDA to a known state. This also clears flags like
  // dda->live, dda->done and dda->wait_for_temp.
	dda->allflags = 0;

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("\nCreate: X %lq  Y %lq  Z %lq  F %lu\n"),
               dda->endpoint.X, dda->endpoint.Y,
               dda->endpoint.Z, dda->endpoint.F);


	// we end at the passed target
	memcpy(&(dda->endpoint), target, sizeof(TARGET));

  #ifdef LOOKAHEAD
    // Set the start and stop speeds to zero for now = full stops between
    // moves. Also fallback if lookahead calculations fail to finish in time.
    dda->crossF = 0;
    dda->F_start = 0;
    dda->start_steps = 0;
    dda->F_end = 0;
    dda->end_steps = 0;
    // Give this move an identifier.
    dda->id = idcnt++;
  #endif

// TODO TODO: We should really make up a loop for all axes.
//            Think of what happens when a sixth axis (multi colour extruder)
//            appears?
	x_delta_um = (uint32_t)abs32(target->X - startpoint.X);
	y_delta_um = (uint32_t)abs32(target->Y - startpoint.Y);
	z_delta_um = (uint32_t)abs32(target->Z - startpoint.Z);

	steps = um_to_steps_x(target->X);
	dda->x_delta = abs32(steps - startpoint_steps.X);
	startpoint_steps.X = steps;
	steps = um_to_steps_y(target->Y);
	dda->y_delta = abs32(steps - startpoint_steps.Y);
	startpoint_steps.Y = steps;
	steps = um_to_steps_z(target->Z);
	dda->z_delta = abs32(steps - startpoint_steps.Z);
	startpoint_steps.Z = steps;

	dda->x_direction = (target->X >= startpoint.X)?1:0;
	dda->y_direction = (target->Y >= startpoint.Y)?1:0;
	dda->z_direction = (target->Z >= startpoint.Z)?1:0;

	if (target->e_relative) {
		e_delta_um = abs32(target->E);
		dda->e_delta = abs32(um_to_steps_e(target->E));
		dda->e_direction = (target->E >= 0)?1:0;
	}
	else {
		e_delta_um = (uint32_t)abs32(target->E - startpoint.E);
		steps = um_to_steps_e(target->E);
		dda->e_delta = abs32(steps - startpoint_steps.E);
		startpoint_steps.E = steps;
		dda->e_direction = (target->E >= startpoint.E)?1:0;
	}

  #ifdef LOOKAHEAD
  // Also displacements in micrometers, but for the lookahead alogrithms.
  dda->delta_um.X = target->X - startpoint.X;
  dda->delta_um.Y = target->Y - startpoint.Y;
  dda->delta_um.Z = target->Z - startpoint.Z;
  dda->delta_um.E = target->e_relative ? target->E : target->E - startpoint.E;
  #endif

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("[%ld,%ld,%ld,%ld]"),
               target->X - startpoint.X, target->Y - startpoint.Y,
               target->Z - startpoint.Z, target->E - startpoint.E);

	dda->total_steps = dda->x_delta;
	if (dda->y_delta > dda->total_steps)
		dda->total_steps = dda->y_delta;
	if (dda->z_delta > dda->total_steps)
		dda->total_steps = dda->z_delta;
	if (dda->e_delta > dda->total_steps)
		dda->total_steps = dda->e_delta;

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR(" [ts:%lu"), dda->total_steps);

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
    // TODO: instead of calculating c_min directly, it's probably more simple
    //       to calculate (maximum) move_duration for each axis, like done for
    //       ACCELERATION_TEMPORAL above. This should make re-calculating the
    //       allowed F easier.
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
      if (dda->c_min < c_limit) {
				dda->c_min = c_limit;
        dda->endpoint.F = move_duration / (dda->c_min >> 8);
      }

      // Lookahead can deal with 16 bits ( = 1092 mm/s), only.
      if (dda->endpoint.F > 65535)
        dda->endpoint.F = 65535;

      // Note: this is inaccurate:
      // - ACCELERATE_RAMP_LEN() uses STEPS_PER_M_X, so axes not matching
      //   this get too much or not enough rampup steps.
      uint32_t fast_um;
      
      if (dda->total_steps == dda->x_delta)
        fast_um = x_delta_um;
      else if (dda->total_steps == dda->y_delta)
        fast_um = y_delta_um;
      else if (dda->total_steps == dda->z_delta)
        fast_um = z_delta_um;
      else if (dda->total_steps == dda->e_delta)
        fast_um = e_delta_um;
      else {
        fast_um = 0;
        sersendf_P(PSTR("WTF? No prev fast axis found\n"));
      }
      dda->rampup_steps = ACCELERATE_RAMP_LEN(muldiv(fast_um, dda->endpoint.F, distance));

      if (dda->rampup_steps > dda->total_steps / 2)
        dda->rampup_steps = dda->total_steps / 2;
      dda->rampdown_steps = dda->total_steps - dda->rampup_steps;

      #ifdef LOOKAHEAD
        dda->distance = distance;
        dda_find_crossing_speed(prev_dda, dda);
        // TODO: this should become a reverse-stepping through the existing
        //       movement queue to allow higher speeds for short moves.
        //       dda_find_crossing_speed() is required only once.
        dda_join_moves(prev_dda, dda);
        dda->n = dda->start_steps;
        if (dda->n == 0)
          dda->c = C0;
        else
          dda->c = ((C0 >> 8) * int_inv_sqrt(dda->n)) >> 5;
        if (dda->c < dda->c_min)
          dda->c = dda->c_min;
      #else
        dda->n = 0;
        dda->c = C0;
      #endif

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
  #ifdef LOOKAHEAD
    prev_dda = dda;
  #endif
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

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("Start: X %lq  Y %lq  Z %lq  F %lu\n"),
               dda->endpoint.X, dda->endpoint.Y,
               dda->endpoint.Z, dda->endpoint.F);

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
    move_state.endstop_stop = 0;
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
    setTimer(dda->c >> 8);
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
	if (dda->axis_to_step == 'y') {
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

	#if STEP_INTERRUPT_INTERRUPTIBLE && ! defined ACCELERATION_RAMPING
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
		move_state.step_no++;
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
      || (move_state.endstop_stop && dda->n == 0)
    #endif
      ) {
		dda->live = 0;
    dda->done = 1;
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
  else {
		psu_timeout = 0;
    // After having finished, dda_start() will set the timer.
    setTimer(dda->c >> 8);
  }

	// turn off step outputs, hopefully they've been on long enough by now to register with the drivers
	// if not, too bad. or insert a (very!) small delay here, or fire up a spare timer or something.
	// we also hope that we don't step before the drivers register the low- limit maximum speed if you think this is a problem.
	unstep();
}

/*! Do regular movement maintenance.

  This should be called pretty often, like once every 1 ot 2 milliseconds.

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
  static volatile uint8_t busy = 0;
  DDA *dda;
  static DDA *last_dda = NULL;
  uint8_t endstop_trigger = 0;
  uint32_t move_step_no, move_c;
  uint8_t recalc_speed;

  dda = queue_current_movement();
  if (dda != last_dda) {
    move_state.debounce_count_xmin = move_state.debounce_count_ymin =
    move_state.debounce_count_zmin = move_state.debounce_count_xmax =
    move_state.debounce_count_ymax = move_state.debounce_count_zmax = 0;
    last_dda = dda;
  }

  if (dda == NULL)
    return;

  // Lengthy calculations ahead!
  // Make sure we didn't re-enter, then allow nested interrupts.
  if (busy)
    return;
  busy = 1;
  sei();

  // Caution: we mangle step counters here without locking interrupts. This
  //          means, we trust dda isn't changed behind our back, which could
  //          in principle (but rarely) happen if endstops are checked not as
  //          endstop search, but as part of normal operations.
  if (dda->endstop_check && ! move_state.endstop_stop) {
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
      endstop_trigger = move_state.debounce_count_xmin >= ENDSTOP_STEPS ||
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
      endstop_trigger = move_state.debounce_count_ymin >= ENDSTOP_STEPS ||
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
      endstop_trigger = move_state.debounce_count_zmin >= ENDSTOP_STEPS ||
                        move_state.debounce_count_zmax >= ENDSTOP_STEPS;
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
    // http://www.embedded.com/columns/technicalinsights/56800129?printable=true
    // and http://www.atmel.com/images/doc8017.pdf (Atmel app note AVR446)
    ATOMIC_START
      move_step_no = move_state.step_no;
      // All other variables are read-only or unused in dda_step(),
      // so no need for atomic operations.
    ATOMIC_END

    recalc_speed = 0;
    if (move_step_no < dda->rampup_steps) {
      #ifdef LOOKAHEAD
        dda->n = dda->start_steps + move_step_no;
      #else
        dda->n = move_step_no;
      #endif
      recalc_speed = 1;
    }
    else if (move_step_no >= dda->rampdown_steps) {
      #ifdef LOOKAHEAD
        dda->n = dda->total_steps - move_step_no + dda->end_steps;
      #else
        dda->n = dda->total_steps - move_step_no;
      #endif
      recalc_speed = 1;
    }
    if (recalc_speed) {
      if (dda->n == 0)
        move_c = C0;
      else
        // Explicit formula: c0 * (sqrt(n + 1) - sqrt(n)),
        // approximation here: c0 * (1 / (2 * sqrt(n))).
        move_c = ((C0 >> 8) * int_inv_sqrt(dda->n)) >> 5;

      // TODO: most likely this whole check is obsolete. It was left as a
      //       safety margin, only. Rampup steps calculation should be accurate
      //       now and give the requested target speed within a few percent.
      if (move_c < dda->c_min) {
        // We hit max speed not always exactly.
        move_c = dda->c_min;

        // This is a hack which deals with movements with an unknown number of
        // acceleration steps. dda_create() sets a very high number, then,
        // but we don't want to re-calculate all the time.
        // This hack doesn't work with lookahead.
        #ifndef LOOKAHEAD
          dda->rampup_steps = move_step_no;
          dda->rampdown_steps = dda->total_steps - dda->rampup_steps;
        #endif
      }

      // Write results.
      ATOMIC_START
        dda->c = move_c;
      ATOMIC_END
    }
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
