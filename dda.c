#include	"dda.h"

/** \file
	\brief Digital differential analyser - this is where we figure out which steppers need to move, and when they need to move
*/

#include	<string.h>
#include	<stdlib.h>
#include	<avr/interrupt.h>

#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"dda_queue.h"
#include	"debug.h"
#include	"sersendf.h"
#include	"pinio.h"
#include	"config.h"
//#include "graycode.c"

#ifdef	DC_EXTRUDER
	#include	"heater.h"
#endif

/// step timeout
volatile uint8_t	steptimeout = 0;

/*
	position tracking
*/

/// \var startpoint
/// \brief target position of last move in queue
TARGET startpoint __attribute__ ((__section__ (".bss")));

/// \var current_position
/// \brief actual position of extruder head
/// \todo make current_position = real_position (from endstops) + offset from G28 and friends
TARGET current_position __attribute__ ((__section__ (".bss")));

/*
	utility functions
*/

// courtesy of http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
/*! linear approximation 2d distance formula
	\param dx distance in X plane
	\param dy distance in Y plane
	\return 3-part linear approximation of \f$\sqrt{\Delta x^2 + \Delta y^2}\f$

	see http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
*/
uint32_t approx_distance( uint32_t dx, uint32_t dy )
{
	uint32_t min, max, approx;

	if ( dx < dy )
	{
		min = dx;
		max = dy;
	} else {
		min = dy;
		max = dx;
	}

	approx = ( max * 1007 ) + ( min * 441 );
	if ( max < ( min << 4 ))
		approx -= ( max * 40 );

	// add 512 for proper rounding
	return (( approx + 512 ) >> 10 );
}

// courtesy of http://www.oroboro.com/rafael/docserv.php/index/programming/article/distance
/*! linear approximation 3d distance formula
	\param dx distance in X plane
	\param dy distance in Y plane
	\param dz distance in Z plane
	\return 3-part linear approximation of \f$\sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}\f$

	see http://www.oroboro.com/rafael/docserv.php/index/programming/article/distance
*/
uint32_t approx_distance_3( uint32_t dx, uint32_t dy, uint32_t dz )
{
	uint32_t min, med, max, approx;

	if ( dx < dy )
	{
		min = dy;
		med = dx;
	} else {
		min = dx;
		med = dy;
	}

	if ( dz < min )
	{
		max = med;
		med = min;
		min = dz;
	} else if ( dz < med ) {
		max = med;
		med = dz;
	} else {
		max = dz;
	}

	approx = ( max * 860 ) + ( med * 851 ) + ( min * 520 );
	if ( max < ( med << 1 )) approx -= ( max * 294 );
	if ( max < ( min << 2 )) approx -= ( max * 113 );
	if ( med < ( min << 2 )) approx -= ( med *  40 );

	// add 512 for proper rounding
	return (( approx + 512 ) >> 10 );
}

/*!
	integer square root algorithm
	\param a find square root of this number
	\return sqrt(a - 1) < returnvalue <= sqrt(a)

	see http://www.embedded-systems.com/98/9802fe2.htm
*/
// courtesy of http://www.embedded-systems.com/98/9802fe2.htm
uint16_t int_sqrt(uint32_t a) {
	uint32_t rem = 0;
	uint32_t root = 0;
	uint16_t i;

	for (i = 0; i < 16; i++) {
		root <<= 1;
		rem = ((rem << 2) + (a >> 30));
		a <<= 2;
		root++;
		if (root <= rem) {
			rem -= root;
			root++;
		}
		else
			root--;
	}
	return (uint16_t) ((root >> 1) & 0xFFFFL);
}

// this is an ultra-crude pseudo-logarithm routine, such that:
// 2 ^ msbloc(v) >= v
/*! crude logarithm algorithm
	\param v value to find \f$log_2\f$ of
	\return floor(log(v) / log(2))
*/
const uint8_t	msbloc (uint32_t v) {
	uint8_t i;
	uint32_t c;
	for (i = 31, c = 0x80000000; i; i--) {
		if (v & c)
			return i;
		c >>= 1;
	}
	return 0;
}

/*! CREATE a dda given current_position and a target, save to passed location so we can write directly into the queue
	\param *dda pointer to a dda_queue entry to overwrite
	\param *target the target position of this move

	\ref startpoint the beginning position of this move

	This function does a /lot/ of math. It works out directions for each axis, distance travelled, the time between the first and second step

	It also pre-fills any data that the selected accleration algorithm needs, and can be pre-computed for the whole move.

	This algorithm is probably the main limiting factor to print speed in terms of firmware limitations
*/
void dda_create(DDA *dda, TARGET *target) {
	uint32_t	distance, c_limit, c_limit_calc;

	// initialise DDA to a known state
	dda->allflags = 0;

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("\n{DDA_CREATE: ["));

	// we end at the passed target
	memcpy(&(dda->endpoint), target, sizeof(TARGET));

	dda->x_steps = dda->x_delta = labs(target->X - startpoint.X);
	dda->y_steps = dda->y_delta = labs(target->Y - startpoint.Y);
	dda->z_steps = dda->z_delta = labs(target->Z - startpoint.Z);
	dda->e_steps = dda->e_delta = labs(target->E - startpoint.E);

	dda->x_direction = (target->X >= startpoint.X)?1:0;
	dda->y_direction = (target->Y >= startpoint.Y)?1:0;
	dda->z_direction = (target->Z >= startpoint.Z)?1:0;
	dda->e_direction = (target->E >= startpoint.E)?1:0;

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
		steptimeout = 0;
		power_on();
		x_enable();
		y_enable();
		// Z is enabled in dda_start()
		e_enable();

		// since it's unusual to combine X, Y and Z changes in a single move on reprap, check if we can use simpler approximations before trying the full 3d approximation.
		if (dda->z_delta == 0)
			distance = approx_distance(dda->x_delta * UM_PER_STEP_X, dda->y_delta * UM_PER_STEP_Y);
		else if (dda->x_delta == 0 && dda->y_delta == 0)
			distance = dda->z_delta * UM_PER_STEP_Z;
		else
			distance = approx_distance_3(dda->x_delta * UM_PER_STEP_X, dda->y_delta * UM_PER_STEP_Y, dda->z_delta * UM_PER_STEP_Z);

		if (distance < 2)
			distance = dda->e_delta * UM_PER_STEP_E;

		if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
			sersendf_P(PSTR(",ds:%lu"), distance);

		#ifdef	ACCELERATION_TEMPORAL
			// bracket part of this equation in an attempt to avoid overflow: 60 * 16MHz * 5mm is >32 bits
			uint32_t move_duration = distance * (60 * F_CPU / startpoint.F);
		#else
			dda->x_counter = dda->y_counter = dda->z_counter = dda->e_counter =
				-(dda->total_steps >> 1);

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
		c_limit_calc = ( (dda->x_delta * (UM_PER_STEP_X * 2400L)) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_X) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check Y axis
		c_limit_calc = ( (dda->y_delta * (UM_PER_STEP_Y * 2400L)) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_Y) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check Z axis
		c_limit_calc = ( (dda->z_delta * (UM_PER_STEP_Z * 2400L)) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_Z) << 8;
		if (c_limit_calc > c_limit)
			c_limit = c_limit_calc;
		// check E axis
		c_limit_calc = ( (dda->e_delta * (UM_PER_STEP_E * 2400L)) / dda->total_steps * (F_CPU / 40000) / MAXIMUM_FEEDRATE_E) << 8;
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
		// add the last bit of dda->total_steps to always round up
			dda->ramp_steps = dda->total_steps / 2 + (dda->total_steps & 1);
			dda->step_no = 0;
			// c is initial step time in IOclk ticks
			dda->c = ACCELERATION_STEEPNESS << 8;
			dda->c_min = (move_duration / target->F) << 8;
			if (dda->c_min < c_limit)
				dda->c_min = c_limit;
			dda->n = 1;
			dda->ramp_state = RAMP_UP;
		#else
			dda->c = (move_duration / target->F) << 8;
			if (dda->c < c_limit)
				dda->c = c_limit;
		#endif
	}

	if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
		serial_writestr_P(PSTR("] }\n"));

	// next dda starts where we finish
	memcpy(&startpoint, target, sizeof(TARGET));
	// if E is relative, reset it here
	#ifndef E_ABSOLUTE
		startpoint.E = 0;
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
	if (dda->nullmove) {
		// just change speed?
		current_position.F = dda->endpoint.F;
		// keep dda->live = 0
	}
	else {
/*		if (dda->waitfor_temp) {
			#ifndef	REPRAP_HOST_COMPATIBILITY
				serial_writestr_P(PSTR("Waiting for target temp\n"));
			#endif
		}
		else {*/
		// get ready to go
		steptimeout = 0;
		if (dda->z_delta)
			z_enable();

		// set direction outputs
		x_direction(dda->x_direction);
		y_direction(dda->y_direction);
		z_direction(dda->z_direction);
		e_direction(dda->e_direction);

		#ifdef	DC_EXTRUDER
		if (dda->e_delta)
			heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
		#endif

// 		}

		// ensure this dda starts
		dda->live = 1;

		// set timeout for first step
		setTimer(dda->c >> 8);
	}
}

/*! STEP
	\param *dda the current move

	This is called from our timer interrupt every time a step needs to occur.
	We first work out which axes need to step, and generate step pulses for them
	Then we re-enable global interrupts so serial data reception and other important things can occur while we do some math.
	Next, we work out how long until our next step using the selected acceleration algorithm and set the timer.
	Then we decide if this was the last step for this move, and if so mark this dda as dead so next timer interrupt we can start a new one.
	Finally we de-assert any asserted step pins.

	\todo take into account the time that interrupt takes to run
*/
void dda_step(DDA *dda) {
	// called from interrupt context! keep it as simple as possible
	uint8_t	did_step = 0;

	if ((dda->x_steps) /* &&
			(x_max() != dda->x_direction) && (x_min() == dda->x_direction) */) {
		dda->x_counter -= dda->x_delta;
		if (dda->x_counter < 0) {
			x_step();
			did_step = 1;
			dda->x_steps--;
			dda->x_counter += dda->total_steps;
		}
	}

	if ((dda->y_steps) /* &&
			(y_max() != dda->y_direction) && (y_min() == dda->y_direction) */) {
		dda->y_counter -= dda->y_delta;
		if (dda->y_counter < 0) {
			y_step();
			did_step = 1;
			dda->y_steps--;
			dda->y_counter += dda->total_steps;
		}
	}

	if ((dda->z_steps) /* &&
			(z_max() != dda->z_direction) && (z_min() == dda->z_direction) */) {
		dda->z_counter -= dda->z_delta;
		if (dda->z_counter < 0) {
			z_step();
			did_step = 1;
			dda->z_steps--;
			dda->z_counter += dda->total_steps;
		}
	}

	if (dda->e_steps) {
		dda->e_counter -= dda->e_delta;
		if (dda->e_counter < 0) {
			e_step();
			did_step = 1;
			dda->e_steps--;
			dda->e_counter += dda->total_steps;
		}
	}

	#if STEP_INTERRUPT_INTERRUPTIBLE
		// since we have sent steps to all the motors that will be stepping and the rest of this function isn't so time critical,
		// this interrupt can now be interruptible
		// however we must ensure that we don't step again while computing the below, so disable *this* interrupt but allow others to fire
// 		disableTimerInterrupt();
		sei();
	#endif

	#ifdef ACCELERATION_REPRAP
		// linear acceleration magic, courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		if (dda->accel) {
			if (
					((dda->n > 0) && (dda->c > dda->end_c)) ||
					((dda->n < 0) && (dda->c < dda->end_c))
				) {
				dda->c = (int32_t) dda->c - ((int32_t) (dda->c * 2) / dda->n);
				dda->n += 4;
			}
			else if (dda->c != dda->end_c) {
				dda->c = dda->end_c;
			}
			// else we are already at target speed
		}
	#endif
	#ifdef ACCELERATION_RAMPING
		// - algorithm courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
		// - for simplicity, taking even/uneven number of steps into account dropped
		// - number of steps moved is always accurate, speed might be one step off
		switch (dda->ramp_state) {
			case RAMP_UP:
			case RAMP_MAX:
				if (dda->step_no >= dda->ramp_steps) {
					// RAMP_UP: time to decelerate before reaching maximum speed
					// RAMP_MAX: time to decelerate
					dda->ramp_state = RAMP_DOWN;
					dda->n = -((int32_t)2) - dda->n;
				}
				if (dda->ramp_state == RAMP_MAX)
					break;
			case RAMP_DOWN:
				dda->n += 4;
				// be careful of signedness!
				dda->c = (int32_t)dda->c - ((int32_t)(dda->c * 2) / dda->n);
				if (dda->c <= dda->c_min) {
					// maximum speed reached
					dda->c = dda->c_min;
					dda->ramp_state = RAMP_MAX;
					dda->ramp_steps = dda->total_steps - dda->step_no;
				}
				break;
		}
		dda->step_no++;
	#endif

	if (did_step) {
		// we stepped, reset timeout
		steptimeout = 0;

		// if we could do anything at all, we're still running
		// otherwise, must have finished
	}
	else if (dda->x_steps == 0 && dda->y_steps == 0 && dda->z_steps == 0 && dda->e_steps == 0) {
		dda->live = 0;
		// if E is relative reset it
		#ifndef E_ABSOLUTE
			current_position.E = 0;
		#endif
		// linear acceleration code doesn't alter F during a move, so we must update it here
		// in theory, we *could* update F every step, but that would require a divide in interrupt context which should be avoided if at all possible
		current_position.F = dda->endpoint.F;
		#ifdef	DC_EXTRUDER
			heater_set(DC_EXTRUDER, 0);
		#endif
		// z stepper is only enabled while moving
		z_disable();
	}

	cli();

	setTimer(dda->c >> 8);

	// turn off step outputs, hopefully they've been on long enough by now to register with the drivers
	// if not, too bad. or insert a (very!) small delay here, or fire up a spare timer or something.
	// we also hope that we don't step before the drivers register the low- limit maximum speed if you think this is a problem.
	unstep();
}

/// update global current_position struct
void update_position() {
	DDA *dda = &movebuffer[mb_tail];

	if (dda->live == 0)
		return;

	if (dda->x_direction)
		current_position.X = dda->endpoint.X - dda->x_steps;
	else
		current_position.X = dda->endpoint.X + dda->x_steps;

	if (dda->y_direction)
		current_position.Y = dda->endpoint.Y - dda->y_steps;
	else
		current_position.Y = dda->endpoint.Y + dda->y_steps;

	if (dda->z_direction)
		current_position.Z = dda->endpoint.Z - dda->z_steps;
	else
		current_position.Z = dda->endpoint.Z + dda->z_steps;

	#ifndef E_ABSOLUTE
		current_position.E = dda->e_steps;
	#else
		if (dda->e_direction)
			current_position.E = dda->endpoint.E - dda->e_steps;
		else
			current_position.E = dda->endpoint.E + dda->e_steps;
	#endif
}
