#include	"dda.h"

#include	<string.h>
#include	<avr/interrupt.h>

#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"dda_queue.h"

#ifndef	ABS
#define	ABS(v)		(((v) >= 0)?(v):(-(v)))
#endif

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

#ifndef	DEBUG
#define	DEBUG 0
#endif

/*
	step timeout
*/

uint8_t	steptimeout = 0;

/*
	position tracking
*/

TARGET startpoint = { 0, 0, 0, 0, 0 };
TARGET current_position = { 0, 0, 0, 0, 0 };

/*
	utility functions
*/

// courtesy of http://www.oroboro.com/rafael/docserv.php/index/programming/article/distance
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

uint32_t abs32(int32_t v) {
	if (v < 0)
		return (uint32_t) (-v);
	return (uint32_t) (v);
}

uint32_t delta32(uint32_t v1, uint32_t v2) {
	if (v1 >= v2)
		return v1 - v2;
	return v2 - v1;
}

/*
	CREATE a dda given current_position and a target, save to passed location so we can write directly into the queue
*/

void dda_create(DDA *dda, TARGET *target) {
	uint32_t	distance;

	// initialise DDA to a known state
	dda->move_duration = 0;
	dda->live = 0;
	dda->total_steps = 0;

	if (DEBUG)
		serial_writestr_P(PSTR("\n{DDA_CREATE: ["));

	// we end at the passed target
	memcpy(&(dda->endpoint), target, sizeof(TARGET));

	dda->x_delta = abs32(target->X - startpoint.X);
	dda->y_delta = abs32(target->Y - startpoint.Y);
	dda->z_delta = abs32(target->Z - startpoint.Z);
	dda->e_delta = abs32(target->E - startpoint.E);
	dda->f_delta = delta32(target->F, startpoint.F);

	dda->x_direction = (target->X >= startpoint.X)?1:0;
	dda->y_direction = (target->Y >= startpoint.Y)?1:0;
	dda->z_direction = (target->Z >= startpoint.Z)?1:0;
	dda->e_direction = (target->E >= startpoint.E)?1:0;
	dda->f_direction = (target->F >= startpoint.F)?1:0;

	if (DEBUG) {
		if (dda->x_direction == 0)
			serial_writechar('-');
		serwrite_uint32(dda->x_delta); serial_writechar(',');
		if (dda->y_direction == 0)
			serial_writechar('-');
		serwrite_uint32(dda->y_delta); serial_writechar(',');
		if (dda->z_direction == 0)
			serial_writechar('-');
		serwrite_uint32(dda->z_delta); serial_writechar(',');
		if (dda->e_direction == 0)
			serial_writechar('-');
		serwrite_uint32(dda->e_delta); serial_writechar(',');
		if (dda->f_direction == 0)
			serial_writechar('-');
		serwrite_uint32(dda->f_delta); serial_writestr_P(PSTR("] ["));
	}

	if (dda->x_delta > dda->total_steps)
		dda->total_steps = dda->x_delta;
	if (dda->y_delta > dda->total_steps)
		dda->total_steps = dda->y_delta;
	if (dda->z_delta > dda->total_steps)
		dda->total_steps = dda->z_delta;
	if (dda->e_delta > dda->total_steps)
		dda->total_steps = dda->e_delta;

	if (dda->total_steps == 0) {
		dda->nullmove = 1;
	}
	else {
		// get steppers ready to go
		steptimeout = 0;
		enable_steppers();

		if (DEBUG) {
			serwrite_uint32(dda->total_steps); serial_writechar(',');
		}

		dda->x_counter = dda->y_counter = dda->z_counter = dda->e_counter = dda->f_counter =
			-(dda->total_steps >> 1);

		// since it's unusual to combine X, Y and Z changes in a single move on reprap, check if we can use simpler approximations before trying the full 3d approximation.
		if (dda->z_delta == 0)
			distance = approx_distance(dda->x_delta * UM_PER_STEP_X, dda->y_delta * UM_PER_STEP_Y);
		else if (dda->x_delta == 0 && dda->y_delta == 0)
			distance = dda->z_delta * UM_PER_STEP_Z;
		else
			distance = approx_distance_3(dda->x_delta * UM_PER_STEP_X, dda->y_delta * UM_PER_STEP_Y, dda->z_delta * UM_PER_STEP_Z);

		if (distance < 2)
			distance = dda->e_delta * UM_PER_STEP_E;

		if (DEBUG) {
			serwrite_uint32(distance); serial_writechar(',');
		}

		// pre-calculate move speed in millimeter microseconds per step minute for less math in interrupt context
		// mm (distance) * 60000000 us/min / step (total_steps) = mm.us per step.min
		//   note: um (distance) * 60000 == mm * 60000000
		// so in the interrupt we must simply calculate
		// mm.us per step.min / mm per min (F) = us per step

		// break this calculation up a bit and lose some precision because 300,000um * 60000 is too big for a uint32
		// calculate this with a uint64 if you need the precision, but it'll take longer so routines with lots of short moves may suffer
		// 2^32/6000 is about 715mm which should be plenty
		dda->move_duration = ((distance * 6000) / dda->total_steps) * 10;

		if (DEBUG)
			serwrite_uint32(dda->move_duration);
	}

	if (DEBUG)
		serial_writestr_P(PSTR("] }\n"));

	// next dda starts where we finish
	memcpy(&startpoint, target, sizeof(TARGET));
}

/*
	Start a prepared DDA
*/

void dda_start(DDA *dda) {
	// called from interrupt context: keep it simple!
	if (dda->nullmove) {
		// just change speed?
		current_position.F = dda->endpoint.F;
		return;
	}

	// ensure steppers are ready to go
	steptimeout = 0;
	enable_steppers();

	// set direction outputs
	x_direction(dda->x_direction);
	y_direction(dda->y_direction);
	z_direction(dda->z_direction);
	e_direction(dda->e_direction);

	// ensure this dda starts
	dda->live = 1;

	// set timeout for first step
	setTimer(dda->move_duration / current_position.F);
}

/*
	CAN STEP
*/

uint8_t	can_step(uint8_t min, uint8_t max, int32_t current, int32_t target, uint8_t dir) {
	if (dir) {
		// forwards/positive
		if (max)
			return 0;
		if (current >= target)
			return 0;
	}
	else {
		// backwards/negative
		if (min)
			return 0;
		if (target >= current)
			return 0;
	}

	return 255;
}

/*
	STEP
*/

void dda_step(DDA *dda) {
	uint8_t	step_option = 0;
#define	X_CAN_STEP	1
#define	Y_CAN_STEP	2
#define	Z_CAN_STEP	4
#define	E_CAN_STEP	8
#define	F_CAN_STEP	16
#define	REAL_MOVE		32
#define	F_REAL_STEP	64

	if (DEBUG)
		serial_writechar('!');

	step_option &= ~(X_CAN_STEP | Y_CAN_STEP | Z_CAN_STEP | E_CAN_STEP | F_CAN_STEP);
// 		step_option |= can_step(x_min(), x_max(), current_position.X, dda->endpoint.X, dda->x_direction) & X_CAN_STEP;
// 		step_option |= can_step(y_min(), y_max(), current_position.Y, dda->endpoint.Y, dda->y_direction) & Y_CAN_STEP;
// 		step_option |= can_step(z_min(), z_max(), current_position.Z, dda->endpoint.Z, dda->z_direction) & Z_CAN_STEP;
	step_option |= can_step(0      , 0      , current_position.X, dda->endpoint.X, dda->x_direction) & X_CAN_STEP;
	step_option |= can_step(0      , 0      , current_position.Y, dda->endpoint.Y, dda->y_direction) & Y_CAN_STEP;
	step_option |= can_step(0      , 0      , current_position.Z, dda->endpoint.Z, dda->z_direction) & Z_CAN_STEP;
	step_option |= can_step(0      , 0      , current_position.E, dda->endpoint.E, dda->e_direction) & E_CAN_STEP;
	step_option |= can_step(0      , 0      , current_position.F, dda->endpoint.F, dda->f_direction) & F_CAN_STEP;

	if (step_option & X_CAN_STEP) {
		dda->x_counter -= dda->x_delta;
		if (dda->x_counter < 0) {
			step_option |= REAL_MOVE;

			x_step();
			if (dda->x_direction)
				current_position.X++;
			else
				current_position.X--;

			dda->x_counter += dda->total_steps;
		}
	}

	if (step_option & Y_CAN_STEP) {
		dda->y_counter -= dda->y_delta;
		if (dda->y_counter < 0) {
			step_option |= REAL_MOVE;

			y_step();
			if (dda->y_direction)
				current_position.Y++;
			else
				current_position.Y--;

			dda->y_counter += dda->total_steps;
		}
	}

	if (step_option & Z_CAN_STEP) {
		dda->z_counter -= dda->z_delta;
		if (dda->z_counter < 0) {
			step_option |= REAL_MOVE;

			z_step();
			if (dda->z_direction)
				current_position.Z++;
			else
				current_position.Z--;

			dda->z_counter += dda->total_steps;
		}
	}

	if (step_option & E_CAN_STEP) {
		dda->e_counter -= dda->e_delta;
		if (dda->e_counter < 0) {
			step_option |= REAL_MOVE;

			e_step();
			if (dda->e_direction)
				current_position.E++;
			else
				current_position.E--;

			dda->e_counter += dda->total_steps;
		}
	}

	if (step_option & F_CAN_STEP) {
		dda->f_counter -= dda->f_delta;
		// since we don't allow total_steps to be defined by F, we may need to step multiple times if f_delta is greater than total_steps
		while (dda->f_counter < 0) {

			dda->f_counter += dda->total_steps;

			if (dda->f_direction) {
				current_position.F += 1;
				if (current_position.F > dda->endpoint.F)
					current_position.F = dda->endpoint.F;
			}
			else {
				current_position.F -= 1;
				if (current_position.F < dda->endpoint.F)
					current_position.F = dda->endpoint.F;
			}

			step_option |= F_REAL_STEP;
		}
	}

	// this interrupt can now be interruptible
	disableTimerInterrupt();
	sei();

	// this generates too much debug output for all but the slowest step rates
	if (0 && DEBUG) {
		serial_writechar('[');
		serwrite_hex8(step_option);
		serial_writechar(':');
		serwrite_int32(current_position.F);
		serial_writechar('/');
		serwrite_int32(dda->endpoint.F);
		serial_writechar('#');
		serwrite_uint32(dda->move_duration);
		serial_writechar(']');
	}

	if (step_option & REAL_MOVE)
		// we stepped, reset timeout
		steptimeout = 0;

	// we have stepped in speed and now need to recalculate our delay
	// WARNING: this is a divide in interrupt context! (which unfortunately seems unavoidable)
	// we simply don't have the memory to precalculate this for each step,
	// can't use a simplified process because the denominator changes rather than the numerator so the curve is non-linear
	// and don't have a process framework to force it to be done outside interrupt context within a usable period of time
	if (step_option & F_REAL_STEP)
		setTimer(dda->move_duration / current_position.F);

	// if we could do anything at all, we're still running
	// otherwise, must have finished
	else if (step_option == 0) {
		dda->live = 0;
		#ifdef	STUPIDLY_HIDE_BUGS
			// this magically makes bugs disappear after each move, probably a bad idea
			memcpy(&current_position, &(dda->endpoint), sizeof(TARGET));
		#endif
	}

	// turn off step outputs, hopefully they've been on long enough by now to register with the drivers
	// if not, too bad. or insert a (very!) small delay here, or fire up a spare timer or something
	unstep();

	// reset interruptible so we can return in the same state we started
	cli();
	enableTimerInterrupt();
}
