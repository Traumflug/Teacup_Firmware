#include	"dda.h"

#include	<string.h>

extern struct {
	volatile int32_t	X;
	volatile int32_t	Y;
	volatile int32_t	Z;
	volatile int32_t	E;
	volatile int32_t	F;
} current_position;

// courtesy of http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
uint32_t approx_distance( int32_t dx, int32_t dy )
{
	uint32_t min, max;

	if ( dx < 0 ) dx = -dx;
	if ( dy < 0 ) dy = -dy;

	if ( dx < dy )
	{
		min = dx;
		max = dy;
	} else {
		min = dy;
		max = dx;
	}

	// coefficients equivalent to ( 123/128 * max ) and ( 51/128 * min )
	return ((( max << 8 ) + ( max << 3 ) - ( max << 4 ) - ( max << 1 ) +
					 ( min << 7 ) - ( min << 5 ) + ( min << 3 ) - ( min << 1 )) >> 8 );
}

/*
	CREATE
*/

void dda_create(GCODE_COMMAND *cmd, DDA *dda) {
	static TARGET startpoint = { 0, 0, 0, 0, 0 };

	// we start at the previous endpoint
	memcpy(&dda->currentpoint, &startpoint, sizeof(TARGET));
	// we end at the passed command's endpoint
	memcpy(&dda->endpoint, &cmd->target, sizeof(TARGET));

	dda->x_delta = dda->endpoint.X - startpoint.X;
	dda->y_delta = dda->endpoint.Y - startpoint.Y;
	// always relative
	dda->e_delta = dda->endpoint.E;
	// always absolute
	dda->f_delta = dda->endpoint.F - startpoint.F;


	dda->distance = approx_distance(dda->x_delta, dda->y_delta);

	if (dda->distance < 2)
		dda->distance = dda->e_delta;
	if (dda->distance < 2)
		dda->distance = dda->f_delta;

	dda->total_steps = dda->x_delta;
	if (dda->y_delta > dda->total_steps)
		dda->total_steps = dda->y_delta;
	if (dda->e_delta > dda->total_steps)
		dda->total_steps = dda->e_delta;
	if (dda->f_delta > dda->total_steps)
		dda->total_steps = dda->f_delta;

	if (dda->total_steps == 0)
		dda->nullmove = 1;

	// MM = sqrt(X^2 + Y^2)
	// STEPS = max(X * STEPS_PER_MM_X, Y * STEPS_PER_MM_Y)
	// DURATION = MM / MM_PER_MIN * 60 SEC_PER_MIN * 1000000 US_PER_SEC
	// US/STEP = DURATION / STEPS
	// intF = sqrt(X^2 + Y^2) / max(X * STEPS_PER_MM_X, Y * STEPS_PER_MM_Y)

// 	dda->endpoint.F = distance / total_steps;

	if (dda->f_delta > dda->total_steps) {
		// TODO: rescale F
		dda->f_scale = dda->f_delta / dda->total_steps;
		if (dda->f_scale > 3) {
			dda->f_delta /= dda->f_scale;
		}
		else {
			dda->f_scale = 1;
			dda->total_steps = dda->f_delta;
		}
	}

	dda->x_direction = (dda->endpoint.X > startpoint.X)?1:0;
	dda->y_direction = (dda->endpoint.Y > startpoint.Y)?1:0;
	dda->e_direction = (dda->endpoint.E > startpoint.E)?1:0;
	dda->f_direction = (dda->endpoint.F > startpoint.F)?1:0;

	dda->x_counter = dda->y_counter = dda->e_counter = dda->f_counter
		= -(dda->total_steps >> 1);

	// next dda starts where we finish
	memcpy(&startpoint, &dda->endpoint, sizeof(TARGET));
}

/*
	START
*/

void dda_start(DDA *dda) {
	x_direction(dda->x_direction);
	y_direction(dda->y_direction);
	z_direction(dda->z_direction);
	e_direction(dda->e_direction);
}

/*
	CAN STEP
*/

uint8_t	can_step(uint8_t min, uint8_t max, int32_t current, int32_t target, uint8_t dir) {
	if (target == current)
		return 0;

	if (min && !dir)
		return 0;

	if (max && dir)
		return 0;

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

	do {
		step_option |= can_step(x_min(), x_max(), current_position.X, dda->endpoint.X, dda->x_direction) & X_CAN_STEP;
		step_option |= can_step(y_min(), y_max(), current_position.Y, dda->endpoint.Y, dda->y_direction) & Y_CAN_STEP;
		step_option |= can_step(z_min(), z_max(), current_position.Z, dda->endpoint.Z, dda->z_direction) & Z_CAN_STEP;
		step_option |= can_step(-1, -1, current_position.E, dda->endpoint.E, dda->e_direction) & E_CAN_STEP;
		step_option |= can_step(-1, -1, current_position.F, dda->endpoint.F, dda->f_direction) & F_CAN_STEP;

		if (step_option & X_CAN_STEP) {
			dda->x_counter += dda->x_delta;
			if (dda->x_counter > 0) {
				step_option |= REAL_MOVE;

				// do X step
				dda->x_counter -= dda->total_steps;

				if (dda->x_direction)
					current_position.X++;
				else
					current_position.X--;
			}
		}

		if (step_option & Y_CAN_STEP) {
			dda->y_counter += dda->y_delta;
			if (dda->y_counter > 0) {
				step_option |= REAL_MOVE;

				// do Y step
				dda->y_counter -= dda->total_steps;

				if (dda->y_direction)
					current_position.Y++;
				else
					current_position.Y--;
			}
		}

		if (step_option & Z_CAN_STEP) {
			dda->z_counter += dda->z_delta;
			if (dda->z_counter > 0) {
				step_option |= REAL_MOVE;

				// do Z step
				dda->z_counter -= dda->total_steps;

				if (dda->z_direction)
					current_position.Z++;
				else
					current_position.Z--;
			}
		}

		if (step_option & E_CAN_STEP) {
			dda->e_counter += dda->e_delta;
			if (dda->e_counter > 0) {
				step_option |= REAL_MOVE;

				// do E step
				dda->e_counter -= dda->total_steps;

				if (dda->e_direction)
					current_position.E++;
				else
					current_position.E--;
			}
		}

		if (step_option & F_CAN_STEP) {
			dda->f_counter += dda->f_delta;
			if (dda->f_counter > 0) {
				// do F step
				dda->f_counter -= dda->total_steps;

				if (dda->f_direction)
					current_position.F += dda->f_scale;
				else
					current_position.F -= dda->f_scale;
			}
		}
	} while (((step_option & REAL_MOVE) == 0) && (step_option & F_CAN_STEP));

	if (step_option & REAL_MOVE) {
		setTimer(dda->distance * (60000000 / current_position.F / dda->total_steps));
	}
}
