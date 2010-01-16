#include	"dda.h"

#include	<string.h>

#include	"timer.h"

extern struct {
	volatile int32_t	X;
	volatile int32_t	Y;
	volatile int32_t	Z;
	volatile int32_t	E;
	volatile int32_t	F;
} current_position;

uint8_t	mb_head = 0;
uint8_t	mb_tail = 0;
DDA movebuffer[MOVEBUFFER_SIZE];

uint8_t queue_full() {
	if (mb_tail == 0)
		return mb_head == (MOVEBUFFER_SIZE - 1);
	else
		return mb_head == (mb_tail - 1);
}

inline uint8_t queue_empty() {
	return (mb_tail == mb_head) && !movebuffer[tail].live;
}

void enqueue(TARGET *t) {
	while (queue_full())
		delay(WAITING_DELAY);

	uint8_t h = mb_head;
	h++;
	if (h == MOVEBUFFER_SIZE)
		h = 0;
	mb_head = h;
	dda_create(t, &movebuffer[h]);
}

void next_move() {
	if ((mb_tail == mb_head) && (!movebuffer[mb_tail].live)) {
		// queue is empty
		disable_steppers();
		setTimer(DEFAULT_TICK);
	}
	else {
		uint8_t t = mb_tail;
		t++;
		if (t == MOVEBUFFER_SIZE)
			t = 0;
		mb_tail = t;
		dda_start(&movebuffer[t]);
	}
}

// courtesy of http://www.oroboro.com/rafael/docserv.php/index/programming/article/distance
uint32_t approx_distance( int32_t dx, int32_t dy )
{
	uint32_t min, max, approx;

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

	approx = ( max * 1007 ) + ( min * 441 );
	if ( max < ( min << 4 ))
		approx -= ( max * 40 );

	// add 512 for proper rounding
	return (( approx + 512 ) >> 10 );
}

// courtesy of http://www.oroboro.com/rafael/docserv.php/index/programming/article/distance
uint32_t approx_distance_3( int32_t dx, int32_t dy, int32_t dz )
{
	uint32_t min, med, max, approx;

	if ( dx < 0 ) dx = -dx;
	if ( dy < 0 ) dy = -dy;
	if ( dz < 0 ) dz = -dz;

	if ( dx < dy )
	{
		min = dy;
		med = dx;
	} else {
		min = dx;
		med = dy;
	}

	if ( dz < (int32_t)min )
	{
		max = med;
		med = min;
		min = dz;
	} else if ( dz < (int32_t)med ) {
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

/*
	CREATE
*/

void dda_create(TARGET *target, DDA *dda) {
	static TARGET startpoint = { 0, 0, 0, 0, 0 };

	// we start at the previous endpoint
	memcpy(&dda->currentpoint, &startpoint, sizeof(TARGET));
	// we end at the passed command's endpoint
	memcpy(&dda->endpoint, target, sizeof(TARGET));

	dda->x_delta = dda->endpoint.X - startpoint.X;
	dda->y_delta = dda->endpoint.Y - startpoint.Y;
	dda->z_delta = dda->endpoint.Z - startpoint.Z;
	// always relative
	dda->e_delta = dda->endpoint.E;
	// always absolute
	dda->f_delta = dda->endpoint.F - startpoint.F;

	// since it's unusual to combine X, Y and Z changes in a single move on reprap, check if we can use simpler approximations before trying the full 3d approximation.
	if (dda->z_delta == 0)
		dda->distance = approx_distance(dda->x_delta, dda->y_delta);
	else if (dda->x_delta == 0 && dda->y_delta == 0)
		dda->distance = dda->z_delta;
	else
		dda->distance = approx_distance_3(dda->x_delta, dda->y_delta, dda->z_delta);

	if (dda->distance < 2)
		dda->distance = dda->e_delta;
	if (dda->distance < 2)
		dda->distance = dda->f_delta;

	dda->total_steps = dda->x_delta;
	if (dda->y_delta > dda->total_steps)
		dda->total_steps = dda->y_delta;
	if (dda->z_delta > dda->total_steps)
		dda->total_steps = dda->z_delta;

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
	dda->z_direction = (dda->endpoint.Z > startpoint.Z)?1:0;
	dda->e_direction = (dda->endpoint.E > startpoint.E)?1:0;
	dda->f_direction = (dda->endpoint.F > startpoint.F)?1:0;

	dda->x_counter = dda->y_counter = dda->z_counter = dda->e_counter = dda->f_counter
		= -(dda->total_steps >> 1);

	// pre-calculate move speed in millimeter microseconds per step minute for less math in interrupt context
	// mm (distance) * 60000000 us/min / step (total_steps) = mm.us per step.min
	// mm.us per step.min / mm/min (F) = us per step
	dda->move_duration = dda->distance * 60000000 / dda->total_steps;

	// next dda starts where we finish
	memcpy(&startpoint, &dda->endpoint, sizeof(TARGET));

	dda->live = 0;
}

/*
	START
*/

void dda_start(DDA *dda) {
	// called from interrupt context: keep it simple!
	if (dda->nullmove)
		return;

	x_direction(dda->x_direction);
	y_direction(dda->y_direction);
	z_direction(dda->z_direction);
	e_direction(dda->e_direction);

	enable_steppers();
	dda->live = 1;
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
		step_option |= can_step(-1     , -1     , current_position.E, dda->endpoint.E, dda->e_direction) & E_CAN_STEP;
		step_option |= can_step(-1     , -1     , current_position.F, dda->endpoint.F, dda->f_direction) & F_CAN_STEP;

		if (step_option & X_CAN_STEP) {
			dda->x_counter -= dda->x_delta;
			if (dda->x_counter < 0) {
				step_option |= REAL_MOVE;

				x_step();

				dda->x_counter += dda->total_steps;

				if (dda->x_direction)
					current_position.X++;
				else
					current_position.X--;
			}
		}

		if (step_option & Y_CAN_STEP) {
			dda->y_counter -= dda->y_delta;
			if (dda->y_counter < 0) {
				step_option |= REAL_MOVE;

				y_step();

				dda->y_counter += dda->total_steps;

				if (dda->y_direction)
					current_position.Y++;
				else
					current_position.Y--;
			}
		}

		if (step_option & Z_CAN_STEP) {
			dda->z_counter -= dda->z_delta;
			if (dda->z_counter < 0) {
				step_option |= REAL_MOVE;

				z_step();

				dda->z_counter += dda->total_steps;

				if (dda->z_direction)
					current_position.Z++;
				else
					current_position.Z--;
			}
		}

		if (step_option & E_CAN_STEP) {
			dda->e_counter -= dda->e_delta;
			if (dda->e_counter < 0) {
				step_option |= REAL_MOVE;

				e_step();

				dda->e_counter += dda->total_steps;

				if (dda->e_direction)
					current_position.E++;
				else
					current_position.E--;
			}
		}

		if (step_option & F_CAN_STEP) {
			dda->f_counter -= dda->f_delta;
			if (dda->f_counter < 0) {

				dda->f_counter += dda->total_steps;

				if (dda->f_direction)
					current_position.F += dda->f_scale;
				else
					current_position.F -= dda->f_scale;
			}
		}
	} while (	((step_option & REAL_MOVE ) == 0) &&
						((step_option & F_CAN_STEP) != 0)	);

	unstep();

	if (step_option & REAL_MOVE) {
		setTimer(dda->move_duration / current_position.F);
	}

	dda->live = (step_option & (X_CAN_STEP | Y_CAN_STEP | Z_CAN_STEP | E_CAN_STEP | F_CAN_STEP));
}
