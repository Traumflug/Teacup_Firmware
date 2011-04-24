#include	<stdio.h>
#include	<stdint.h>
#include	<stdlib.h>
#include	<math.h>

#define	F_CPU 16000000L

#define	X_STEPS_PER_MM	320.0
#define	Y_STEPS_PER_MM	320.0
#define	Z_STEPS_PER_MM	200.0
#define	E_STEPS_PER_MM	287.0

#define	X_UM_PER_STEP		(1000.0 / X_STEPS_PER_MM)
#define	Y_UM_PER_STEP		(1000.0 / Y_STEPS_PER_MM)
#define	Z_UM_PER_STEP		(1000.0 / Z_STEPS_PER_MM)
#define	E_UM_PER_STEP		(1000.0 / E_STEPS_PER_MM)

#define	X_ACCEL_MM_S_S	9.0
#define	Y_ACCEL_MM_S_S	5.0
#define	Z_ACCEL_MM_S_S	1.0
#define	E_ACCEL_MM_S_S	15.0

#define	X_DECEL_MM_S_S	3.0
#define	Y_DECEL_MM_S_S	8.0
#define	Z_DECEL_MM_S_S	1.0
#define	E_DECEL_MM_S_S	18.0


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



void move(int32_t dx, int32_t dy, int32_t dz, int32_t de, uint32_t f) {
	uint32_t distance = 0;
	uint32_t x_delta, y_delta, z_delta, e_delta;
	uint32_t x_speed, y_speed, z_speed, e_speed;
	uint32_t x_accel_distance, y_accel_distance, z_accel_distance, e_accel_distance;
	uint32_t x_c, y_c, z_c, e_c;
	uint32_t x_n, y_n, z_n, e_n;
	uint32_t x_cr, y_cr, z_cr, e_cr;
	uint32_t x_minc, y_minc, z_minc, e_minc;
	uint32_t x_accel = X_ACCEL_MM_S_S * 1000.0, y_accel = Y_ACCEL_MM_S_S * 1000.0, z_accel = Z_ACCEL_MM_S_S * 1000.0, e_accel = E_ACCEL_MM_S_S * 1000.0;
	uint32_t duration;

	uint32_t accel_distance, decel_distance;

	uint32_t elapsed_ticks, total_ticks;

	// distance is micrometers
	if ((dx != 0 || dy != 0) && dz == 0)
		distance = approx_distance(dx * X_UM_PER_STEP, dy * Y_UM_PER_STEP);
	if (dx == 0 && dy == 0 && dz != 0)
		distance = dz * Z_UM_PER_STEP;
	if (distance < 2 && de != 0)
		distance = de * E_UM_PER_STEP;
	if (distance == 0)
		return;

	printf("distance: %dum\n", distance);

	// duration is microseconds
	duration = distance * 3L * (F_CPU / 50L / f);

	printf("duration: %d ticks (%ldms)\n", duration, duration / (F_CPU / 1000L));

	// deltas are in steps
	x_delta = labs(dx);
	y_delta = labs(dy);
	z_delta = labs(dz);
	e_delta = labs(de);

	// speeds are in um per second
	if (x_delta)
		x_speed = x_delta * X_UM_PER_STEP * F_CPU / duration;
	if (y_delta)
		y_speed = y_delta * Y_UM_PER_STEP * F_CPU / duration;
	if (z_delta)
		z_speed = z_delta * Z_UM_PER_STEP * F_CPU / duration;
	if (e_delta)
		e_speed = e_delta * E_UM_PER_STEP * F_CPU / duration;

	printf("X speed: %dum/s, Y speed: %dum/s\n", x_speed, y_speed);

	accel_distance = 0;

// n = w^2 / 2aw'
// 	my $x_steps_to_accel = $x_speed * $x_speed * $x_steps_per_mm / 2 / $x_accel_mm_s_s;
// 	x_accel_steps = (x_speed * x_speed / 1000000) * X_STEPS_PER_MM / 2 / X_ACCEL_MM_S_S;
// 	x_accel_steps = (x_delta * 1000 / X_STEPS_PER_MM * F_CPU / duration * x_delta * 1000 / X_STEPS_PER_MM * F_CPU / duration / 1000000) * X_STEPS_PER_MM / 2 / X_ACCEL_MM_S_S;
// 	x_accel_steps = (x_delta / X_STEPS_PER_MM * F_CPU / (distance * F_CPU * 3 / 50 / f) * x_delta / X_STEPS_PER_MM * F_CPU / (distance * F_CPU * 3 / 50 / f)) * X_STEPS_PER_MM / 2 / X_ACCEL_MM_S_S;
// 	x_accel_steps = (x_delta * x_delta / X_STEPS_PER_MM * F_CPU / distance / F_CPU / 3 * 50 * f / X_STEPS_PER_MM * F_CPU / distance / F_CPU / 3 * 50 * f) * X_STEPS_PER_MM / 2 / X_ACCEL_MM_S_S;
// 	x_accel_steps = (x_delta * x_delta / X_STEPS_PER_MM / distance / 3 * 50 * f / X_STEPS_PER_MM / distance / 3 * 50 * f) * X_STEPS_PER_MM / 2 / X_ACCEL_MM_S_S;
// 	x_accel_steps = (x_delta * x_delta * 1250 * f * f / X_STEPS_PER_MM / distance / distance / 3 / 3) / X_ACCEL_MM_S_S;
// x_accel_steps = (x_delta * f / distance / 3) * (x_delta * f / distance / 3)  * 1250 / X_STEPS_PER_MM / X_ACCEL_MM_S_S;
// x_accel_distance = x_accel_steps * X_UM_PER_STEP;
	#warning This calculation is susceptible to overflow!
	if (x_delta) {
		x_accel_distance = (x_delta * f / distance / 3L) * (x_delta * f / distance / 3L)  * 1250L / X_STEPS_PER_MM / X_ACCEL_MM_S_S * 1000L / X_STEPS_PER_MM;
		if (x_accel_distance > accel_distance)
			accel_distance = x_accel_distance;
	}
	if (y_delta) {
		y_accel_distance = (y_delta * f / distance / 3L) * (y_delta * f / distance / 3L)  * 1250L / Y_STEPS_PER_MM / Y_ACCEL_MM_S_S * 1000L / Y_STEPS_PER_MM;
		if (y_accel_distance > accel_distance)
			accel_distance = y_accel_distance;
	}
	if (z_delta) {
		z_accel_distance = (z_delta * f / distance / 3L) * (z_delta * f / distance / 3L)  * 1250L / Z_STEPS_PER_MM / Z_ACCEL_MM_S_S * 1000L / Z_STEPS_PER_MM;
		if (z_accel_distance > accel_distance)
			accel_distance = z_accel_distance;
	}
	if (e_delta) {
		e_accel_distance = (e_delta * f / distance / 3L) * (e_delta * f / distance / 3L)  * 1250L / E_STEPS_PER_MM / E_ACCEL_MM_S_S * 1000L / E_STEPS_PER_MM;
		if (e_accel_distance > accel_distance)
			accel_distance = e_accel_distance;
	}

	printf("Accel Distance: %dum\n", accel_distance);

	// 		n = w^2 / 2aw'
	//    w' = w^2 / 2an
	//    w' = w^2 * steps_per_mm / 2n
	//    x_accel = x_speed * x_speed * X_STEPS_PER_MM / 2 / (accel_distance * X_STEPS_PER_MM)
	//    x_accel = x_speed * x_speed / 2 / accel_distance / 1000
	// let's store in um/s2 instead of mm/s2 for precision
	#warning This calculation is susceptible to overflow!
	if (x_accel_distance < accel_distance)
		x_accel = x_speed * x_speed / accel_distance / 2L;
	if (y_accel_distance < accel_distance)
		y_accel = y_speed * y_speed / accel_distance / 2L;
	if (z_accel_distance < accel_distance)
		z_accel = z_speed * z_speed / accel_distance / 2L;
	if (e_accel_distance < accel_distance)
		e_accel = e_speed * e_speed / accel_distance / 2L;

	printf("X accel: %dum/s2, Y accel: %dum/s2\n", x_accel, y_accel);

	// 	c0 = f . sqrt(2a / accel)
	//     = F_CPU * sqrt(2 / accel * steps_per_mm)
	//     = F_CPU * sqrt(2) / sqrt(accel / 1000) / sqrt(steps_per_mm)
	//     = F_CPU * sqrt(2) / int_sqrt(accel * steps_per_mm / 1000)
	//     = F_CPU * sqrt(2) * sqrt(1000) / int_sqrt(accel * steps_per_mm)
	//     = F_CPU / int_sqrt(accel * steps_per_mm) * (20 * sqrt(5))
	//                                                 20.sqrt(5) ~= 313/7 (0.12%)
	//     = F_CPU / int_sqrt(accel * steps_per_mm) * 313 / 7
	//     2**32 / 313 is about 13MHz, so we can't start with F_CPU * 313 if F_CPU is above 13MHz
	if (x_delta) {
		printf("x_accel(%u) * X_STEPS_PER_MM(%u) = %u, sqrt() = %u\n", x_accel, ((uint32_t) X_STEPS_PER_MM), x_accel * ((uint32_t) X_STEPS_PER_MM), int_sqrt(x_accel * ((uint32_t) X_STEPS_PER_MM)));
		x_c = (F_CPU / int_sqrt(x_accel * ((uint32_t) X_STEPS_PER_MM))) * 313L / 7L;
		printf("Xc: %u\n", (F_CPU / int_sqrt(x_accel * ((uint32_t) X_STEPS_PER_MM))) * 313L / 7L);
// 		x_c = F_CPU * sqrt(2.0 / x_accel * X_UM_PER_STEP);
		x_minc = (F_CPU * X_UM_PER_STEP) / x_speed;
	}
	if (y_delta) {
		y_c = (F_CPU / int_sqrt(y_accel * Y_STEPS_PER_MM)) * 313L / 7L;
// 		y_c = F_CPU * sqrt(Y_UM_PER_STEP / y_accel) * 1.414;
		y_minc = (F_CPU * Y_UM_PER_STEP) / y_speed;
	}
	if (z_delta) {
		z_c = (F_CPU / int_sqrt(z_accel * Z_STEPS_PER_MM)) * 313L / 7L;
		z_minc = (F_CPU * Z_UM_PER_STEP) / z_speed;
	}
	if (e_delta) {
		e_c = (F_CPU / int_sqrt(e_accel * E_STEPS_PER_MM)) * 313L / 7L;
		e_minc = (F_CPU * E_UM_PER_STEP) / e_speed;
	}

	printf("Xc: %d, Yc: %d\n", x_c, y_c);
	printf("Xminc: %d, Yminc: %d\n", x_minc, y_minc);

	x_n = y_n = z_n = e_n = 1;

	x_cr = x_c; y_cr = y_c; z_cr = z_c; e_cr = e_c;

	total_ticks = 0;

	while (x_delta > 0 || y_delta > 0 || z_delta > 0 || e_delta > 0) {
		if (x_cr <= 0 && x_delta > 0) {
			x_delta--;
			if (x_n == 1)
				x_c = 0.4056 * x_c;
			else
				x_c = x_c - ((2 * x_c) / ((4 * x_n) + 1));
			if (x_c < x_minc)
				x_c = x_minc;
			x_cr = x_c;
			x_n++;
		}
		if (y_cr <= 0 && y_delta > 0) {
			y_delta--;
			if (y_n == 1)
				y_c = 0.4056 * y_c;
			else
				y_c = y_c - ((2 * y_c) / ((4 * y_n) + 1));
			if (y_c < y_minc)
				y_c = y_minc;
			y_cr = y_c;
			y_n++;
		}
		if (z_cr <= 0 && z_delta > 0) {
			z_delta--;
			if (z_n == 1)
				z_c = 0.4056 * z_c;
			else
				z_c = z_c - ((2 * z_c) / ((4 * z_n) + 1));
			if (z_c < z_minc)
				z_c = z_minc;
			z_cr = z_c;
			z_n++;
		}
		if (e_cr <= 0 && e_delta > 0) {
			e_delta--;
			if (e_n == 1)
				e_c = 0.4056 * e_c;
			else
				e_c = e_c - ((2 * e_c) / ((4 * e_n) + 1));
			if (e_c < e_minc)
				e_c = e_minc;
			e_cr = e_c;
			e_n++;
		}

// 		printf("[xc: %d, xd: %d, yc: %d, yd: %d, ", x_cr, x_delta, y_cr, y_delta);
		fprintf(stderr, "%u %.3f %.3f\n", total_ticks, x_delta * X_UM_PER_STEP, y_delta * Y_UM_PER_STEP);

		elapsed_ticks = 0x7FFFFFFF;
		if ((x_delta > 0) && (x_cr < elapsed_ticks))
			elapsed_ticks = x_cr;
		if ((y_delta > 0) && (y_cr < elapsed_ticks))
			elapsed_ticks = y_cr;
		if ((z_delta > 0) && (z_cr < elapsed_ticks))
			elapsed_ticks = z_cr;
		if ((e_delta > 0) && (e_cr < elapsed_ticks))
			elapsed_ticks = e_cr;

// 		printf("e: %u]\n", elapsed_ticks);

		x_cr -= elapsed_ticks;
		y_cr -= elapsed_ticks;
		z_cr -= elapsed_ticks;
		e_cr -= elapsed_ticks;

		total_ticks += elapsed_ticks;
	}
}

int main(int argc, char **argv) {
	float x = 40,
			y = 34,
			z = 0,
			e = 55,
			f = 1500;

	move(x * X_STEPS_PER_MM, y * Y_STEPS_PER_MM, z * Z_STEPS_PER_MM, e * E_STEPS_PER_MM, f);
}
