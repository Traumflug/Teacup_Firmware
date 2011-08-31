
#include "dda_util.h"

/*
	utility functions
*/

// courtesy of http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
/*! linear approximation 2d distance formula
	\param dx distance in X plane [um]
	\param dy distance in Y plane [um]
	\return 3-part linear approximation of \f$\sqrt{\Delta x^2 + \Delta y^2}\f$

	see http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
*/

// Instruct compiler to take extra effort in optimizing this code
uint32_t approx_distance_2d( uint32_t dx, uint32_t dy) __attribute__ ((hot));

uint32_t approx_distance_2d( uint32_t dx, uint32_t dy)
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

// Instruct compiler to take extra effort in optimizing this code
uint32_t approx_distance_3d( uint32_t dx, uint32_t dy, uint32_t dz ) __attribute__ ((hot));

uint32_t approx_distance_3d( uint32_t dx, uint32_t dy, uint32_t dz )
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


