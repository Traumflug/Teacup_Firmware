#ifndef _DDA_UTIL_H
#define _DDA_UTIL_H

#include	<stdint.h>

uint32_t approx_distance_2d( uint32_t dx, uint32_t dy );
uint32_t approx_distance_3d( uint32_t dx, uint32_t dy, uint32_t dz );

// const because return value is always the same given the same v
const uint8_t	msbloc (uint32_t v)																		__attribute__ ((const));

uint16_t int_sqrt( uint32_t a);

#endif

