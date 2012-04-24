#ifndef	_DDA_MATHS_H
#define	_DDA_MATHS_H

#include	<stdint.h>

#include	"config.h"

// return rounded result of multiplicand * multiplier / divisor
const int32_t muldiv(int32_t multiplicand, uint32_t multiplier,
                     uint32_t divisor);

// convert micrometer distances to motor step distances
#define um_to_steps_x(distance) muldiv(distance, STEPS_PER_M_X, 1000000UL);
#define um_to_steps_y(distance) muldiv(distance, STEPS_PER_M_Y, 1000000UL);
#define um_to_steps_z(distance) muldiv(distance, STEPS_PER_M_Z, 1000000UL);
#define um_to_steps_e(distance) muldiv(distance, STEPS_PER_M_E, 1000000UL);

#endif	/* _DDA_MATHS_H */
