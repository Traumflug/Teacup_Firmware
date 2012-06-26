#ifndef	_DDA_MATHS_H
#define	_DDA_MATHS_H

#include	<stdint.h>

#include	"config.h"

// return rounded result of multiplicand * multiplier / divisor
#define muldiv(multiplicand, multiplier, divisor) \
  muldivQR(multiplicand, multiplier / divisor, multiplier % divisor, divisor)

// same as before, but with quotient and remainder precalculated elsewhere
const int32_t muldivQR(int32_t multiplicand, uint32_t qn, uint32_t rn,
                       uint32_t divisor);

// convert micrometer distances to motor step distances
#define um_to_steps_x(distance) muldivQR(distance, \
  STEPS_PER_M_X / 1000000UL, STEPS_PER_M_X % 1000000UL, 1000000UL);
#define um_to_steps_y(distance) muldivQR(distance, \
  STEPS_PER_M_Y / 1000000UL, STEPS_PER_M_Y % 1000000UL, 1000000UL);
#define um_to_steps_z(distance) muldivQR(distance, \
  STEPS_PER_M_Z / 1000000UL, STEPS_PER_M_Z % 1000000UL, 1000000UL);
#define um_to_steps_e(distance) muldivQR(distance, \
  STEPS_PER_M_E / 1000000UL, STEPS_PER_M_E % 1000000UL, 1000000UL);

#endif	/* _DDA_MATHS_H */
