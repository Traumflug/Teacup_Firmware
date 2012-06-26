#ifndef	_DDA_MATHS_H
#define	_DDA_MATHS_H

#include	<stdint.h>

#include	"config.h"

// return rounded result of multiplicand * multiplier / divisor
// this version is with quotient and remainder precalculated elsewhere
const int32_t muldivQR(int32_t multiplicand, uint32_t qn, uint32_t rn,
                       uint32_t divisor);

// return rounded result of multiplicand * multiplier / divisor
static int32_t muldiv(int32_t, uint32_t, uint32_t) __attribute__ ((always_inline));
inline int32_t muldiv(int32_t multiplicand, uint32_t multiplier,
                      uint32_t divisor) {
  return muldivQR(multiplicand, multiplier / divisor,
                  multiplier % divisor, divisor);
}

/*
	micrometer distance <=> motor step distance conversions
*/
// Like shown in the patch attached to this post:
// http://forums.reprap.org/read.php?147,89710,130225#msg-130225 ,
// it might be worth pre-calculating muldivQR()'s qn and rn in dda_init()
// as soon as STEPS_PER_M_{XYZE} is no longer a compile-time variable.

static int32_t um_to_steps_x(int32_t) __attribute__ ((always_inline));
inline int32_t um_to_steps_x(int32_t distance) {
    return muldivQR(distance, STEPS_PER_M_X / 1000000UL,
                    STEPS_PER_M_X % 1000000UL, 1000000UL);
}

static int32_t um_to_steps_y(int32_t) __attribute__ ((always_inline));
inline int32_t um_to_steps_y(int32_t distance) {
    return muldivQR(distance, STEPS_PER_M_Y / 1000000UL,
                    STEPS_PER_M_Y % 1000000UL, 1000000UL);
}

static int32_t um_to_steps_z(int32_t) __attribute__ ((always_inline));
inline int32_t um_to_steps_z(int32_t distance) {
    return muldivQR(distance, STEPS_PER_M_Z / 1000000UL,
                    STEPS_PER_M_Z % 1000000UL, 1000000UL);
}

static int32_t um_to_steps_e(int32_t) __attribute__ ((always_inline));
inline int32_t um_to_steps_e(int32_t distance) {
    return muldivQR(distance, STEPS_PER_M_E / 1000000UL,
                    STEPS_PER_M_E % 1000000UL, 1000000UL);
}

#endif	/* _DDA_MATHS_H */
