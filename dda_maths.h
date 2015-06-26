#ifndef  _DDA_MATHS_H
#define  _DDA_MATHS_H

#include  <stdint.h>

#include  "config_wrapper.h"
#include "dda.h"

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

/*!
  Micrometer distance <=> motor step distance conversions.
*/

#define UM_PER_METER (1000000UL)

extern const axes_uint32_t PROGMEM axis_qn_P;
extern const axes_uint32_t PROGMEM axis_qr_P;

static int32_t um_to_steps(int32_t, enum axis_e) __attribute__ ((always_inline));
inline int32_t um_to_steps(int32_t distance, enum axis_e a) {
  return muldivQR(distance, pgm_read_dword(&axis_qn_P[a]),
                  pgm_read_dword(&axis_qr_P[a]), UM_PER_METER);
}

// approximate 2D distance
uint32_t approx_distance(uint32_t dx, uint32_t dy);

// approximate 3D distance
uint32_t approx_distance_3(uint32_t dx, uint32_t dy, uint32_t dz);

// integer square root algorithm
uint16_t int_sqrt(uint32_t a);

// integer inverse square root, 12bits precision
uint16_t int_inv_sqrt(uint16_t a);

// this is an ultra-crude pseudo-logarithm routine, such that:
// 2 ^ msbloc(v) >= v
const uint8_t msbloc (uint32_t v);

// Calculates acceleration ramp length in steps.
uint32_t acc_ramp_len(uint32_t feedrate, uint32_t steps_per_m);

// For X axis only, should become obsolete:
#define ACCELERATE_RAMP_LEN(speed) (((speed)*(speed)) / (uint32_t)((7200000.0f * ACCELERATION) / (float)STEPS_PER_M_X))

#endif  /* _DDA_MATHS_H */
