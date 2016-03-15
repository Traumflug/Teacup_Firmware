#ifndef	_DDA_MATHS_H
#define	_DDA_MATHS_H

#include	<stdint.h>
#include	<math.h>
#include	"config_wrapper.h"
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

//32 bit square root
uint32_t SquareRoot32(uint32_t a_nInput);

// integer inverse square root, 12bits precision
uint16_t int_inv_sqrt(uint16_t a);

// this is an ultra-crude pseudo-logarithm routine, such that:
// 2 ^ msbloc(v) >= v
const uint8_t msbloc (uint32_t v);

// Calculates acceleration ramp length in steps.
uint32_t acc_ramp_len(uint32_t feedrate, uint32_t steps_per_m);

// For X axis only, should become obsolete:
#define ACCELERATE_RAMP_LEN(speed) (((speed)*(speed)) / (uint32_t)((7200000.0f * ACCELERATION) / (float)STEPS_PER_M_X))

// Compile-time trigonometric functions. See Taylor series.
// Converts degrees to radians.
#define DegToRad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
//Make an angle (in radian) coterminal (0 >= x > 2pi).
//#define COTERMINAL(x) (fmod((x), M_PI*2)) //This causes error since fmod() implementation is different from i386 compiler.
//TODO: Solve coterminality
#define COTERMINAL(x) (x)
// Get quadrant of an angle in radian.
#define QUADRANT(x) ((uint8_t)((COTERMINAL((x))) / M_PI_2) + 1)
//Calculate sine of an angle in radians.
//Formula will be adjusted accordingly to the angle's quadrant.
//Don't worry about pow() function here as it will be optimized by the compiler.
#define SIN0(x) (x)
#define SIN1(x) (SIN0(x) - (pow ((x), 3) / 6))
#define SIN2(x) (SIN1(x) + (pow ((x), 5) /  120))
#define SIN3(x) (SIN2(x) - (pow ((x), 7) /  5040))
#define SIN4(x) (SIN3(x) + (pow ((x), 9) /  362880))
//*
#define SIN(x) (QUADRANT((x)) == 1 ? (SIN4(COTERMINAL((x)))) : \
               (QUADRANT((x)) == 2 ? (SIN4(M_PI - COTERMINAL((x)))) : \
               (QUADRANT((x)) == 3 ? -(SIN4(COTERMINAL((x)) - M_PI)) : \
               -(SIN4(M_PI*2 - COTERMINAL((x)))))))
//               */
//#define SIN(x) (SIN4(x))
//Calculate cosine of an angle in radians.
//Formula will be adjusted accordingly to the angle's quadrant.
//Don't worry about pow() function here as it will be optimized by the compiler.
#define COS0(x) 1
#define COS1(x) (COS0(x) - (pow ((x), 2) /  2))
#define COS2(x) (COS1(x) + (pow ((x), 4) /  24))
#define COS3(x) (COS2(x) - (pow ((x), 6) /  720))
#define COS4(x) (COS3(x) + (pow ((x), 8) /  40320))
//*
#define COS(x) (QUADRANT((x)) == 1 ? (COS4(COTERMINAL((x)))) : \
               (QUADRANT((x)) == 2 ? -(COS4(M_PI - COTERMINAL((x)))) : \
               (QUADRANT((x)) == 3 ? -(COS4(COTERMINAL((x)) - M_PI)) : \
               (COS4(M_PI*2 - COTERMINAL((x)))))))
//               */
//#define COS(x) COS4((x))
#endif	/* _DDA_MATHS_H */
