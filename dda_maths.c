
/** \file
  \brief Mathematic algorithms for the digital differential analyser (DDA).
*/

#include "dda_maths.h"

#include <stdlib.h>
#include <stdint.h>

/*!
  Pre-calculated constant values for axis um <=> steps conversions.

  These should be calculated at run-time once in dda_init() if the
  STEPS_PER_M_* constants are ever replaced with run-time options.
*/
const axes_uint32_t PROGMEM axis_qn_P = {
  (uint32_t)STEPS_PER_M_X / UM_PER_METER,
  (uint32_t)STEPS_PER_M_Y / UM_PER_METER,
  (uint32_t)STEPS_PER_M_Z / UM_PER_METER,
  (uint32_t)STEPS_PER_M_E / UM_PER_METER
};

const axes_uint32_t PROGMEM axis_qr_P = {
  (uint32_t)STEPS_PER_M_X % UM_PER_METER,
  (uint32_t)STEPS_PER_M_Y % UM_PER_METER,
  (uint32_t)STEPS_PER_M_Z % UM_PER_METER,
  (uint32_t)STEPS_PER_M_E % UM_PER_METER
};

/*!
  Integer multiply-divide algorithm. Returns the same as muldiv(multiplicand, multiplier, divisor), but also allowing to use precalculated quotients and remainders.

  \param multiplicand
  \param qn ( = multiplier / divisor )
  \param rn ( = multiplier % divisor )
  \param divisor
  \return rounded result of multiplicand * multiplier / divisor

  Calculate a * b / c, without overflowing and without using 64-bit integers.
  Doing this the standard way, a * b could easily overflow, even if the correct
  overall result fits into 32 bits. This algorithm avoids this intermediate
  overflow and delivers valid results for all cases where each of the three
  operators as well as the result fits into 32 bits.

  Found on  http://stackoverflow.com/questions/4144232/
  how-to-calculate-a-times-b-divided-by-c-only-using-32-bit-integer-types-even-i
*/
const int32_t muldivQR(int32_t multiplicand, uint32_t qn, uint32_t rn,
                       uint32_t divisor) {
  uint32_t quotient = 0;
  uint32_t remainder = 0;
  uint8_t negative_flag = 0;

  if (multiplicand < 0) {
    negative_flag = 1;
    multiplicand = -multiplicand;
  }

  while(multiplicand) {
    if (multiplicand & 1) {
      quotient += qn;
      remainder += rn;
      if (remainder >= divisor) {
        quotient++;
        remainder -= divisor;
      }
    }
    multiplicand  >>= 1;
    qn <<= 1;
    rn <<= 1;
    if (rn >= divisor) {
      qn++;
      rn -= divisor;
    }
  }

  // rounding
  if (remainder > divisor / 2)
    quotient++;

  // remainder is valid here, but not returned
  return negative_flag ? -((int32_t)quotient) : (int32_t)quotient;
}

// courtesy of http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
/*! linear approximation 2d distance formula
  \param dx distance in X plane
  \param dy distance in Y plane
  \return 3-part linear approximation of \f$\sqrt{\Delta x^2 + \Delta y^2}\f$

  see http://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
*/
uint32_t approx_distance(uint32_t dx, uint32_t dy) {
  uint32_t min, max, approx;

  // If either axis is zero, return the other one.
  if (dx == 0 || dy == 0) return dx + dy;

  if ( dx < dy ) {
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
uint32_t approx_distance_3(uint32_t dx, uint32_t dy, uint32_t dz) {
  uint32_t min, med, max, approx;

  if ( dx < dy ) {
    min = dy;
    med = dx;
  } else {
    min = dx;
    med = dy;
  }

  if ( dz < min ) {
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

#if __FPU_PRESENT
uint_fast16_t int_f_sqrt(uint32_t a) {
  // using FPUs floating square root to return an unsigned fast16 bit result
  float result;

  __ASM volatile ("vcvt.f32.u32 %[result], %[a];\n"
                  "  vsqrt.f32 %[result], %[result];"
                  : [result]"=t" (result)
                  : [a]"t" (a));

  return (uint_fast16_t)(result);
}
#endif /* __FPU_PRESENT */
/*!
  integer square root algorithm
  \param a find square root of this number
  \return sqrt(a - 1) < returnvalue <= sqrt(a)

  This is a binary search but it uses only the minimum required bits for
  each step.
*/
uint16_t int_sqrt(uint32_t a) {
  uint16_t b = a >> 16;
  uint8_t c = b >> 8;
  uint16_t x = 0;
  uint8_t z = 0;
  uint16_t i;
  uint8_t j;

  for (j = 0x8; j; j >>= 1) {
    uint8_t y2;

    z |= j;
    y2 = z * z;
    if (y2 > c)
      z ^= j;
  }

  x = z << 4;
  for(i = 0x8; i; i >>= 1) {
    uint16_t y2;

    x |= i;
    y2 = x * x;
    if (y2 > b)
      x ^= i;
  }

  x <<= 8;
  for(i = 0x80; i; i >>= 1) {
    uint32_t y2;

    x |= i;
    y2 = (uint32_t)x * x;
    if (y2 > a)
      x ^= i;
  }

  return x;
}

/*!
  integer inverse square root algorithm
  \param a find the inverse of the square root of this number
  \return 0x1000 / sqrt(a) - 1 < returnvalue <= 0x1000 / sqrt(a)

  This is a binary search but it uses only the minimum required bits for each step.
*/
uint16_t int_inv_sqrt(uint16_t a) {
  /// 16bits inverse (much faster than doing a full 32bits inverse)
  /// the 0xFFFFU instead of 0x10000UL hack allows using 16bits and 8bits
  /// variable for the first 8 steps without overflowing and it seems to
  /// give better results for the ramping equation too :)
  uint8_t z = 0, i;
  uint16_t x, j;
  uint32_t q = ((uint32_t)(0xFFFFU / a)) << 8;

  for (i = 0x80; i; i >>= 1) {
    uint16_t y;

    z |= i;
    y = (uint16_t)z * z;
    if (y > (q >> 8))
      z ^= i;
  }

  x = z << 4;
  for (j = 0x8; j; j >>= 1) {
    uint32_t y;

    x |= j;
    y = (uint32_t)x * x;
    if (y > q)
      x ^= j;
  }

  return x;
}

// this is an ultra-crude pseudo-logarithm routine, such that:
// 2 ^ msbloc(v) >= v
/*! crude logarithm algorithm
  \param v value to find \f$log_2\f$ of
  \return floor(log(v) / log(2))
*/
const uint8_t msbloc (uint32_t v) {
  uint8_t i;
  uint32_t c;
  for (i = 31, c = 0x80000000; i; i--) {
    if (v & c)
      return i;
    c >>= 1;
  }
  return 0;
}

/*!
  Pre-calculated constant values for acceleration ramp calculations.
*/
static const axes_uint32_t PROGMEM acc_ramp_div_P = {
  (uint32_t)((7200000.0f * ACCELERATION_X) / STEPS_PER_M_X),
  (uint32_t)((7200000.0f * ACCELERATION_Y) / STEPS_PER_M_Y),
  (uint32_t)((7200000.0f * ACCELERATION_Z) / STEPS_PER_M_Z),
  (uint32_t)((7200000.0f * ACCELERATION_E) / STEPS_PER_M_E)
};

/*! Acceleration ramp length in steps.
 * \param feedrate Target feedrate of the accelerateion.
 * \param fast_axis Number of the fastest axis.
 * \return Accelerating steps neccessary to achieve target feedrate.
 *
 * s = 1/2 * a * t^2, v = a * t ==> s = v^2 / (2 * a)
 * 7200000 = 60 * 60 * 1000 * 2 (mm/min -> mm/s, steps/m -> steps/mm, factor 2)
 *
 * Note: this function has shown to be accurate between 10 and 10'000 mm/s2 and
 *       2000 to 4096000 steps/m (and higher). The numbers are a few percent
 *       too high at very low acceleration. Test code see commit message.
 */
uint32_t acc_ramp_len(uint32_t feedrate, uint8_t fast_axis) {
  return (feedrate * feedrate) / pgm_read_dword(&acc_ramp_div_P[fast_axis]);
}

