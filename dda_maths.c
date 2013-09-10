
/** \file
  \brief Mathematic algorithms for the digital differential analyser (DDA).
*/

#include "dda_maths.h"

#include <stdlib.h>
#include <stdint.h>

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
