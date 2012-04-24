
/** \file
	\brief Mathematic algorithms for the digital differential analyser (DDA).
*/

#include "dda_maths.h"

#include <stdlib.h>
#include <stdint.h>

/*!
  Integer multiply-divide algorithm.

  \param multiplicand
  \param multiplier
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
const uint32_t muldiv(uint32_t multiplicand, uint32_t multiplier,
                      uint32_t divisor) {
  uint32_t quotient = 0;
  uint32_t remainder = 0;
  uint32_t qn = multiplier / divisor;
  uint32_t rn = multiplier % divisor;

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
  return quotient;
}

