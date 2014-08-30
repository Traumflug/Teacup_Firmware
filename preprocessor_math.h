/** \file

  \brief Math functions meant to be calculated in the C preprocessor.

  \details Math functions presented here avoid library calls, which means they
           can be solved at compile time and as such used to initialise
           constants. When used for intitialising, their cost at runtime is
           zero, as they resolve into a single number.
*/

#ifndef _PREPROCESSOR_MATH_H
#define _PREPROCESSOR_MATH_H


/*! Preprocessor square root.

  (uint32_t)(SQRT(i) + .5)
  equals
  (uint32_t)(sqrt(i) + .5)

  These two provide identical results for all tested numbers across the
  uint32 range. Casting to other sizes is also possible.

  Can principally be used for calculations at runtime, too, but its compiled
  size is prohibitively large (more than 20kB per instance).

  Initial version found on pl.comp.lang.c, posted by Jean-Louis PATANE.
*/
#define SQR00(x) (((x) > 65535) ? (double)65535 : (double)(x) / 2)
#define SQR01(x) ((SQR00(x) + ((x) / SQR00(x))) / 2)
#define SQR02(x) ((SQR01(x) + ((x) / SQR01(x))) / 2)
#define SQR03(x) ((SQR02(x) + ((x) / SQR02(x))) / 2)
#define SQR04(x) ((SQR03(x) + ((x) / SQR03(x))) / 2)
#define SQR05(x) ((SQR04(x) + ((x) / SQR04(x))) / 2)
#define SQR06(x) ((SQR05(x) + ((x) / SQR05(x))) / 2)
#define SQR07(x) ((SQR06(x) + ((x) / SQR06(x))) / 2)
#define SQR08(x) ((SQR07(x) + ((x) / SQR07(x))) / 2)
#define SQR09(x) ((SQR08(x) + ((x) / SQR08(x))) / 2)
#define SQR10(x) ((SQR09(x) + ((x) / SQR09(x))) / 2)
#define SQR11(x) ((SQR10(x) + ((x) / SQR10(x))) / 2)
#define SQR12(x) ((SQR11(x) + ((x) / SQR11(x))) / 2)
// We use 9 iterations, note how SQR10() and up get ignored. You can add more
// iterations here, but beware, the length of the preprocessed term
// explodes, leading to several seconds compile time above about SQR10().
#define SQRT(x) ((SQR09(x) + ((x) / SQR09(x))) / 2)


#endif /* _PREPROCESSOR_MATH_H */
