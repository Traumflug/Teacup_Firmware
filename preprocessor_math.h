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

  These two provide identical results up to 1'861'860 (tested at runtime) and
  up to 10'000'000 at compile time. At 90'000'000, deviation is about 5%,
  increasing further at even higher numbers. This "+ .5" is for rounding and
  not crucial. Casting to other sizes is also possible.

  Can be used for calculations at runtime, too, where it costs 944(!) bytes
  binary size and takes roughly 10'000(!) clock cycles.

  Initial version found on pl.comp.lang.c, posted by Jean-Louis PATANE.
*/
#define SQR00(x) (((double)(x)) / 2)
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
// You can add more of these lines here, each additional line increases
// accurate range by about factor 2 and costs additional 40 bytes binary size
// in the non-constant case. But beware, the length of the preprocessed term
// explodes, leading to several seconds compile time above about SQR13.
#define SQRT(x) ((SQR12(x) + ((x) / SQR12(x))) / 2)


#endif /* _PREPROCESSOR_MATH_H */
