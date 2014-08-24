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

/*! Preprocessor exponential function.

  EXP(i) equals exp(i)

  Works reasonable with doubles, only.

  The implementation uses the series found here ("Numerical value"):

    http://schools-wikipedia.org/wp/e/Exponential_function.htm

  which doesn't "explode" like the pattern for SQRT() above.
*/
#define EXP(x) ((x) > 0. ? EXPD(x) : 1 / EXPD(-(x)))
//#define EXP(x) ((x) > 0. ? EXPT07(x) : 1 / EXPT07(-(x)))
// For better convergence of large numbers you'd use these lines and shorten
// the series in EXPD() in real code, but as preprocessor macro they tend to
// "explode".
// They use this formula, found similarly in the HP-35 algorithm, too:
//   exp(x + ln(y)) = exp(x) * y
#define EXPT07(x) ((x) > 4.852030264 ? 128. * EXPT06((x) - 4.852030264) : EXPT06(x))
#define EXPT06(x) ((x) > 4.158883083 ? 64. * EXPT05((x) - 4.158883083) : EXPT05(x))
#define EXPT05(x) ((x) > 3.465735903 ? 32. * EXPT04((x) - 3.465735903) : EXPT04(x))
#define EXPT04(x) ((x) > 2.772588722 ? 16. * EXPT03((x) - 2.772588722) : EXPT03(x))
#define EXPT03(x) ((x) > 2.079441542 ? 8. * EXPT02((x) - 2.079441542) : EXPT02(x))
// See the pattern?      ^ ^ ^ ^ ^ ^ = ln(8.)
#define EXPT02(x) ((x) > 1.386294361 ? 4. * EXPT01((x) - 1.386294361) : EXPT01(x))
#define EXPT01(x) ((x) > 0.693147180 ? 2. * EXPD((x) - 0.693147180) : EXPD(x))
#define EXPD(x) ((double)1. + (x) * ( \
                 1. + (x) / 2. * ( \
                 1. + (x) / 3. * ( \
                 1. + (x) / 4. * ( \
                 1. + (x) / 5. * ( \
                 1. + (x) / 6. * ( \
                 1. + (x) / 7. * ( \
                 1. + (x) / 8. * ( \
                 1. + (x) / 9. * ( \
                 1. + (x) / 10. * ( \
                 1. + (x) / 11. * ( \
                 1. + (x) / 12. * ( \
                 1. + (x) / 13. * ( \
                 1. + (x) / 14. * ( \
                 1. + (x) / 15. * ( \
                 1. + (x) / 16. * ( \
                 1. + (x) / 17. * ( \
                 1. + (x) / 18. * ( \
                 1. + (x) / 19. * ( \
                 1. + (x) / 20. * ( \
                 1. + (x) / 21. * ( \
                 1. + (x) / 22. * ( \
                 1. + (x) / 23. * ( \
                 1. + (x) / 24. * ( \
                 1. + (x) / 25. * ( \
                 1. + (x) / 26. * ( \
                 1. + (x) / 27. * ( \
                 1. + (x) / 28. * ( \
                 1. + (x) / 29. * ( \
                 1. + (x) / 30. * ( \
                 1. + (x) / 31. * ( \
                 1. + (x) / 32. * ( \
                 1. + (x) / 33. * ( \
                 1. + (x) / 34. * ( \
                 1. + (x) / 35. * ( \
                 1. + (x) / 36. * ( \
                 1. + (x) / 37. * ( \
                 1. + (x) / 38. * ( \
                 1. + (x) / 39. * ( \
                 1. + (x) / 40.))))))))))))))))))))))))))))))))))))))))

#endif /* _PREPROCESSOR_MATH_H */
