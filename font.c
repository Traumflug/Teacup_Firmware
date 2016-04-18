
/** \file

  \brief Font broker.

  Here we map generic font variables to the actually used font. This is done in
  a rather primitive way by having the same variable name in all font variants
  and wrapping each font in the corresponding #ifdef.
*/

#include "font.h"

#define TEACUP_C_INCLUDE
#include "font_8x4.c"
//#include "font_ another.c"
#undef TEACUP_C_INCLUDE
