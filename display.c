
/** \file

  \brief Display broker.

  Here we map generic display calls to calls to the actually used display and
  also define functions common to all displays.
*/

#include "display.h"

#define TEACUP_C_INCLUDE
  #include "display_ssd1306.c"
#undef TEACUP_C_INCLUDE

/* No common code so far. */
