
/** \file

  \brief Display broker.

  Here we map generic display calls to calls to the actually used display and
  also define functions common to all displays.
*/

#include "display.h"

#define TEACUP_C_INCLUDE
  #include "display_ssd1306.c"
#undef TEACUP_C_INCLUDE


#ifdef DISPLAY

void display_writestr_P(PGM_P data_P) {
  uint8_t r, i = 0;

  // Yes, this is *supposed* to be assignment rather than comparison, so we
  // break when r is assigned zero.
  while ((r = pgm_read_byte(&data_P[i]))) {
    display_writechar(r);
    i++;
  }
}

#endif /* DISPLAY */
