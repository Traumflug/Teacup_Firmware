
/** \file
  \brief Timer management - step pulse clock and system clock.

  Implementations for AVR and ARM are very different, so see the platform
  specific files for details.
*/

#include "timer.h"

#define TEACUP_C_INCLUDE
#include "timer-avr.c"
#include "timer-arm.c"
#undef TEACUP_C_INCLUDE

// No common code so far.
