
/** \file
  \brief CPU initialisation.

  Functions to bring the CPU to a working state.
*/

#include "cpu.h"

#define TEACUP_C_INCLUDE
#include "cpu-avr.c"
#include "cpu-arm_lpc11xx.c"
#undef TEACUP_C_INCLUDE

/* No common code so far. */
