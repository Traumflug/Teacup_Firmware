
/** \file
  \brief I2C / TWI subsystem

  Implementations for AVR and ARM are very different, so see the platform
  specific files for details.
*/

#include "i2c.h"

#define TEACUP_C_INCLUDE
#include "i2c-avr.c"
// #include "i2c-arm_lpc11xx.c"
// #include "i2c-arm_stm32f4xx.c"
#undef TEACUP_C_INCLUDE

// No common code so far.
