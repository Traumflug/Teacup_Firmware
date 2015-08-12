
/** \file
  \brief CPU initialisation, ARM specific part.

  To be included from cpu.c, for details see there.
*/

#if defined TEACUP_C_INCLUDE && defined __ARMEL__

#include "config_wrapper.h"

void cpu_init() {
  /**
    Other than on ATmegas, the LPC11xx disables all peripherals except GPIO
    and SPI0 on reset. Brown Out detector is set to kick in at 1.46 V, without
    causing a reset. See chapter 3.5.14 in the LPC11xx User Manual.

    GPIO is needed in all configurations, so we just turn off SPI0 if not
    needed.
  */
  #ifndef SPI
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 11);
  #endif
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
