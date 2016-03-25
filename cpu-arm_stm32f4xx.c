
/** \file
  \brief CPU initialisation, ARM specific part.

  To be included from cpu.c, for details see there.
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__
  
#include "config_wrapper.h"

void cpu_init() {
  /**
    Enable all periphals.
  */
    // Enable power and clocking for all GPIO
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | 
                  RCC_AHB1ENR_GPIOBEN | 
                  RCC_AHB1ENR_GPIOCEN | 
                  RCC_AHB1ENR_GPIODEN | 
                  RCC_AHB1ENR_GPIOHEN;
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
