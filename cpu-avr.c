
/** \file
  \brief CPU initialisation, AVR specific part.

  To be included from cpu.c, for details see there.
*/

#if defined TEACUP_C_INCLUDE && defined __AVR__

#include <avr/io.h>
#include "pinio.h"


/**
  This sets up the CPU the way we need it. It disables modules we don't use,
  so they don't mess on the I/O pins they're connected to.
*/
void cpu_init() {
  #ifdef PRR
    #if defined TEMP_MAX6675 || defined SD
      PRR = MASK(PRTWI) | MASK(PRADC);
    #else
      PRR = MASK(PRTWI) | MASK(PRADC) | MASK(PRSPI);
    #endif
  #elif defined PRR0
    #if defined TEMP_MAX6675 || defined SD
      PRR0 = MASK(PRTWI) | MASK(PRADC);
    #else
      PRR0 = MASK(PRTWI) | MASK(PRADC) | MASK(PRSPI);
    #endif
    #if defined(PRUSART3)
      // Don't use USART2 or USART3. Leave USART1 for GEN3 and derivatives.
      PRR1 |= MASK(PRUSART3) | MASK(PRUSART2);
    #endif
    #if defined(PRUSART2)
      // Don't use USART2 or USART3. Leave USART1 for GEN3 and derivatives.
      PRR1 |= MASK(PRUSART2);
    #endif
  #endif
  ACSR = MASK(ACD);
}

#endif /* defined TEACUP_C_INCLUDE && defined __AVR__ */
