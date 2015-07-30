
#ifndef _CPU_H
#define _CPU_H

#if defined __AVR__

  #include <avr/interrupt.h>

#elif defined __ARMEL__

  #include "cmsis-lpc11xx.h"  // For __ASM().

  /** Enable interrupts.

    This enables interrupts by clearing the I-bit in the CPSR.

    Code copied from MBED, __enable_irq(), in file
    mbed/libraries/mbed/targets/cmsis/core_cmFunc.h.
  */
  static void sei(void) __attribute__ ((always_inline));
  inline void sei(void) {
    __ASM volatile ("cpsie i" ::: "memory");
  }

  /** Disable interrupts.

    This disables interrupts by setting the I-bit in the CPSR.

    Code copied from MBED, __disable_irq(), in file
    mbed/libraries/mbed/targets/cmsis/core_cmFunc.h.
  */
  static void cli(void) __attribute__ ((always_inline));
  inline void cli(void) {
    __ASM volatile ("cpsid i" ::: "memory");
  }

#endif /* __AVR__, __ARMEL__ */

void cpu_init(void);

#endif /* _CPU_H */
