
/** \file
  \brief Delay routines, ARM specific part.
*/

#if defined TEACUP_C_INCLUDE && defined __ARMEL__

#include "cmsis-lpc11xx.h"  // For __ASM() and __SYSTEM_CLOCK.


/** Delay in microseconds.

  \param delay Time to wait in microseconds.

  Execution times on ARM aren't as predictable as they could be, because
  there's a code prefetch engine which can change timings depending on the
  position of the code in Flash. We could use the System Tick Timer for this
  task, but this timer is probably better used for more important tasks.
  delay_us() and delay_ms() are used only rarely and not in a way which would
  require high precision.

  Nevertheless, calibrated on the oscilloscope. Measured accuracy:

             delay_us(10)    ...(100)   ...(1000)  ...(10000)  ...(65000)
    48 MHz       10.82 us    100.9 us    1.002 ms    10.01 ms    65.10 ms

  CAUTION: this currently works for a 48 MHz clock, only! As other clock rates
           appear, there's more math neccessary, see the AVR version. Or simply
           a second implementation.
*/
void delay_us(uint16_t delay) {

  #if __SYSTEM_CLOCK == 48000000UL
    __ASM (".balign 16");  // No gambling with the prefetch engine.
    while (delay) {
      __ASM volatile (
        "   nop               \n\t"  // One nop before the loop slows about 2%.
        "   nop               \n\t"
        "   movs  r7, #4      \n\t"  // One more loop round slows about 20%
        "1: nop               \n\t"
        "   sub   r7, #1      \n\t"
        "   cmp   r7, #0      \n\t"
        "   bne   1b          \n\t"
        :
        :
        : "r7", "cc"
      );
      delay--;
    }
  #else
    #error No delay_us() implementation for this CPU clock frequency.
  #endif
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
