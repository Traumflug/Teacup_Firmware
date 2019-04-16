#ifndef _DELAY_STM32_H
#define _DELAY_STM32_H

#include "cmsis-stm32f4xx.h" // For __ASM() and ...

/** Delay in microseconds.

  \param delay Time to wait in microseconds.

  Execution times on ARM aren't as predictable as they could be, because
  there's a code prefetch engine which can change timings depending on the
  position of the code in Flash. We could use the System Tick Timer for this
  task, but this timer is probably better used for more important tasks.
  delay_us() and delay_ms() are used only rarely and not in a way which would
  require high precision.
*/
static void _delay(uint32_t) __attribute__ ((always_inline));
inline void _delay(uint32_t delay)
{

    if (delay)
    {
        __ASM volatile(
            "1:  subs %0, %0, #1 \n\t"
            "    bne 1b\n\t"
            : "+r"(delay));
    }
    else
        __ASM volatile("nop \n\t");
}

// void _delay(uint32_t delay);
#define TUNER 1
#define delay_us(delay) (_delay((uint32_t)(((uint64_t)__SYSTEM_CLOCK) * (delay) / 3000000ULL * TUNER)))

#endif /* _DELAY_STM32_H */