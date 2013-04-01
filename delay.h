#ifndef	_DELAY_H
#define	_DELAY_H

#include	<stdint.h>

// microsecond delay, does NOT reset WDT if feature enabled
void delay_us(uint16_t delay);

// millisecond delay, does reset WDT if feature enabled
void delay_ms(uint32_t delay);

#if !defined(__AVR__)
#if !defined(__DOXYGEN__)
static inline void _delay_loop_1(uint8_t __count) __attribute__((always_inline));
static inline void _delay_loop_2(uint16_t __count) __attribute__((always_inline));
#endif

/** \ingroup util_delay_basic

    Delay loop using an 8-bit counter \c __count, so up to 256
    iterations are possible.  (The value 256 would have to be passed
    as 0.)  The loop executes three CPU cycles per iteration, not
    including the overhead the compiler needs to setup the counter
    register.

    Thus, at a CPU speed of 1 MHz, delays of up to 768 microseconds
    can be achieved.
*/
void
_delay_loop_1(uint8_t __count)
{
        __asm__ volatile (
                "1: dec %0" "\n\t"
                "brne 1b"
                : "=r" (__count)
                : "0" (__count)
        );
}

/** \ingroup util_delay_basic

    Delay loop using a 16-bit counter \c __count, so up to 65536
    iterations are possible.  (The value 65536 would have to be
    passed as 0.)  The loop executes four CPU cycles per iteration,
    not including the overhead the compiler requires to setup the
    counter register pair.

    Thus, at a CPU speed of 1 MHz, delays of up to about 262.1
    milliseconds can be achieved.
 */
void
_delay_loop_2(uint16_t __count)
{
        __asm__ volatile (
                "1: sbiw %0,1" "\n\t"
                "brne 1b"
                : "=w" (__count)
                : "0" (__count)
        );
}

#endif
#endif	/* _DELAY_H */
