#ifndef _MEMORY_BARRIER_H_
#define _MEMORY_BARRIER_H_

#ifdef SIMULATOR
  #define CLI_SEI_BUG_MEMORY_BARRIER()
  #define MEMORY_BARRIER()

  #define ATOMIC_START { \
                         uint8_t save_reg = sim_interrupts; \
                         cli();

  #define ATOMIC_END     MEMORY_BARRIER(); \
                         if (save_reg) sei(); \
                       }

#else
#include <util/atomic.h>
#ifdef __AVR__
#include <avr/io.h>
#endif

// Provide a memory barrier to the compiler. This informs
// the compiler that is should write any cached values that
// are destined for a global variable and discard any other
// cached values from global variables.
//
// Note that this behavior does apply to all global variables,
// not just volatile ones. However, cached local variables
// are not affected as they are not externally visible.

#define MEMORY_BARRIER() __asm volatile( "" ::: "memory" )

// There is a bug in the CLI/SEI functions in older versions of
// avr-libc - they should be defined to include a memory barrier.
// This macro is used to define the barrier in the code so that 
// it will be easy to remove once the bug has become ancient history.
// At the moment the bug is included in most of the distributed
// compilers.

#if __AVR_LIBC_VERSION__ < 10700UL
	#define CLI_SEI_BUG_MEMORY_BARRIER() MEMORY_BARRIER()
#else
	#define CLI_SEI_BUG_MEMORY_BARRIER()
#endif

#if defined __AVR__
  #define ATOMIC_START { \
                         uint8_t save_reg = SREG; \
                         cli(); \
                         CLI_SEI_BUG_MEMORY_BARRIER();
#elif ! defined SIMULATOR
  #define ATOMIC_START cli();
#else
  #define ATOMIC_START
#endif

#if defined __AVR__
  #define ATOMIC_END   MEMORY_BARRIER(); \
                       SREG = save_reg; \
                     }
#elif ! defined SIMULATOR
  #define ATOMIC_END sei();
#else
  #define ATOMIC_END
#endif

#endif /* SIMULATOR */
#endif /* _MEMORY_BARRIER_H_ */
