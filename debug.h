#ifndef	_DEBUG_H
#define	_DEBUG_H

#include	<stdint.h>
#include "sersendf.h"
#include "memory_barrier.h"
#include "pinio.h"

/** \def DEBUG
  Enable debug output generally and some additional M-codes.

  Individual flags are enabled at runtime using M111 (see gcode_process.c) or
  by modifying the presets here.

  WARNING: enabling some or all of these is done can produce /heaps/ of extra
  output. This extra output will break most host-side talkers that expect
  particular responses from firmware such as RepRap Host or ReplicatorG. Use
  with serial terminal or other suitable talker only.
*/
// #define DEBUG

#ifdef	DEBUG
  #define DEBUG_ECHO       1
  #define DEBUG_INFO       2
  #define DEBUG_ERRORS     4
  #define DEBUG_DRYRUN     8
  #define DEBUG_PID       16
  #define DEBUG_DDA       32
  #define DEBUG_POSITION  64
#else
	// by setting these to zero, the compiler should optimise the relevant code out
	#define		DEBUG_PID				0
	#define		DEBUG_DDA				0
	#define		DEBUG_POSITION	0
	#define		DEBUG_ECHO			0
  #define DEBUG_INFO       0
  #define DEBUG_DRYRUN     0
#endif


extern volatile uint8_t	debug_flags;

#define DEBUG_CM4
// #define DEBUG_SIM

#ifdef DEBUG_CM4
#define DEBUG_VARS \
        static uint32_t c_max = 0; \
        static uint32_t c_min = 0xFFFFFFFF; \
        static uint32_t c_all; \
        static uint32_t c_samples = 0; \
        uint32_t cycles;
        // static uint32_t debug_tick = 0;

#define DEBUG_INIT \
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#define DEBUG_START DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; \
  DWT->CYCCNT = 0;

#define DEBUG_END   cycles = DWT->CYCCNT; \
  DWT->CTRL &= ~(DWT_CTRL_CYCCNTENA_Msk); \
  cycles--; \
  if (c_max < cycles) c_max = cycles; \
  if (c_min > cycles) c_min = cycles; \
  c_samples++; \
  c_all += cycles;

#define DEBUG_PRINT \
  if (c_min < 0xFFFFFFFF) { \
    c_all /= c_samples; \
  sersendf_P(PSTR("min:%lu max:%lu avg:%lu n:%lu\n"), c_min, c_max, c_all, c_samples); \
  } \
  c_all = 0; \
  c_min = 0xFFFFFFFF; \
  c_max = 0; \
  c_samples = 0;
  
#elif DEBUG_SIM
#define DEBUG_VARS
#define DEBUG_PRINT
#define DEBUG_INIT

#define DEBUG_START  ATOMIC_START \
  WRITE(DEBUG_LED_PIN, 1);

#define DEBUG_END   WRITE(DEBUG_LED_PIN, 0); \
  ATOMIC_END
#else

#define DEBUG_VARS
#define DEBUG_PRINT
#define DEBUG_INIT
#define DEBUG_START
#define DEBUG_END

#endif

#endif	/* _DEBUG_H */
