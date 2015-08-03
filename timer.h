#ifndef	_TIMER_H
#define	_TIMER_H

#include	<stdint.h>
#include "arduino.h"  // For F_CPU on ARM.

// time-related constants
#define	US	* (F_CPU / 1000000)
#define	MS	* (F_CPU / 1000)

/// How often we overflow and update our clock.
/// With F_CPU = 16MHz, max is < 4.096ms (TICK_TIME = 65535).
#define TICK_TIME (2 MS)

/// Convert back to ms from cpu ticks so our system clock runs
/// properly if you change TICK_TIME.
#define TICK_TIME_MS (TICK_TIME / (F_CPU / 1000))


void timer_init(void);

uint8_t timer_set(int32_t delay, uint8_t check_short);

void timer_reset(void);

void timer_stop(void);

#endif	/* _TIMER_H */
