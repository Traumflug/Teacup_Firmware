#ifndef	_TIMER_H
#define	_TIMER_H

#include	<stdint.h>
#ifndef SIMULATOR
#include	<avr/io.h>
#endif
#include "simulator.h"

// time-related constants
#define	US	* (F_CPU / 1000000)
#define	MS	* (F_CPU / 1000)

/// How often we overflow and update our clock.
/// With F_CPU = 16MHz, max is < 4.096ms (TICK_TIME = 65535).
#define TICK_TIME 2 MS

/// Convert back to ms from cpu ticks so our system clock runs
/// properly if you change TICK_TIME.
#define TICK_TIME_MS (TICK_TIME / (F_CPU / 1000))

/*
clock stuff
*/
extern volatile uint8_t	clock_flag_10ms;
extern volatile uint8_t	clock_flag_250ms;
extern volatile uint8_t	clock_flag_1s;

// If the specific bit is set, execute the following block exactly once
// and then clear the flag.
#define	ifclock(F)	for (;F;F=0 )

/*
timer stuff
*/
void timer_init(void) __attribute__ ((cold));

void setTimer(uint32_t delay);

void timer_stop(void);

uint32_t get_millis(void);

#endif	/* _TIMER_H */
