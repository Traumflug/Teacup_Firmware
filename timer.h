#ifndef	_TIMER_H
#define	_TIMER_H

#include	<stdint.h>
#include	<avr/io.h>

// time-related constants
#define	US	* (F_CPU / 1000000)
#define	MS	* (F_CPU / 1000)

/*
clock stuff
*/
extern volatile uint8_t	clock_flag_10ms;
extern volatile uint8_t	clock_flag_250ms;
extern volatile uint8_t	clock_flag_1s;

extern volatile uint8_t	timer1_compa_deferred_enable;

// If the specific bit is set, execute the following block exactly once
// and then clear the flag.
#define	ifclock(F)	for (;F;F=0 )

/*
timer stuff
*/
void timer_init(void) __attribute__ ((cold));

void setTimer(uint32_t delay);

void timer_stop(void);

#endif	/* _TIMER_H */
