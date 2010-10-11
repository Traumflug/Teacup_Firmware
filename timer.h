#ifndef	_TIMER_H
#define	_TIMER_H

#include	<stdint.h>
#include	<avr/io.h>

// time-related constants
#define	US	* (F_CPU / 1000000)
#define	MS	* (F_CPU / 1000)

// #define	DEFAULT_TICK	(100 US)
#define	WAITING_DELAY	(10 MS)

void setupTimerInterrupt(void) __attribute__ ((cold));

uint8_t getTimerResolution(const uint32_t delay);
void setTimerResolution(uint8_t r);

uint16_t getTimerCeiling(const uint32_t delay);
#define setTimerCeiling(c)		OCR1A = c

void setTimer(uint32_t delay);

#define enableTimerInterrupt()	do { TIMSK1 |= (1<<OCIE1A); } while (0)
#define disableTimerInterrupt() do { TIMSK1 &= ~(1<<OCIE1A); } while (0)
#define timerInterruptIsEnabled() (TIMSK1 & (1 << OCIE1A))

#endif	/* _TIMER_H */
