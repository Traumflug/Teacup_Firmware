#ifndef	_TIMER_H
#define	_TIMER_H

#include	<stdint.h>
#include	<avr/io.h>

// time-related constants
#define	US	* (F_CPU / 1000000)
#define	MS	* (F_CPU / 1000)

#define	DEFAULT_TICK	(100 US)
#define	WAITING_DELAY	(10 MS)

void setupTimerInterrupt(void);
uint8_t getTimerResolution(const uint32_t delay);
void setTimerResolution(uint8_t r);
uint16_t getTimerCeiling(const uint32_t delay);

void setTimer(uint32_t delay);

void delay(uint32_t delay);
void delayMicrosecondsInterruptible(unsigned int us);

inline void enableTimerInterrupt(void)
{
	TIMSK1 |= (1<<OCIE1A);
}

inline void disableTimerInterrupt(void)
{
	TIMSK1 &= ~(1<<OCIE1A);
}

#define setTimerCeiling(c)		OCR1A = c

#endif	/* _TIMER_H */
