#ifndef	_DELAY_H
#define	_DELAY_H

#include	<stdint.h>

void delay(uint32_t delay);

void delay_ms(uint32_t delay);

#define	delay_us(d) delayMicrosecondsInterruptible(d)
void delayMicrosecondsInterruptible(unsigned int us);

#endif	/* _DELAY_H */
