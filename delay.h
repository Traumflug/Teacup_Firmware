#ifndef	_DELAY_H
#define	_DELAY_H

#include	<stdint.h>

// microsecond delay, does NOT reset WDT if feature enabled
void delay_us(uint16_t delay);

// millisecond delay, does reset WDT if feature enabled
void delay_ms(uint32_t delay);

#endif	/* _DELAY_H */
