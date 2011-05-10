#ifndef	_DELAY_H
#define	_DELAY_H

#include	<stdint.h>
#include	<util/delay_basic.h>

#define		WAITING_DELAY		10000

void delay_us(uint16_t delay);

void _delay_ms(uint32_t delay);

static void delay(uint32_t) __attribute__ ((always_inline));
inline void delay(uint32_t d) {
	if (d >= (65536L / (F_CPU / 4000000L))) {
		delay_us(d);
	}
	else
		_delay_loop_2(d * (F_CPU / 4000000L));
}

static void delay_ms(uint32_t) __attribute__ ((always_inline));
inline void delay_ms(uint32_t d) {
	if (d > 65)
		delay_ms(d);
	else
		delay_us(d * 1000);
}

#endif	/* _DELAY_H */
