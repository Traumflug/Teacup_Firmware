#ifndef	_DELAY_H
#define	_DELAY_H

#include	<stdint.h>
#include	<util/delay_basic.h>
#include	"watchdog.h"

#if F_CPU < 4000000UL
#error Delay functions only work with F_CPU >= 4000000UL 
#endif

// microsecond delay, does NOT reset WDT if feature enabled
void delay_us(uint16_t delay);

// microsecond delay, does reset WDT if feature enabled
void _delay(uint32_t delay);

// millisecond delay, does reset WDT if feature enabled
void delay_ms(uint32_t delay);


// microsecond timer, does reset WDT if feature enabled
// 0 results in no real delay, but the watchdog
// reset is called if the feature is enabled
static void delay(uint32_t) __attribute__ ((always_inline));
inline void delay(uint32_t d) {
	if (d > (65536L / (F_CPU / 4000000L))) {
		_delay(d);
	}
	else {
		wd_reset();
		if( d ) {
			_delay_loop_2(d * (F_CPU / 4000000L));
			wd_reset();
		}
	}
}

#endif	/* _DELAY_H */
