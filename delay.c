#include	"delay.h"

/** \file
	\brief Delay routines
*/

#include	<stdint.h>
#include	<util/delay_basic.h>
#include	"watchdog.h"

#if F_CPU < 4000000UL
#error Delay functions only work with F_CPU >= 4000000UL 
#endif

/// delay microseconds
/// \param delay time to wait in microseconds
void delay_us(uint16_t delay) {
	wd_reset();
	while (delay > (65536L / (F_CPU / 4000000L))) {
		_delay_loop_2(65534); // we use 65534 here to compensate for the time that the surrounding loop takes. TODO: exact figure needs tuning
		delay -= (65536L / (F_CPU / 4000000L));
		wd_reset();
	}
	_delay_loop_2(delay * (F_CPU / 4000000L));
	wd_reset();
}

/// delay milliseconds
/// \param delay time to wait in milliseconds
void delay_ms(uint32_t delay) {
	wd_reset();
	while (delay > 65) {
		delay_us(64999);
		delay -= 65;
		wd_reset();
	}
	delay_us(delay * 1000);
	wd_reset();
}
