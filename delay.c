#include	"delay.h"

/** \file
	\brief Delay routines
*/

#include "watchdog.h"


/** Delay in milliseconds.

  \param delay Time to wait in milliseconds.

  Accuracy on AVR, 20 MHz: delay < 0.04% too long over the whole range.
  Accuracy on AVR, 16 MHz: delay < 0.8% too short over the whole range.
  Accuracy on LPC1114, 48 MHz: delay < 0.1% too long over the whole range.
  Accuracy on STM32F411, 96 MHz: delay < 0.7% too short over the whole range.
*/
void delay_ms(uint32_t delay) {
	wd_reset();
	while (delay > 65) {
		delay_us(64999);
		delay -= 65;
		wd_reset();
	}
  delay_us(delay * 1000 - 2);
	wd_reset();
}
