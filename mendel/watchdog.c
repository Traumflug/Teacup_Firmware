#include	"watchdog.h"

#include	<avr/wdt.h>
#include	<avr/interrupt.h>

#include	"arduino.h"

volatile uint8_t	wd_flag = 0;

ISR(WDT_vect) {
	// watchdog has tripped- no main loop activity for 0.25s, probably a bad thing
	wd_flag |= 1;
}

void wd_init() {
	// 0.25s timeout, interrupt and system reset
	wdt_enable(WDTO_250MS);
	WDTCSR |= MASK(WDIE);
}

void wd_reset() {
	wdt_reset();
	if (wd_flag) {
		WDTCSR |= MASK(WDIE);
		wd_flag &= ~1;
	}
}
