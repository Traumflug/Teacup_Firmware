#include	"watchdog.h"

/** \file
	\brief Watchdog - reset if main loop doesn't run for too long

	The usefulness of this feature is questionable at best.

	What do you think will happen if your avr resets in the middle of a print?

	Is that preferable to it simply locking up?
*/

#ifdef USE_WATCHDOG

#include	<avr/wdt.h>
#include	<avr/interrupt.h>

#include	"arduino.h"
#ifndef	EXTRUDER
	#include	"serial.h"
#endif

volatile uint8_t	wd_flag = 0;

// uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
// void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
// void get_mcusr(void) {
// 	mcusr_mirror = MCUSR;
// 	MCUSR = 0;
// 	wdt_disable();
// }

ISR(WDT_vect) {
	// watchdog has tripped- no main loop activity for 0.5s, probably a bad thing
	// if watchdog fires again, we will reset
	// perhaps we should do something more intelligent in this interrupt?
	wd_flag |= 1;
}

/// intialise watchdog
void wd_init() {
	// check if we were reset by the watchdog
// 	if (mcusr_mirror & MASK(WDRF))
// 		serial_writestr_P(PSTR("Watchdog Reset!\n"));

	// 0.5s timeout, interrupt and system reset
	wdt_enable(WDTO_500MS);
	WDTCSR |= MASK(WDIE);
}

/// reset watchdog. MUST be called every 0.5s after init or avr will reset.
void wd_reset() {
	wdt_reset();
	if (wd_flag) {
		WDTCSR |= MASK(WDIE);
		wd_flag &= ~1;
	}
}

#endif /* USE_WATCHDOG */
