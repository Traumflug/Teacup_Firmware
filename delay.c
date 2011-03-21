#include	"delay.h"

/** \file
	\brief Delay routines
*/

#include	"watchdog.h"

/// delay microseconds
/// \param delay time to wait in microseconds
void delay(uint32_t delay) {
	wd_reset();
	while (delay > 65535) {
		delayMicrosecondsInterruptible(65533);
		delay -= 65535;
		wd_reset();
	}
	delayMicrosecondsInterruptible(delay & 0xFFFF);
	wd_reset();
}

/// delay milliseconds
/// \param delay time to wait in milliseconds
void delay_ms(uint32_t delay) {
	wd_reset();
	while (delay > 65) {
		delayMicrosecondsInterruptible(64999);
		delay -= 65;
		wd_reset();
	}
	delayMicrosecondsInterruptible(delay * 1000);
	wd_reset();
}

/// internal- wait for up to 65.5ms using a busy loop
/// \param us time to wait in microseconds
void delayMicrosecondsInterruptible(uint16_t us)
{
	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;

	// busy wait
	__asm__ __volatile__ ("1: sbiw %0,1" "\n\t" // 2 cycles
	"brne 1b" :
	"=w" (us) :
	"0" (us) // 2 cycles
	);
}
