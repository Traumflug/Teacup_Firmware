#include	"pinout.h"

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

void x_step() {
	WRITE(DIO0, 1);
	delayMicrosecondsInterruptible(5);
	WRITE(DIO0, 0);
}

void y_step() {
	WRITE(DIO4, 1);
	delayMicrosecondsInterruptible(5);
	WRITE(DIO4, 0);
}

void z_step() {
	WRITE(DIO8, 1);
	delayMicrosecondsInterruptible(5);
	WRITE(DIO8, 0);
}

void e_step() {
	WRITE(DIO12, 1);
	delayMicrosecondsInterruptible(5);
	WRITE(DIO12, 0);
}
