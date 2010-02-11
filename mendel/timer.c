#include	"timer.h"

#include	<avr/interrupt.h>

#include	"pinout.h"
#include	"dda_queue.h"
#include	"dda.h"
#include	"watchdog.h"

ISR(TIMER1_COMPA_vect) {
	WRITE(SCK, 1);
	// do our next step
	// NOTE: dda_step makes this interrupt interruptible after steps have been sent but before new speed is calculated.
	if (movebuffer[mb_tail].live)
		dda_step(&(movebuffer[mb_tail]));

	#if STEP_INTERRUPT_INTERRUPTIBLE
		// this interrupt can now be interruptible
		disableTimerInterrupt();
		sei();
	#endif

	// fall directly into dda_start instead of waiting for another step
	if (movebuffer[mb_tail].live == 0)
		next_move();

	#if STEP_INTERRUPT_INTERRUPTIBLE
		// return from interrupt in a way that prevents this interrupt nesting with itself at high step rates
		cli();
		// check queue, if empty we don't need to interrupt again until re-enabled in dda_create
		if (queue_empty() == 0)
			enableTimerInterrupt();
	#endif
	WRITE(SCK, 0);
}

void setupTimerInterrupt()
{
	// no outputs
	TCCR1A = 0;
	// CTC mode
	TCCR1B = MASK(WGM12);
	// no interrupts yet
	TIMSK1 = 0;

	//start off with a slow frequency.
	setTimer(10000);
}

// the following are all from reprap project 5D firmware

uint8_t getTimerResolution(const uint32_t delay)
{
	// these also represent frequency: 1000000 / delay / 2 = frequency in hz.

	// our slowest speed at our highest resolution ( (2^16-1) * 0.0625 usecs = 4095 usecs (4 millisecond max))
	// range: 8Mhz max - 122hz min
	if (delay <= 65535L)
		return 1;
	// our slowest speed at our next highest resolution ( (2^16-1) * 0.5 usecs = 32767 usecs (32 millisecond max))
	// range:1Mhz max - 15.26hz min
	else if (delay <= 524280L)
		return 2;
	// our slowest speed at our medium resolution ( (2^16-1) * 4 usecs = 262140 usecs (0.26 seconds max))
	// range: 125Khz max - 1.9hz min
	else if (delay <= 4194240L)
		return 3;
	// our slowest speed at our medium-low resolution ( (2^16-1) * 16 usecs = 1048560 usecs (1.04 seconds max))
	// range: 31.25Khz max - 0.475hz min
	else if (delay <= 16776960L)
		return 4;
	// our slowest speed at our lowest resolution ((2^16-1) * 64 usecs = 4194240 usecs (4.19 seconds max))
	// range: 7.812Khz max - 0.119hz min
// 	else if (delay <= 67107840L)
// 		return 5;
	//its really slow... hopefully we can just get by with super slow.
// 	else
	return 5;
}

void setTimerResolution(uint8_t r)
{
	// assuming CS10,CS11,CS12 are adjacent bits in platform endian order,
	TCCR1B = (TCCR1B & ~(MASK(CS12) | MASK(CS11) | MASK(CS10))) | (r << CS10);
}

uint16_t getTimerCeiling(const uint32_t delay)
{
	// our slowest speed at our highest resolution ( (2^16-1) * 0.0625 usecs = 4095 usecs)
	if (delay <= 65535L)
		return (delay & 0xffff);
	// our slowest speed at our next highest resolution ( (2^16-1) * 0.5 usecs = 32767 usecs)
	else if (delay <= 524280L)
		return ((delay >> 3) & 0xffff);
	// our slowest speed at our medium resolution ( (2^16-1) * 4 usecs = 262140 usecs)
	else if (delay <= 4194240L)
		return ((delay >> 6) & 0xffff);
	// our slowest speed at our medium-low resolution ( (2^16-1) * 16 usecs = 1048560 usecs)
	else if (delay <= 16776960L)
		return ((delay >> 8) & 0xffff);
	// our slowest speed at our lowest resolution ((2^16-1) * 64 usecs = 4194240 usecs)
	else if (delay <= 67107840L)
		return ((delay >> 10) & 0xffff);
	//its really slow... hopefully we can just get by with super slow.
	else
		return 65535;
}


// Depending on how much work the interrupt function has to do, this is
// pretty accurate between 10 us and 0.1 s.  At fast speeds, the time
// taken in the interrupt function becomes significant, of course.

// Note - it is up to the user to call enableTimerInterrupt() after a call
// to this function.

void setTimer(uint32_t delay)
{
	// delay is the delay between steps in microsecond ticks.
	//
	// we break it into 5 different resolutions based on the delay.
	// then we set the resolution based on the size of the delay.
	// we also then calculate the timer ceiling required. (ie what the counter counts to)
	// the result is the timer counts up to the appropriate time and then fires an interrupt.

	// Actual ticks are 0.0625 us, so multiply delay by 16

	// convert to ticks
	delay = delay US;

	setTimerCeiling(getTimerCeiling(delay));
	setTimerResolution(getTimerResolution(delay));
}

// delay( microseconds )
void delay(uint32_t delay) {
	wd_reset();
	while (delay > 65535) {
		delayMicrosecondsInterruptible(65534);
		delay -= 65535;
		wd_reset();
	}
	delayMicrosecondsInterruptible(delay & 0xFFFF);
	wd_reset();
}

// delay_ms( milliseconds )
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
