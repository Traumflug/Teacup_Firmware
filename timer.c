#include	"timer.h"

#include	<avr/interrupt.h>

#include	"dda_queue.h"
#include	"watchdog.h"

ISR(TIMER1_COMPA_vect) {
	WRITE(SCK, 1);

	queue_step();

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
}

// the following are all from reprap project 5D firmware with some modification to reduce redundancy

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
	//its really slow... hopefully we can just get by with super slow.
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
	// delay is the delay between steps in IOclk ticks.
	//
	// we break it into 5 different resolutions based on the delay.
	// then we set the resolution based on the size of the delay.
	// we also then calculate the timer ceiling required. (ie what the counter counts to)
	// the result is the timer counts up to the appropriate time and then fires an interrupt.

	setTimerResolution(0);													// stop timer
	GTCCR = MASK(PSRSYNC);													// reset prescaler - affects timer 0 too but since it's doing PWM, it's not using the prescaler

	setTimerCeiling(getTimerCeiling(delay));				// set timeout
	setTimerResolution(getTimerResolution(delay));	// restart timer with proper prescaler
}
