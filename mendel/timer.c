#include	"timer.h"

#include	<avr/interrupt.h>

#include	"pinout.h"
#include	"dda.h"

ISR(TIMER1_COMPA_vect) {
// 	static interruptBlink = 0;
//
// 	interruptBlink++;
// 	if (interruptBlink == 0x80) {
// 		blink();
// 		interruptBlink = 0;
// 	}

	if(movebuffer[mb_tail].live)
		dda_step(&movebuffer[mb_tail]);
	else
		next_move();
}

void setupTimerInterrupt()
{
	//clear the registers
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	TIMSK1 = 0;

	//waveform generation = 0100 = CTC
	TCCR1B &= ~(1<<WGM13);
	TCCR1B |=  (1<<WGM12);
	TCCR1A &= ~(1<<WGM11);
	TCCR1A &= ~(1<<WGM10);

	//output mode = 00 (disconnected)
	TCCR1A &= ~(1<<COM1A1);
	TCCR1A &= ~(1<<COM1A0);
	TCCR1A &= ~(1<<COM1B1);
	TCCR1A &= ~(1<<COM1B0);

	//start off with a slow frequency.
	setTimerResolution(4);
	setTimerCeiling(65535);
}

uint8_t getTimerResolution(const uint32_t delay)
{
	// these also represent frequency: 1000000 / delay / 2 = frequency in hz.

	// our slowest speed at our highest resolution ( (2^16-1) * 0.0625 usecs = 4095 usecs (4 millisecond max))
	// range: 8Mhz max - 122hz min
	if (delay <= 65535L)
		return 0;
	// our slowest speed at our next highest resolution ( (2^16-1) * 0.5 usecs = 32767 usecs (32 millisecond max))
	// range:1Mhz max - 15.26hz min
	else if (delay <= 524280L)
		return 1;
	// our slowest speed at our medium resolution ( (2^16-1) * 4 usecs = 262140 usecs (0.26 seconds max))
	// range: 125Khz max - 1.9hz min
	else if (delay <= 4194240L)
		return 2;
	// our slowest speed at our medium-low resolution ( (2^16-1) * 16 usecs = 1048560 usecs (1.04 seconds max))
	// range: 31.25Khz max - 0.475hz min
	else if (delay <= 16776960L)
		return 3;
	// our slowest speed at our lowest resolution ((2^16-1) * 64 usecs = 4194240 usecs (4.19 seconds max))
	// range: 7.812Khz max - 0.119hz min
// 	else if (delay <= 67107840L)
// 		return 4;
	//its really slow... hopefully we can just get by with super slow.
// 	else
	return 4;
}

void setTimerResolution(uint8_t r)
{
	//here's how you figure out the tick size:
	// 1000000 / ((F_CPU / prescaler))
	// 1000000 = microseconds in 1 second
	// prescaler = your prescaler

	// assuming CS10,CS11,CS12 are adjacent bits in platform endian order,
	TCCR1B = (TCCR1B & ~(MASK(CS10) | MASK(CS11) | MASK(CS12))) | ((r + 1) << CS10);
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

void delay(uint32_t delay) {
	while (delay > 65535) {
		delayMicrosecondsInterruptible(delay);
		delay -= 65535;
	}
	delayMicrosecondsInterruptible(delay & 0xFFFF);
}

// from reprap project 5D firmware
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
