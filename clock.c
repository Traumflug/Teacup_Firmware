/*
	clock.c

	a system clock with 1ms ticks
*/

#include	"clock.h"

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"config.h"

// global clock
#ifdef	GLOBAL_CLOCK
volatile uint32_t	clock = 0;
#endif

// 1/4 second tick
uint8_t						clock_counter_250ms = 0;
uint8_t						clock_counter_1s = 0;
volatile uint8_t	clock_flag = 0;

void clock_setup() {
	// use system clock
	ASSR = 0;

	// no compare match, CTC mode
	TCCR2A = MASK(WGM21);
	// TODO: Timer 2 has higher priority than Timer 1 used for the stepper
	//       interrupts, which is bad. See AVR Reference Manual p. 9:
	//          "The interrupts have priority in accordance
	//           with their Interrupt Vector position. The
	//           lower the Interrupt Vector address, the higher
	//           the priority."
	//       in conjunction with p. 63 (interrupt vector table).

	// 128 prescaler (16MHz / 128 = 125KHz)
	TCCR2B = MASK(CS22) | MASK(CS20);

	// 125KHz / 125 = 1KHz for a 1ms tick rate
	OCR2A = 125;

	// interrupt on overflow, when counter reaches OCR2A
	TIMSK2 |= MASK(OCIE2A);
}

ISR(TIMER2_COMPA_vect) {
	// global clock
#ifdef	GLOBAL_CLOCK
	clock++;
#endif
	// 1/4 second tick
	if (++clock_counter_250ms == 250) {
		clock_flag |= CLOCK_FLAG_250MS;
		clock_counter_250ms = 0;
		if (++clock_counter_1s == 4) {
			clock_flag |= CLOCK_FLAG_1S;
			clock_counter_1s = 0;
		}
	}
}

#ifdef	GLOBAL_CLOCK
uint32_t clock_read() {
	uint32_t	c;

	cli();			// set atomic
	c = clock;	// copy clock value
	sei();			// release atomic

	return c;
}
#endif
