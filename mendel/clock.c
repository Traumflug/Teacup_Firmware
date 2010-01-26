/*
	clock.c

	a system clock with 1ms ticks
*/

#include	"clock.h"

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"arduino.h"

// global clock
#ifdef	GLOBAL_CLOCK
volatile uint32_t	clock = 0;
#endif

// 1/4 second tick
uint8_t						clock_counter_250ms = 0;
volatile uint8_t	clock_flag_250ms = 0;

void clock_setup() {
	// use system clock
	ASSR = 0;

	// no compare match, CTC mode
	TCCR2A = MASK(WGM21);

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
		clock_flag_250ms = 255;
		clock_counter_250ms = 0;
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
