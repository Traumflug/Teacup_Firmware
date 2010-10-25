#include	"timer.h"

#include	<avr/interrupt.h>

#include	"dda_queue.h"

volatile uint32_t	next_step_time;

uint8_t						clock_counter_250ms = 0;
uint8_t						clock_counter_1s = 0;
volatile uint8_t	clock_flag = 0;

// how often we overflow and update our clock; with F_CPU=16MHz, max is < 4.096ms (TICK_TIME = 65535)
#define		TICK_TIME		2 MS

// timer overflow, happens every TICK_TIME
ISR(TIMER1_CAPT_vect) {
	/*
	check if next step time will occur before next overflow
	*/
	if (next_step_time > TICK_TIME)
		next_step_time -= TICK_TIME;
	else {
		if (next_step_time > 0) {
			OCR1A = next_step_time & 0xFFFF;
			TIMSK1 |= MASK(OCIE1A);
		}
	}
	
	/*
	clock stuff
	*/
	clock_counter_250ms += (TICK_TIME / (F_CPU / 1000));
	if (clock_counter_250ms >= 250) {
		clock_counter_250ms -= 250;
		clock_flag |= CLOCK_FLAG_250MS;
		clock_counter_1s += 1;
		if (clock_counter_1s >= 4) {
			clock_counter_1s -= 4;
			clock_flag |= CLOCK_FLAG_1S;
		}
	}
}

ISR(TIMER1_COMPA_vect) {
	// led on
	WRITE(SCK, 1);
	
	// disable this interrupt. if we set a new timeout, it will be re-enabled when appropriate
	TIMSK1 &= ~MASK(OCIE1A);
	
	// ensure we don't interrupt again unless timer is reset
	next_step_time = 0;
	
	/*
	stepper tick
	*/
	queue_step();
	
	// led off
	WRITE(SCK, 0);
}

void timer_init()
{
	// no outputs
	TCCR1A = 0;
	// CTC mode- use ICR for top
	TCCR1B = MASK(WGM13) | MASK(WGM12) | MASK(CS10);
	// set timeout- first timeout is indeterminate, probably doesn't matter
	ICR1 = TICK_TIME;
	// overflow interrupt (uses input capture interrupt in CTC:ICR mode)
	TIMSK1 = MASK(ICIE1);
}

void setTimer(uint32_t delay)
{
	// save interrupt flag
	uint8_t sreg = SREG;
	// disable interrupts
	cli();

	// re-enable clock interrupt in case we're recovering from emergency stop
	TIMSK1 |= MASK(ICIE1);
	
	if (delay > 0) {
		// mangle timer variables
		next_step_time = delay + TCNT1;
		if (delay <= 16) {
			// force interrupt
			// TODO: datasheet says force only doesn't work in CTC:COMPA mode, test if CTC:ICR mode allows force
			TIMSK1 |= MASK(OCIE1A);
			TCCR1C |= MASK(FOC1A);
			next_step_time = 0;
		}
		else if (delay <= TICK_TIME) {
			OCR1A = next_step_time & 0xFFFF;
			TIMSK1 |= MASK(OCIE1A);
		}
	}
	else {
		next_step_time = 0;
	}
	
	// restore interrupt flag
	SREG = sreg;
}

void timer_stop() {
	// disable all interrupts
	TIMSK1 = 0;
	// reset timeout
	next_step_time = 0;
}
