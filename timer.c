#include	"timer.h"

/** \file
	\brief Timer management - step pulse clock and system clock

	Teacup uses timer1 to generate both step pulse clock and system clock.

	We achieve this by using the output compare registers to generate the two clocks while the timer free-runs.

	Teacup has tried numerous timer management methods, and this is the best so far.
*/

#ifndef SIMULATOR
#include	<avr/interrupt.h>
#endif
#include	"memory_barrier.h"

#include	"arduino.h"
#include	"config_wrapper.h"

#ifdef	MOTHERBOARD
#include	"dda_queue.h"
#endif

/// time until next step, as output compare register is too small for long step times
uint32_t	next_step_time;

#ifdef ACCELERATION_TEMPORAL
/// unwanted extra delays, ideally always zero
uint32_t	step_extra_time = 0;
#endif /* ACCELERATION_TEMPORAL */

/// every time our clock fires, we increment this so we know when 10ms has elapsed
uint8_t						clock_counter_10ms = 0;
/// keep track of when 250ms has elapsed
uint8_t						clock_counter_250ms = 0;
/// keep track of when 1s has elapsed
uint8_t						clock_counter_1s = 0;

/// flags to tell main loop when above have elapsed
volatile uint8_t	clock_flag_10ms = 0;
volatile uint8_t	clock_flag_250ms = 0;
volatile uint8_t	clock_flag_1s = 0;

#if defined (__AVR__)
/// comparator B is the system clock, happens every TICK_TIME
ISR(TICKER_ISR) {

	// set output compare register to the next clock tick
	OCR1B = (OCR1B + TICK_TIME) & 0xFFFF;

	/*
	clock stuff
	*/
	clock_counter_10ms += TICK_TIME_MS;
	if (clock_counter_10ms >= 10) {
		clock_counter_10ms -= 10;
		clock_flag_10ms = 1;

		clock_counter_250ms++;
		if (clock_counter_250ms >= 25) {
			clock_counter_250ms = 0;
			clock_flag_250ms = 1;

			clock_counter_1s++;
			if (clock_counter_1s >= 4) {
				clock_counter_1s = 0;
				clock_flag_1s = 1;
			}
		}
	}

  dda_clock();
}

#else // assume ARM
// A TICK_TIME PIT seems appropriate

ISR(TICKER_ISR){ // Periodic Interrupt Timer 0
	/*
	clock stuff
	*/
  TICKER_CLEAR();

// Markus: much the same as above.
	clock_counter_10ms += TICK_TIME_MS;
	if (clock_counter_10ms >= 10) {
		clock_counter_10ms -= 10;
		clock_flag_10ms = 1;

		clock_counter_250ms++;
		if (clock_counter_250ms >= 25) {
			clock_counter_250ms = 0;
			clock_flag_250ms = 1;

			clock_counter_1s++;
			if (clock_counter_1s >= 4) {
				clock_counter_1s = 0;
				clock_flag_1s = 1;
			}
		}
	}

  dda_clock();
}
#endif /* __ARMEL__ */


#ifdef	MOTHERBOARD

//process the step timer.
#if defined (__AVR__)
/// comparator A is the step timer. It has higher priority then B.
ISR(TIMER1_COMPA_vect) {
	// Check if this is a real step, or just a next_step_time "overflow"
	if (next_step_time < 65536) {
		// step!
		#ifdef DEBUG_LED_PIN
			WRITE(DEBUG_LED_PIN, 1);
		#endif

		// disable this interrupt. if we set a new timeout, it will be re-enabled when appropriate
		TIMSK1 &= ~MASK(OCIE1A);
		
		// stepper tick
		queue_step();

		// led off
		#ifdef DEBUG_LED_PIN
			WRITE(DEBUG_LED_PIN, 0);
		#endif

		return;
	}

	next_step_time -= 65536;

	// similar algorithm as described in setTimer below.
	if (next_step_time < 65536) {
		OCR1A = (OCR1A + next_step_time) & 0xFFFF;
	} else if(next_step_time < 75536){
		OCR1A = (OCR1A - 10000) & 0xFFFF;
		next_step_time += 10000;
	}
	// leave OCR1A as it was
}
#else // non-AVR, use FTM1, channel 1
ISR(ftm1_isr) {

  TIMER_CLEAR();

  // Check if this is a real step, or just a next_step_time "overflow"
  if (next_step_time < FTM1_MOD) {
    // step!
    #ifdef DEBUG_LED_PIN
      WRITE(DEBUG_LED_PIN, 1);
    #endif

    // disable this interrupt. if we set a new timeout, it will be 
      // re-enabled when appropriate
    // TIMSK1 &= ~MASK(OCIE1A); // AVR
    TIMER_DISABLE();

    // stepper tick
    queue_step();

    // led off
    #ifdef DEBUG_LED_PIN
      WRITE(DEBUG_LED_PIN, 0);
    #endif

    return;
  }
  else { // probably overflow
    //		serial_writestr_P(PSTR("o")); /// drf xxx
    //	  sersendf_P(PSTR("FTM1 ovf: %lu / %u\n"),next_step_time, FTM1_MOD);
  }

  next_step_time -= 65536;

  // similar algorithm as described in setTimer below.
  if (next_step_time < 65536) {
    TIMER_COMPARE_VALUE = (TIMER_COMPARE_VALUE + next_step_time) & 0xFFFF;
  } else if(next_step_time < 75536){
    TIMER_COMPARE_VALUE = (TIMER_COMPARE_VALUE - 10000) & 0xFFFF;
    next_step_time += 10000;
  }
}
#endif // step timer AVR|ARM
#endif /* ifdef MOTHERBOARD */

/// initialise timer and enable system clock interrupt.
/// step interrupt is enabled later when we start using it

void timer_init()
{
#if defined (__AVR__)
	// no outputs
	TCCR1A = 0;
	// Normal Mode
	TCCR1B = MASK(CS10); // no prescaling, 
	// set up "clock" comparator for first tick
	OCR1B = TICK_TIME & 0xFFFF;
	// enable interrupt
	TIMSK1 = MASK(OCIE1B);
  #ifdef SIMULATOR
    // Tell simulator
    sim_timer_init();
  #endif
#else // non-AVR
  // Use two timers on the mk20dx128, FTM1 for steps, and PIT for periodic
  //PIT0:  32 bit timer, clocked at F_BUS, or up to 2^32/F_BUS secs
        //FTM1:  16bit timer, F_CPU/2^(0..7)
  delay(500);
  // TICKER_INIT(); 

  SIM_SCGC6 |= SIM_SCGC6_PIT;PIT_MCR = 0x00;
  // PIT_LDVAL0 = F_BUS * ( TICK_TIME_MS/1000);
  PIT_LDVAL0 = F_BUS * 2./1000;
        PIT_TCTRL0 = TIE|TEN; NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
  //TICKER_ENABLE();

  TIMER_INIT();
  TIMER_ENABLE();

  // sersendf_P(PSTR(";; timer init: FTM1 INIT: %lu / %u\n"),next_step_time, FTM1_MOD);

#endif // timer_init AVR|ARM
}

#ifdef	MOTHERBOARD
/*! Specify how long until the step timer should fire.
	\param delay in CPU ticks

	This enables the step interrupt, but also disables interrupts globally.
	So, if you use it from inside the step interrupt, make sure to do so
	as late as possible. If you use it from outside the step interrupt,
	do a sei() after it to make the interrupt actually fire.
*/
void setTimer(uint32_t delay)
{
	uint16_t step_start = 0;
	#ifdef ACCELERATION_TEMPORAL
	uint16_t current_time;
	uint32_t earliest_time, actual_time;
	#endif /* ACCELERATION_TEMPORAL */

	// An interrupt would make all our timing calculations invalid,
	// so stop that here.
	// called by dda.c:dda_start() and dda_create(), neither do sei() 
        // called by dda_queue.c:queue_step(), next_move(), enqueue_home(), queue_stop()
	// 
	cli();
	CLI_SEI_BUG_MEMORY_BARRIER();

	// Assume all steps belong to one move. Within one move the delay is
	// from one step to the next one, which should be more or less the same
	// as from one step interrupt to the next one. The last step interrupt happend
	// at OCR1A, so start delay from there.
  step_start = TIMER_COMPARE_VALUE;
	next_step_time = delay;

	#ifdef ACCELERATION_TEMPORAL
	// 300 = safe number of cpu cycles until the interrupt actually happens
  current_time = TIMER_COUNTER;
	earliest_time = (uint32_t)current_time + 300;
	if (current_time < step_start) // timer counter did overflow recently
		earliest_time += 0x00010000;
	actual_time = (uint32_t)step_start + next_step_time;

	// Setting the interrupt earlier than it can happen obviously doesn't
	// make sense. To keep the "belongs to one move" idea, add an extra,
	// remember this extra and compensate the extra if a longer delay comes in.
	if (earliest_time > actual_time) {
		step_extra_time += (earliest_time - actual_time);
		next_step_time = earliest_time - (uint32_t)step_start;
	}
	else if (step_extra_time) {
		if (step_extra_time < actual_time - earliest_time) {
			next_step_time -= step_extra_time;
			step_extra_time = 0;
		}
		else {
			step_extra_time -= (actual_time - earliest_time);
			next_step_time -= (actual_time - earliest_time);
		}
	}
	#endif /* ACCELERATION_TEMPORAL */

	// Now we know how long we actually want to delay, so set the timer.
	if (next_step_time < 65536) {
		// set the comparator directly to the next real step
		TIMER_COMPARE_VALUE = (next_step_time + step_start) & 0xFFFF;
	}
	else if (next_step_time < 75536) {
		// Next comparator interrupt would have to trigger another
		// interrupt within a short time (possibly within 1 cycle).
		// Avoid the impossible by firing the interrupt earlier.
    TIMER_COMPARE_VALUE = (step_start - 10000) & 0xFFFF;
		next_step_time += 10000;
	}
	else {
    TIMER_COMPARE_VALUE = step_start;
	}

	// Enable this interrupt, but only do it after disabling
	// global interrupts (see above). This will cause push any possible
	// timer1a interrupt to the far side of the return, protecting the 
	// stack from recursively clobbering memory.
  #if defined (__AVR__)
    TIMSK1 |= MASK(OCIE1A); 
  #elif defined (__ARMEL__)
    TIMER_ENABLE(); // sei();
  #elif defined SIMULATOR
    // Tell simulator
    sim_setTimer();
  #endif
}

/// stop timers - emergency stop
void timer_stop() {
	// disable timing interrupts
  TIMER_DISABLE();
  TICKER_DISABLE();
  #ifdef SIMULATOR
    // Tell simulator
    sim_timer_stop();
  #endif
}
#endif /* ifdef MOTHERBOARD */
