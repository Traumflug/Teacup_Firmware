#include	"home.h"

/** \file
	\brief Interrupt driven homing routines
*/

// 20110814 modmaker - First release, some comment on this code:
//
// - Features:
//			= Because of interrupt driven step pulses, the homing
//			  movement is smooth now.
//			= Automatic timer prescaler selection covers a wide range
//			  of stepping speeds (as low as 61 Hz on a 16 MHz mega1280).
//			= Limits the movement to the axis' length if a switch does not work.
//			= Uses the free 8-bit timer Timer0.
//			= Also moves to position 0 after seeking a MIN switch instead of
//			  doing that only for the MAX switches.
//
// - Cons:
//			= Code size increases, does not fit in an ATmega168 anymore.
//
// - Limitations:
//			= This code was only tested on an Arduino Mega (ATmega1280).
//			= Only the axis 'MIN' switches were tested.
//

#include	<avr/interrupt.h>

#include	"dda.h"
#include	"dda_queue.h"
#include	"delay.h"
#include	"pinio.h"
#ifdef DEBUG
#include	"sersendf.h"
//#undef DEBUG
#endif
#include	"memory_barrier.h"


// Sanity checks for config.h settings:
#if !defined( X_DIR_PIN) || !defined( Y_DIR_PIN) || !defined( Z_DIR_PIN)
#error "Need direction signals for homing operation"
#endif

// NOTE:
//		If an axis' switch isn't activated after the steps needed to traverse that entire axis,
//		either the hardware failed, or we started moving at the wrong side of the switch!

// Create identifiers for different homing operations, 6 axis will fit an uint8_t,
// use bit positions so we can use them in a bitmap.
typedef enum {
	home_x_min = (1 << 0),
	home_x_max = (1 << 1),
	home_y_min = (1 << 2),
	home_y_max = (1 << 3),
	home_z_min = (1 << 4),
	home_z_max = (1 << 5)
} homing_t;

// These values are used by the ISR and write only for the application.
// Don't change these while the timer is generating interrupts!
static uint8_t 	timer_interval;
static uint8_t 	end_timer_interval;
static uint8_t 	running_axis;
static uint16_t pulse_limit;
static uint8_t 	limit_switch_state;  // set to 1 to run to switch, 0 to run from the switch

/// timer0 comparator B is used to generate the step pulse active period
ISR(TIMER0_COMPB_vect)
{
	// NOTE: This code is written to prevent promotion of the uint8_t to integers,
	// as that generates a lot of unnecessary overhead!
	if (running_axis == home_x_min || running_axis == home_x_max) {
		_x_step( 0);
	} else if (running_axis == home_y_min || running_axis == home_y_max) {
		_y_step( 0);
	} else if (running_axis == home_z_min || running_axis == home_z_max) {
		_z_step( 0);
	}
//	TIMSK0 &= ~MASK( OCIE0B);				// disable compare mask B interrupt
}

/// timer0 comparator A is used to generate the step cycle time
ISR(TIMER0_COMPA_vect)
{
	static uint8_t debounce_count = 0;
	static uint8_t pulse_limit_prescaler = 0;	// 1:256 prescaler
	uint8_t timestamp = OCR0A;
	uint8_t switch_state = 255;	// init to illegal state (to keep compiler happy)
	// determine applicable limit switch state
	// NOTE: This code is written to prevent promotion of the uint8_t to integers,
	// as that generates a lot of unnecessary overhead!
	if (running_axis == home_x_min) {
		switch_state = x_min();
	} else if (running_axis == home_y_min) {
		switch_state = y_min();
	} else if (running_axis == home_z_min) {
		switch_state = z_min();
	} else if (running_axis == home_x_max) {
		switch_state = x_max();
	} else if (running_axis == home_y_max) {
		switch_state = y_max();
	} else if (running_axis == home_z_max) {
		switch_state = z_max();
	}
	// if limit switch has been permanently active for some time, stop generating
	// step pulses and signal the application we've reached the home switch.
	if (switch_state == limit_switch_state) {
		++debounce_count;
	} else {
		debounce_count = 0;
	}
	if (--pulse_limit_prescaler == 0) {
		--pulse_limit;
	}
	if (debounce_count == 8 || pulse_limit == 0) {
		TIMSK0 = 0;		// disable all timer0 interrupts, used to signal completion
	} else {
		// NOTE: This code is written to prevent promotion of the uint8_t to integers,
		// as that generates a lot of unnecessary overhead!
		if (running_axis == home_x_min || running_axis == home_x_max) {
			_x_step( 1);
		} else if (running_axis == home_y_min || running_axis == home_y_max) {
			_y_step( 1);
		} else if (running_axis == home_z_min || running_axis == home_z_max) {
			_z_step( 1);
		}
		// set output compare A interrupt to start the next step pulse
		if (timer_interval > end_timer_interval && (pulse_limit_prescaler & 31) == 0) {
			timer_interval -= 1;
		}
		OCR0A = (timestamp + timer_interval) & 0xFF;
		// set output compare B interrupt to end the current step pulse (approx. 50% D.C.)
		OCR0B = (timestamp + (timer_interval / 2)) & 0xFF;
		TIMSK0 |= MASK( OCIE0B);			// enable compare mask B interrupt
	}
}

// Use timer0 to generate stepping pulses for homing operations.
// Run until the appropriate limit switch gets activated or released, as selected
// by switch_state.
static void step_axis_until_switch( uint8_t axis, uint16_t start_step_period, uint16_t end_step_period, uint8_t switch_state)
{
#ifdef DEBUG
	sersendf_P( PSTR( "step_axis_until_switch( axis = %u, step_period = [%u -> %u] switch_state = %u)\n"), axis, start_step_period, end_step_period, switch_state);
#endif

	// determine the minimum prescaler needed (to run with maximum resolution)
	// take the prescaler on the safe side (larger than needed)
	uint16_t 	min_prescale 	= 1 + start_step_period / (256 * 1000000 / F_CPU);
	uint8_t		prescaler_shift	= 0;
	uint8_t		prescaler_mask	= 0;

	if (min_prescale >= 256) {
		if (min_prescale >= 1024) {
			// can't step this slow, clip values below threshold
			start_step_period = (uint16_t) ((1000000.0 * 255 * 1024) / F_CPU);
			if (end_step_period > start_step_period) {
				end_step_period = start_step_period;
			}
		}
		// use clk/1024
		prescaler_shift	= 10;
		prescaler_mask 	= MASK( CS00) |               MASK( CS02);
	} else if (min_prescale >= 64) {
		// use clk/256
		prescaler_shift	= 8;
		prescaler_mask 	=                             MASK( CS02);
	} else if (min_prescale >= 8) {
		// use clk/64
		prescaler_shift	= 6;
		prescaler_mask 	= MASK( CS00) | MASK( CS01);
	} else if (min_prescale >= 1) {
		// use clk/8
		prescaler_shift	= 3;
		prescaler_mask 	=               MASK( CS01);
	} else {
		// use clk/1
		prescaler_shift	= 1;
		prescaler_mask 	= MASK( CS00);
	}

#ifdef DEBUG
	sersendf_P( PSTR( "min. prescaler = %u => _shift = %u, _mask = %u\n"), min_prescale, prescaler_shift, prescaler_mask);
#endif

//	original formula: timer_interval [-] = 1 + period [us] / (1000000 [us/s] * prescaler [-] / F_CPU [1/s]);
	timer_interval     = 1 + (((F_CPU / 1000000) * start_step_period) >> prescaler_shift);
	end_timer_interval = 1 + (((F_CPU / 1000000) * end_step_period  ) >> prescaler_shift);

	// set expected signal on home switch
	limit_switch_state = switch_state;		// used by ISR

#ifdef DEBUG
	sersendf_P( PSTR( "timer_interval = [%u -> %u], pulse_limit = %lu\n"), timer_interval, end_timer_interval, ((uint32_t)pulse_limit << 8));
#endif

	running_axis = axis;	// for the ISR to use

	PRR0 &= ~MASK( PRTIM0);					// disable powerdown of timer0 
	TCCR0A = 0;
	TCCR0B = prescaler_mask;				// set prescaler
	GTCCR  = MASK( PSRASY);					// needed?
	ASSR = 0;
	TCNT0 = 0;
	OCR0A = timer_interval;
	TIFR0 = MASK( OCF0A) | MASK( OCF0B);	// clear any pending interrupt(s)
	// Several variables used by the ISR aren't declared volatile to allow the compiler to
	// optimize the code. As a result of this, a variable may be kept in a processor register
	// until the need arises (if ever) to write it to memory.
	// With the barrier we enforce the compiler to update the memory image, so that
	// the ISR has access to these variables.
	// We do this just before the ISR will access them.
	MEMORY_BARRIER();
	TIMSK0 = MASK( OCIE0A);					// enable compare mask A interrupt
	
	do {
		delay_ms( 1);
	} while (TIMSK0 & MASK( OCIE0A));
}

static void axis_enable( uint8_t axis)
{
	switch (axis) {
	case home_x_min:
	case home_x_max:
		x_enable();
		break;
	case home_y_min:
	case home_y_max:
		y_enable();
		break;
	case home_z_min:
	case home_z_max:
		z_enable();
		break;
	}
}

static void axis_direction( uint8_t axis, uint8_t from_limit)
{
	switch (axis) {
	case home_x_min:
		x_direction( ((from_limit) ? 1 : 0));
		break;
	case home_x_max:
		x_direction( ((from_limit) ? 0 : 1));
		break;
	case home_y_min:
		y_direction( ((from_limit) ? 1 : 0));
		break;
	case home_y_max:
		y_direction( ((from_limit) ? 0 : 1));
		break;
	case home_z_min:
		z_direction( ((from_limit) ? 1 : 0));
		break;
	case home_z_max:
		z_direction( ((from_limit) ? 0 : 1));
		break;
	}
}

// Execute the actual homing operation. The hardware selected with the 'axis'
// variable must exist or we'll fail miserably, so filter before calling here!
static void run_home_one_axis( uint8_t axis, uint32_t feed)
{
	uint32_t 	fast_step_period 		=  75;	// init to keep compiler happy
	uint32_t 	slow_step_period 		= 250;	// init to keep compiler happy
	uint32_t	max_pulses_on_axis 		=   0;	// init to keep compiler happy
	uint32_t	max_pulses_for_release 	=   0;	// init to keep compiler happy
	uint8_t		limit_switch_state 		=   0;	// init to keep compiler happy

	// TODO: handle undefined _MIN and _MAX values!

	if (axis == home_x_min || axis == home_x_max) {
#if 1
		if (feed > MAXIMUM_FEEDRATE_X) {
			feed = MAXIMUM_FEEDRATE_X;			
		}
		fast_step_period		= (uint32_t) 1 + (60000000L / STEPS_PER_MM_X) / feed;
#else
		fast_step_period 		= (uint32_t) HOME_FAST_STEP_PERIOD_X;
#endif
		slow_step_period 		= (uint32_t) HOME_SLOW_STEP_PERIOD_X;
		limit_switch_state 		= (axis & home_x_max) ? x_max() : x_min();
		max_pulses_on_axis 		= (uint32_t)((X_MAX - X_MIN) * STEPS_PER_MM_X);
		max_pulses_for_release 	= (uint32_t)(RELEASE_DISTANCE * STEPS_PER_MM_X);
	} else if (axis == home_y_min || axis == home_y_max) {
#if 1
		if (feed > MAXIMUM_FEEDRATE_Y) {
			feed = MAXIMUM_FEEDRATE_Y;			
		}
		fast_step_period		= (uint32_t) 1 + (60000000L / STEPS_PER_MM_Y) / feed;
#else
		fast_step_period 		= (uint32_t) HOME_FAST_STEP_PERIOD_Y;
#endif
		slow_step_period 		= (uint32_t) HOME_SLOW_STEP_PERIOD_Y;
		limit_switch_state 		= (axis & home_y_max) ? y_max() : y_min();
		max_pulses_on_axis 		= (uint32_t)((Y_MAX - Y_MIN) * STEPS_PER_MM_Y);
		max_pulses_for_release 	= (uint32_t)(RELEASE_DISTANCE * STEPS_PER_MM_Y);
	} else if (axis == home_z_min || axis == home_z_max) {
#if 1
		if (feed > MAXIMUM_FEEDRATE_Z) {
			feed = MAXIMUM_FEEDRATE_Z;			
		}
		fast_step_period		= (uint32_t) 1 + (60000000L / STEPS_PER_MM_Z) / feed;
#else
		fast_step_period 		= (uint32_t) HOME_FAST_STEP_PERIOD_Z;
#endif
		slow_step_period 		= (uint32_t) HOME_SLOW_STEP_PERIOD_Z;
		limit_switch_state 		= (axis & home_z_max) ? z_max() : z_min();
		max_pulses_on_axis 		= (uint32_t)((Z_MAX - Z_MIN) * STEPS_PER_MM_Z);
		max_pulses_for_release 	= (uint32_t)(RELEASE_DISTANCE * STEPS_PER_MM_Z);
	}
	// 12.5% extra for inaccurate _MAX and _MIN settings
	max_pulses_on_axis 			= (9 * max_pulses_on_axis) / 8;
	// prevent unpredictable behaviour caused by bit loss from too large values
	// make the clipped values recognizable in the debug output
	if (fast_step_period & 0xffff0000) {
		fast_step_period = 65000;
	}
	if (slow_step_period & 0xffff0000) {
		slow_step_period = 65000;
	}
	if (max_pulses_on_axis & 0xff000000) {
		max_pulses_on_axis = 16000000;
	}
	// enable stepper and set proper direction
	axis_enable( axis);
	// Fast move towards the switch, but skip this action if the switch is already activated.
	if (limit_switch_state == 0) {
		axis_direction( axis, 0 /* move towards the switch */);
		// limit number of pulses to physical range with a small
		// offset for truncation and prescaler implementation.
		pulse_limit = 2 + (max_pulses_on_axis >> 8);
		// hit home hard
		step_axis_until_switch( axis, 2 * fast_step_period, fast_step_period, 1 /* until switch is activated */);
	}
	// Allow mechanics to stabilize
	delay_ms( 500);
	// Slowly move in opposite direction until the switch is released
	axis_direction( axis, 1 	/* move away from switch */);
	// limit number of pulses
	pulse_limit = 2 + (max_pulses_for_release >> 8);
	// back off slowly
	step_axis_until_switch( axis, slow_step_period, slow_step_period, 0 /* until switch is released */);
}

/// home the selected axis to the selected limit switch.
// keep all preprocessor configuration stuff at or below this level.
static void home_one_axis( uint8_t axis, uint32_t feed) {

	// get ready for the action
	power_on();
	queue_wait();

	// move to a limit switch or sensor
	run_home_one_axis( axis, feed);

}

/// find X MIN endstop
void home_x_negative( uint32_t feed) {
	#if defined X_MIN_PIN
		home_one_axis( home_x_min, feed);
	#endif
	// reference 'home' position to current position
	#ifdef X_MIN
		startpoint.X =
			current_position.X = (int32_t) (X_MIN * STEPS_PER_MM_X);
	#else
		startpoint.X =
			current_position.X = 0;
	#endif
}

/// find X_MAX endstop
void home_x_positive( uint32_t feed) {
	#if defined X_MAX_PIN
		home_one_axis( home_x_max, feed);
	#endif
	// reference 'home' position to current position
	#ifdef X_MAX
		startpoint.X =
			current_position.X = (int32_t) (X_MAX * STEPS_PER_MM_X);
	#else
		startpoint.X =
			current_position.X = 0;
	#endif
}

/// find Y MIN endstop
void home_y_negative( uint32_t feed) {
	#if defined Y_MIN_PIN
		home_one_axis( home_y_min, feed);
	#endif
	// reference 'home' position to current position
	#ifdef Y_MIN
		startpoint.Y =
			current_position.Y = (int32_t) (Y_MIN * STEPS_PER_MM_Y);
	#else
		startpoint.Y =
			current_position.Y = 0;
	#endif
}

/// find Y MAX endstop
void home_y_positive( uint32_t feed) {
	#if defined Y_MAX_PIN
		home_one_axis( home_y_max, feed);
	#endif
	// reference 'home' position to current position
	#ifdef Y_MAX
		startpoint.Y =
			current_position.Y = (int32_t) (Y_MAX * STEPS_PER_MM_Y);
	#else
		startpoint.Y =
			current_position.Y = 0;
	#endif
}

/// find Z MIN endstop
void home_z_negative( uint32_t feed) {
	#if defined Z_MIN_PIN
		home_one_axis( home_z_min, feed);
	#endif
	// reference 'home' position to current position
	#ifdef Z_MIN
		startpoint.Z =
			current_position.Z = (int32_t) (Z_MIN * STEPS_PER_MM_Z);
	#else
		startpoint.Z =
			current_position.Z = 0;
	#endif
}

/// find Z MAX endstop
void home_z_positive( uint32_t feed) {
	#if defined Z_MAX_PIN
		home_one_axis( home_z_max, feed);
	#endif
	// reference 'home' position to current position
	#ifdef Z_MAX
		startpoint.Z =
			current_position.Z = (int32_t) (Z_MAX * STEPS_PER_MM_Z);
	#else
		startpoint.Z =
			current_position.Z = 0;
	#endif
}


