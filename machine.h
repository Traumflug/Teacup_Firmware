#ifndef	_MACHINE_H
#define	_MACHINE_H

// --------------------------------------------------------------------------
// values reflecting the gearing of your machine
// all numbers are integers, so no decimals, please :-)

// calculate these values appropriate for your machine
#define	STEPS_PER_MM_X		320
#define	STEPS_PER_MM_Y		320
#define	STEPS_PER_MM_Z		320

// http://blog.arcol.hu/?p=157 may help with this next one
#define	STEPS_PER_MM_E		320

// this is how many steps to suck back the filament by when we stop
#define	E_STARTSTOP_STEPS	20

// --------------------------------------------------------------------------
// values depending on the capabilities of your stepper motors and other mechanics
// again, all numbers are integers

// used for G0 rapid moves and as a cap for all other feedrates
#define	MAXIMUM_FEEDRATE_X	200
#define	MAXIMUM_FEEDRATE_Y	200
#define	MAXIMUM_FEEDRATE_Z	200
#define	MAXIMUM_FEEDRATE_E	200

// used when searching endstops and similar
#define	SEARCH_FEEDRATE_X	50
#define	SEARCH_FEEDRATE_Y	50
#define	SEARCH_FEEDRATE_Z	50
#define	SEARCH_FEEDRATE_E	50

// extruder settings
#define	TEMP_HYSTERESIS		20
#define	TEMP_RESIDENCY_TIME	60

/*
	acceleration, reprap style.
		Each movement starts at the speed of the previous command and accelerates or decelerates linearly to reach target speed at the end of the movement.
		Can also be set in Makefile
*/
#ifndef	ACCELERATION_RAMPING
#define ACCELERATION_REPRAP
#endif

/*
	acceleration and deceleration ramping.
		Each movement starts at (almost) no speed, linearly accelerates to target speed and decelerates just in time to smoothly stop at the target. alternative to ACCELERATION_REPRAP
		Can also be set in Makefile
*/
//#define ACCELERATION_RAMPING

// how fast to accelerate when using ACCELERATION_RAMPING
// smaller values give quicker acceleration
// valid range = 1 to 8,000,000; 500,000 is a good starting point
#define ACCELERATION_STEEPNESS	500000

#ifdef ACCELERATION_REPRAP
	#ifdef ACCELERATION_RAMPING
		#error Cant use ACCELERATION_REPRAP and ACCELERATION_RAMPING together.
	#endif
#endif

// which temperature sensor are you using?
// #define	TEMP_MAX6675
#define	TEMP_THERMISTOR
// #define	TEMP_AD595

// if you selected thermistor or AD595, what pin is it on?
#define	TEMP_PIN_CHANNEL	AIO0_PIN
#define	ANALOG_MASK				MASK(TEMP_PIN_CHANNEL)

/*
firmware build options
*/

// this option makes the step interrupt interruptible.
// this should help immensely with dropped serial characters, but may also make debugging infuriating due to the complexities arising from nested interrupts
#define		STEP_INTERRUPT_INTERRUPTIBLE	1

/*
	Xon/Xoff flow control.
		Redundant when using RepRap Host for sending GCode, but mandatory when sending GCode files with a plain terminal emulator, like GtkTerm (Linux), CoolTerm (Mac) or HyperTerminal (Windows).
		Can also be set in Makefile
*/
// #define	XONXOFF

/*
	move buffer size, in number of moves
		note that each move takes a fair chunk of ram (69 bytes as of this writing) so don't make the buffer too big - a bigger serial readbuffer may help more than increasing this unless your gcodes are more than 70 characters long on average.
		however, a larger movebuffer will probably help with lots of short consecutive moves, as each move takes a bunch of math (hence time) to set up so a longer buffer allows more of the math to be done during preceding longer moves
*/
#define	MOVEBUFFER_SIZE	8

/*
	analog subsystem stuff
	REFERENCE - which analog reference to use. see analog.h for choices
	ANALOG_MASK - which analog inputs we will be using, bitmask. eg; #define ANALOG_MASK	MASK(AIO0_PIN) | MASK(3) for AIN0 and AIN3
*/
#define	REFERENCE			REFERENCE_AREF
#ifndef	ANALOG_MASK
#define	ANALOG_MASK		0
#endif

// --------------------------------------------------------------------------
// you shouldn't need to edit anything below this line

// same as above with 25.4 scale factor
#define	STEPS_PER_IN_X		((uint32_t) ((25.4 * STEPS_PER_MM_X) + 0.5))
#define	STEPS_PER_IN_Y		((uint32_t) ((25.4 * STEPS_PER_MM_Y) + 0.5))
#define	STEPS_PER_IN_Z		((uint32_t) ((25.4 * STEPS_PER_MM_Z) + 0.5))
#define	STEPS_PER_IN_E		((uint32_t) ((25.4 * STEPS_PER_MM_E) + 0.5))

// inverse, used in distance calculation during DDA setup
#define	UM_PER_STEP_X		((uint32_t) ((1000.0 / STEPS_PER_MM_X) + 0.5))
#define	UM_PER_STEP_Y		((uint32_t) ((1000.0 / STEPS_PER_MM_Y) + 0.5))
#define	UM_PER_STEP_Z		((uint32_t) ((1000.0 / STEPS_PER_MM_Z) + 0.5))
#define	UM_PER_STEP_E		((uint32_t) ((1000.0 / STEPS_PER_MM_E) + 0.5))

#endif	/* _MACHINE_H */
