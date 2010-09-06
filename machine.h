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

// --------------------------------------------------------------------------
// you shouldn't need to edit something below this line

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


/*
	firmware build options
*/

// this option makes the step interrupt interruptible.
// this should help immensely with dropped serial characters, but may also make debugging infuriating due to the complexities arising from nested interrupts
#define		STEP_INTERRUPT_INTERRUPTIBLE	1

// Xon/Xoff flow control. Redundant when using RepRap Host for sending GCode,
// but mandatory when sending GCode files with a plain terminal emulator,
// like GtkTerm (Linux), CoolTerm (Mac) or HyperTerminal (Windows).
// #define	XONXOFF

/*
	move buffer size, in number of moves
		note that each move takes a fair chunk of ram (69 bytes as of this writing) so don't make the buffer too big - a bigger serial readbuffer may help more than increasing this unless your gcodes are more than 70 characters long on average.
		however, a larger movebuffer will probably help with lots of short consecutive moves, as each move takes a bunch of math (hence time) to set up
*/
#define	MOVEBUFFER_SIZE	8

/*
*/
#define	REFERENCE	REFERENCE_AREF

#endif	/* _MACHINE_H */
