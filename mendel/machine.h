#ifndef	_MACHINE_H
#define	_MACHINE_H

/*
	move buffer size, in number of moves
		note that each move takes a fair chunk of ram (69 bytes as of this writing) so don't make the buffer too big - a bigger serial readbuffer may help more than increasing this unless your gcodes are more than 70 characters long on average.
		however, a larger movebuffer will probably help with lots of short consecutive moves, as each move takes a bunch of math (hence time) to set up
*/
#define	MOVEBUFFER_SIZE	8

/*
	axis calculations, adjust as necessary
*/

// XY can have lots of precision and still move speedily, so microstepping is helpful
#define	X_STEPS_PER_REV						3200.0
#define	Y_STEPS_PER_REV						X_STEPS_PER_REV

// we need far more speed than precision on Z due to the threaded rod drive, maybe just half stepping
#define	Z_STEPS_PER_REV						400.0

#define	X_COG_CIRCUMFERENCE				(4.77 * 16.0)
#define	Y_COG_CIRCUMFERENCE				X_COG_CIRCUMFERENCE
// also try:
// #define	XY_COG_RADIUS			9.5
// #define	XY_COG_CIRCUMFERENCE	(XY_COG_RADIUS * PI * 2)

// this is the ratio between number of teeth on the Z motor gear, and teeth on the Z leadscrew base gear.
#define	Z_GEAR_RATIO							1.0

// we need more torque and smoothness at very low speeds on E, maximum microstepping
#define	E_STEPS_PER_REV						3200.0
#define	EXTRUDER_SHAFT_RADIUS			5.0
#define	EXTRUDER_INLET_DIAMETER		3.0
#define	EXTRUDER_NOZZLE_DIAMETER	0.8

// these feedrates are used during homing and G0 rapid moves
#define	FEEDRATE_FAST_XY					6000
#define	FEEDRATE_SLOW_XY					300

#define	FEEDRATE_FAST_Z						6000
#define	FEEDRATE_SLOW_Z						300

#define	FEEDRATE_FAST_E						1200

// this is how many steps to suck back the filament by when we stop
#define	E_STARTSTOP_STEPS					20

// extruder settings
#define	TEMP_HYSTERESIS						20
#define	TEMP_RESIDENCY_TIME				60

/*
	calculated values - you shouldn't need to touch these
	however feel free to put in your own values if they can be more precise than the calculated approximations, remembering that they must end up being integers- floating point by preprocessor only thanks!
*/

#define	STEPS_PER_MM_X						((uint32_t) ((X_STEPS_PER_REV / X_COG_CIRCUMFERENCE) + 0.5))
#define	STEPS_PER_MM_Y						((uint32_t) ((Y_STEPS_PER_REV / Y_COG_CIRCUMFERENCE) + 0.5))
#define	STEPS_PER_MM_Z						((uint32_t) ((Z_STEPS_PER_REV * Z_GEAR_RATIO) + 0.5))

// http://blog.arcol.hu/?p=157 may help with this next one
// I haven't tuned this at all- it's just a placeholder until I read the above carefully enough
// does this refer to filament or extrudate? extrudate depends on XY distance vs E distance.. hm lets go with filament
// #define	STEPS_PER_MM_E						((uint32_t) ((E_STEPS_PER_REV / (EXTRUDER_SHAFT_RADIUS * PI * EXTRUDER_INLET_DIAMETER / EXTRUDER_NOZZLE_DIAMETER)) + 0.5))

#define	STEPS_PER_MM_E						((uint32_t) ((E_STEPS_PER_REV * EXTRUDER_NOZZLE_DIAMETER / (EXTRUDER_SHAFT_RADIUS * PI * EXTRUDER_INLET_DIAMETER)) + 0.5))

// same as above with 25.4 scale factor
#define	STEPS_PER_IN_X						((uint32_t) ((25.4 * X_STEPS_PER_REV / X_COG_CIRCUMFERENCE) + 0.5))
#define	STEPS_PER_IN_Y						((uint32_t) ((25.4 * Y_STEPS_PER_REV / Y_COG_CIRCUMFERENCE) + 0.5))
#define	STEPS_PER_IN_Z						((uint32_t) ((25.4 * Z_STEPS_PER_REV * Z_GEAR_RATIO) + 0.5))
#define	STEPS_PER_IN_E						((uint32_t) ((25.4 * E_STEPS_PER_REV * EXTRUDER_NOZZLE_DIAMETER / (EXTRUDER_SHAFT_RADIUS * PI * EXTRUDER_INLET_DIAMETER)) + 0.5))

// inverse, used in distance calculation during DDA setup
#define	UM_PER_STEP_X			((uint32_t) ((1000.0 / STEPS_PER_MM_X) + 0.5))
#define	UM_PER_STEP_Y			((uint32_t) ((1000.0 / STEPS_PER_MM_Y) + 0.5))
#define	UM_PER_STEP_Z			((uint32_t) ((1000.0 / STEPS_PER_MM_Z) + 0.5))
#define	UM_PER_STEP_E			((uint32_t) ((1000.0 / STEPS_PER_MM_E) + 0.5))

// should be the same for all machines! ;)
#define	PI	3.1415926535

/*
	firmware build options
*/

// this option makes the step interrupt interruptible.
// this should help immensely with dropped serial characters, but may also make debugging infuriating due to the complexities arising from nested interrupts
#define		STEP_INTERRUPT_INTERRUPTIBLE	1

// Xon/Xoff flow control. Should be redundant
// #define	XONXOFF

#endif	/* _MACHINE_H */
