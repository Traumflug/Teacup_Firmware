#ifndef	_MACHINE_H
#define	_MACHINE_H

/*
	move buffer size, in number of moves
		note that each move takes a fair chunk of ram (71 bytes as of this writing) so don't make the buffer too big - a bigger serial readbuffer may help more than increasing this unless your gcodes are more than 70 characters long on average.
		however, a larger movebuffer will probably help with lots of short moves, as each move takes a bunch of math to set up
*/
#define	MOVEBUFFER_SIZE	8

/*
	axis calculations, adjust as necessary
*/

#define	X_STEPS_PER_REV						3200.0
#define	Y_STEPS_PER_REV						X_STEPS_PER_REV
// we need more speed than precision on Z, turn off microstepping
#define	Z_STEPS_PER_REV						200.0

#define	X_COG_CIRCUMFERENCE				(4.77 * 16.0)
#define	Y_COG_CIRCUMFERENCE				X_COG_CIRCUMFERENCE
// also try:
// #define	XY_COG_RADIUS			9.5
// #define	XY_COG_CIRCUMFERENCE	(XY_COG_RADIUS * PI * 2)
#define	Z_GEAR_RATIO							1.0

// we need more torque and smoothness at very low speeds on E, maximum microstepping
#define	E_STEPS_PER_REV						3200.0
#define	EXTRUDER_SHAFT_RADIUS			5.0
#define	EXTRUDER_INLET_DIAMETER		3.0
#define	EXTRUDER_NOZZLE_DIAMETER	0.8

#define	STEPS_PER_MM_X						((uint32_t) ((X_STEPS_PER_REV / X_COG_CIRCUMFERENCE) + 0.5))
#define	STEPS_PER_MM_Y						((uint32_t) ((Y_STEPS_PER_REV / Y_COG_CIRCUMFERENCE) + 0.5))
#define	STEPS_PER_MM_Z						((uint32_t) ((Z_STEPS_PER_REV * Z_GEAR_RATIO) + 0.5))

// http://blog.arcol.hu/?p=157 may help with this next one
// I haven't tuned this at all- it's just a placeholder until I read the above carefully enough
// does this refer to filament or extrudate? extrudate depends on XY distance vs E distance.. hm lets go with filament
#define	STEPS_PER_MM_E						((uint32_t) ((E_STEPS_PER_REV / (EXTRUDER_SHAFT_RADIUS * PI * EXTRUDER_INLET_DIAMETER)) + 0.5))

#define	FEEDRATE_FAST_XY					6000
#define	FEEDRATE_SLOW_XY					300
#define	FEEDRATE_FAST_Z						6000
#define	FEEDRATE_SLOW_Z						300

#define	E_STARTSTOP_STEPS					20
#define	FEEDRATE_FAST_E						1200

/*
	calculated values - you shouldn't need to touch these
	however feel free to put in your own values if they can be more precise than the calculated approximations, remembering that they must end up being integers- floating point by preprocessor only thanks!
*/

#define	UM_PER_STEP_X			((uint32_t) ((1000.0 / STEPS_PER_MM_X) + 0.5))
#define	UM_PER_STEP_Y			((uint32_t) ((1000.0 / STEPS_PER_MM_Y) + 0.5))
#define	UM_PER_STEP_Z			((uint32_t) ((1000.0 / STEPS_PER_MM_Z) + 0.5))
#define	UM_PER_STEP_E			((uint32_t) ((1000.0 / STEPS_PER_MM_E) + 0.5))

/*
	should be the same for all machines! ;)
*/
#define	PI	3.1415926535

#endif	/* _MACHINE_H */
