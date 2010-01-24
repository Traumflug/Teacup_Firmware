#ifndef	_MACHINE_H
#define	_MACHINE_H

/*
	machine variables
*/

#define	MOVEBUFFER_SIZE	8

/*
	axis calculations, adjust as necessary
*/

#define	XY_STEPS_PER_REV			1600.0
#define	XY_COG_CIRCUMFERENCE	(4.77 * 16.0)

#define	EXTRUDER_STEPS_PER_REV		3200
#define	EXTRUDER_SHAFT_RADIUS			5
#define	EXTRUDER_INLET_DIAMETER		3
#define	EXTRUDER_NOZZLE_DIAMETER	0.8

#define	STEPS_PER_MM_X				((XY_STEPS_PER_REV / XY_COG_CIRCUMFERENCE) + 0.5)
#define	STEPS_PER_MM_Y				((XY_STEPS_PER_REV / XY_COG_CIRCUMFERENCE) + 0.5)
#define	STEPS_PER_MM_Z				(200L)
#define	STEPS_PER_MM_E				(EXTRUDER_STEPS_PER_REV * EXTRUDER_SHAFT_RADIUS * PI * EXTRUDER_INLET_DIAMETER / EXTRUDER_NOZZLE_DIAMETER)

#define	FEEDRATE_FAST_XY			2400
#define	FEEDRATE_SLOW_XY			120
#define	FEEDRATE_FAST_Z				240
#define	FEEDRATE_SLOW_Z				12

#define	E_STARTSTOP_STEPS			20
#define	FEEDRATE_FAST_E				1200

/*
	should be the same for all machines ;)
*/
#define	PI	3.1415926535

#endif	/* _MACHINE_H */
