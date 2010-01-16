#ifndef	_MACHINE_H
#define	_MACHINE_H

/*
	machine variables
*/
#define	AXIS_COUNT 5
#define	AXIS_HOMING_COUNT	3

/*
	axis calculations, adjust as necessary
*/

#define	XY_STEPS_PER_REV			3200
#define	XY_COG_CIRCUMFERENCE	(4.77 * 16)

#define	EXTRUDER_STEPS_PER_REV		3200
#define	EXTRUDER_SHAFT_RADIUS			5
#define	EXTRUDER_INLET_DIAMETER		3
#define	EXTRUDER_NOZZLE_DIAMETER	0.8


#define	STEPS_PER_MM_X				(XY_STEPS_PER_REV * XY_COG_CIRCUMFERENCE)
#define	STEPS_PER_MM_Y				(XY_STEPS_PER_REV * XY_COG_CIRCUMFERENCE)
#define	STEPS_PER_MM_Z				(3200)
#define	STEPS_PER_MM_E				(EXTRUDER_STEPS_PER_REV * EXTRUDER_SHAFT_RADIUS * PI * EXTRUDER_INLET_DIAMETER / EXTRUDER_NOZZLE_DIAMETER)

#define	STEPS_PER_MM_PER_S_F	(3200)

#define	STEPS_PER_MM_F				STEPS_PER_MM_PER_S_F

/*
	F is sent in units of millimeters per second
	and implemented as microseconds per step

	MM/S * steps/mm * 1000000us/s = steps per microsecond
*/


#define	FAST_MM_PER_SEC				40
#define	SLOW_MM_PER_SEC				20
#define	ACCEL									10


#endif	/* _MACHINE_H */
