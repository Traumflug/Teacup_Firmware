#ifndef	_MACHINE_H
#define	_MACHINE_H

/*
	machine variables
*/

#define	MOVEBUFFER_SIZE	8

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

/*
	heater PID variables
	used as int16_t * FACTOR so don't put brackets around them
*/

#define	P_FACTOR	133 / 1024
#define	I_FACTOR	17 / 1024
#define	D_FACTOR  180 / 1024

#endif	/* _MACHINE_H */
