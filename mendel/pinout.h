#ifndef	_PINOUT_H
#define	_PINOUT_H

#include	"arduino.h"

/*
	Machine Pin Definitions
*/

// RXD											DIO0
// TXD											DIO1

#define	X_STEP_PIN					AIO0
#define	X_DIR_PIN						AIO1
#define	X_MIN_PIN						AIO2

#define	Y_STEP_PIN					AIO3
#define	Y_DIR_PIN						AIO4
#define	Y_MIN_PIN						AIO5

#define	Z_STEP_PIN					DIO5
#define	Z_DIR_PIN						DIO6
#define	Z_MIN_PIN						DIO7

#define	E_STEP_PIN					DIO2
#define	E_DIR_PIN						DIO3

// list of PWM-able pins
// OC0A											DIO6
// OC0B											DIO5
// OC1A											DIO9
// OC1B											DIO10
// OC2A											DIO11
// OC2B											DIO3

#define	HEATER_PIN					DIO6
#define	HEATER_PIN_PWM			OC0A

#define	SCK									DIO13
#define	MISO								DIO12
#define	MOSI								DIO11
#define	SS									DIO10

/*
	X Stepper
*/

#define	_x_step(st)					WRITE(X_STEP_PIN, st)
#define	x_step()						_x_step(1);
#define	x_direction(dir)		WRITE(X_DIR_PIN, dir)
#define	x_min()							READ(X_MIN_PIN)
#ifdef	X_MAX_PIN
	#define	x_max()						READ(X_MAX_PIN)
#else
	#define	x_max()						(0)
#endif

/*
	Y Stepper
*/

#define	_y_step(st)					WRITE(Y_STEP_PIN, st)
#define	y_step()						_y_step(1);
#define	y_direction(dir)		WRITE(Y_DIR_PIN, dir)
#define	y_min()							READ(Y_MIN_PIN)
#ifdef	Y_MAX_PIN
	#define	y_max()						READ(Y_MAX_PIN)
#else
	#define	y_max()						(0)
#endif

/*
	Z Stepper
*/

#define	_z_step(st)					WRITE(Z_STEP_PIN, st)
#define	z_step()						_z_step(1);
#define	z_direction(dir)		WRITE(Z_DIR_PIN, dir)
#define	z_min()							READ(Z_MIN_PIN)
#ifdef	Z_MAX_PIN
	#define	z_max()						READ(Z_MAX_PIN)
#else
	#define	z_max()						(0)
#endif

/*
	Extruder
*/

#define	_e_step(st)					WRITE(E_STEP_PIN, st)
#define	e_step()						_e_step(1);
#define	e_direction(dir)		WRITE(E_DIR_PIN, dir)

/*
	Heater
*/

#define	enable_heater()			WRITE(HEATER_PIN, 1)
#define	disable_heater()		WRITE(HEATER_PIN, 0)

/*
	Stepper Enable (ATX PSU pwr_good signal?)
*/

#ifdef	STEPPER_ENABLE_PIN
	#define	enable_steppers()		WRITE(STEPPER_ENABLE_PIN, 1)
	#define	disable_steppers()	WRITE(STEPPER_ENABLE_PIN, 0)
#else
	#define	enable_steppers()		if (0) {}
	#define	disable_steppers()	if (0) {}
#endif

/*
	End Step - All Steppers
	(so we don't have to delay in interrupt context)
*/

#define unstep() 						do { _x_step(0); _y_step(0); _z_step(0); _e_step(0); } while (0)

#endif	/* _PINOUT_H */
