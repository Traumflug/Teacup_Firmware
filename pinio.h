#ifndef	_PINIO_H
#define	_PINIO_H

#ifndef	X_INVERT_DIR
	#define	X_INVERT_DIR 0
#endif
#ifndef	X_INVERT_MIN
	#define	X_INVERT_MIN 0
#endif
#ifndef	X_INVERT_MAX
	#define	X_INVERT_MAX 0
#endif
#ifndef	X_INVERT_ENABLE
	#define	X_INVERT_ENABLE 0
#endif

#ifndef	Y_INVERT_DIR
	#define	Y_INVERT_DIR 0
#endif
#ifndef	Y_INVERT_MIN
	#define	Y_INVERT_MIN 0
#endif
#ifndef	Y_INVERT_MAX
	#define	Y_INVERT_MAX 0
#endif
#ifndef	Y_INVERT_ENABLE
	#define	Y_INVERT_ENABLE 0
#endif

#ifndef	Z_INVERT_DIR
	#define	Z_INVERT_DIR 0
#endif
#ifndef	Z_INVERT_MIN
	#define	Z_INVERT_MIN 0
#endif
#ifndef	Z_INVERT_MAX
	#define	Z_INVERT_MAX 0
#endif
#ifndef	Z_INVERT_ENABLE
	#define	Z_INVERT_ENABLE 0
#endif

#ifndef	E_INVERT_DIR
	#define	E_INVERT_DIR 0
#endif

#ifndef	STEPPER_ENABLE_INVERT
	#define	STEPPER_ENABLE_INVERT 0
#endif

/*
Power
*/

#ifdef	STEPPER_ENABLE_PIN
	#define	power_on()					do { WRITE(STEPPER_ENABLE_PIN, STEPPER_ENABLE_INVERT); SET_OUTPUT(STEPPER_ENABLE_PIN); } while (0)
#elif defined PS_ON_PIN
	#define	power_on()					do { WRITE(PS_ON_PIN, 0); SET_OUTPUT(PS_ON_PIN); } while (0)
#else
	#define	power_on()					do { } while (0)
#endif

void power_off(void);

/*
X Stepper
*/

#define	_x_step(st)						WRITE(X_STEP_PIN, st)
#define	x_step()							_x_step(1);
#define	x_direction(dir)			WRITE(X_DIR_PIN, dir ^ X_INVERT_DIR)
#define	x_min()								(READ(X_MIN_PIN)?(X_INVERT_MIN ^ 1):X_INVERT_MIN)
#ifdef	X_MAX_PIN
	#define	x_max()							(READ(X_MAX_PIN)?(X_INVERT_MAX ^ 1):X_INVERT_MAX)
#else
	#define	x_max()							(0)
#endif

/*
Y Stepper
*/

#define	_y_step(st)						WRITE(Y_STEP_PIN, st)
#define	y_step()							_y_step(1);
#define	y_direction(dir)			WRITE(Y_DIR_PIN, dir ^ Y_INVERT_DIR)
#define	y_min()								(READ(Y_MIN_PIN)?(Y_INVERT_MIN ^ 1):Y_INVERT_MIN)
#ifdef	Y_MAX_PIN
	#define	y_max()							(READ(Y_MAX_PIN)?(Y_INVERT_MAX ^ 1):Y_INVERT_MAX)
#else
	#define	y_max()							(0)
#endif

/*
Z Stepper
*/

#define	_z_step(st)						WRITE(Z_STEP_PIN, st)
#define	z_step()							_z_step(1);
#define	z_direction(dir)			WRITE(Z_DIR_PIN, dir ^ Z_INVERT_DIR)
#define	z_min()								(READ(Z_MIN_PIN)?(Z_INVERT_MIN ^ 1):Z_INVERT_MIN)
#ifdef	Z_MAX_PIN
	#define	z_max()							(READ(Z_MAX_PIN)?(Z_INVERT_MAX ^ 1):Z_INVERT_MAX)
#else
	#define	z_max()							(0)
#endif

/*
Extruder
*/

#define	_e_step(st)						WRITE(E_STEP_PIN, st)
#define	e_step()							_e_step(1);
#define	e_direction(dir)			WRITE(E_DIR_PIN, dir ^ E_INVERT_DIR)

/*
End Step - All Steppers
(so we don't have to delay in interrupt context)
*/

#ifndef	DC_EXTRUDER
	#define unstep() 							do { _x_step(0); _y_step(0); _z_step(0); _e_step(0); } while (0)
#else
	#define unstep() 							do { _x_step(0); _y_step(0); _z_step(0); } while (0)
#endif

/*
Stepper Enable Pins
*/

#ifdef	X_ENABLE_PIN
	#define	x_enable()					do { WRITE(X_ENABLE_PIN, X_INVERT_ENABLE); SET_OUTPUT(X_ENABLE_PIN); } while (0)
	#define	x_disable()					do { WRITE(X_ENABLE_PIN, X_INVERT_ENABLE ^ 1); SET_OUTPUT(X_ENABLE_PIN); } while (0)
#else
	#define	x_enable()					do { } while (0)
	#define	x_disable()					do { } while (0)
#endif

#ifdef	Y_ENABLE_PIN
	#define	y_enable()					do { WRITE(Y_ENABLE_PIN, Y_INVERT_ENABLE); SET_OUTPUT(Y_ENABLE_PIN); } while (0)
	#define	y_disable()					do { WRITE(Y_ENABLE_PIN, Y_INVERT_ENABLE ^ 1); SET_OUTPUT(Y_ENABLE_PIN); } while (0)
#else
	#define	y_enable()					do { } while (0)
	#define	y_disable()					do { } while (0)
#endif

#ifdef	Z_ENABLE_PIN
	#define	z_enable()					do { WRITE(Z_ENABLE_PIN, Z_INVERT_ENABLE); SET_OUTPUT(Z_ENABLE_PIN); } while (0)
	#define	z_disable()					do { WRITE(Z_ENABLE_PIN, Z_INVERT_ENABLE ^ 1); SET_OUTPUT(Z_ENABLE_PIN); } while (0)
#else
	#define	z_enable()					do { } while (0)
	#define	z_disable()					do { } while (0)
#endif

#endif	/* _PINIO_H */
