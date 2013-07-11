/** \file
 \brief I/O primitives - step, enable, direction, endstops etc
*/

#ifndef	_PINIO_H
#define	_PINIO_H

#include	"config.h"

/*
Power
*/

/// psu_timeout is set to zero when we step, and increases over time so we can
/// turn the motors off when they've been idle for a while.
/// A second function is to guarantee a minimum on time of the PSU.
/// Timeout counting is done in clock.c.
/// It is used inside and outside of interrupts, which is why it has been made volatile
extern volatile uint8_t psu_timeout;

static void power_init(void);
inline void power_init(void) {
  #ifdef PS_MOSFET_PIN
    WRITE(PS_MOSFET_PIN, 0);
    SET_OUTPUT(PS_MOSFET_PIN);
  #endif
}

void power_on(void);
void power_off(void);

/*
X Stepper
*/

#define	_x_step(st)						WRITE(X_STEP_PIN, st)
#define	x_step()							_x_step(1);
#ifndef	X_INVERT_DIR
	#define	x_direction(dir)		WRITE(X_DIR_PIN, dir)
#else
	#define	x_direction(dir)		WRITE(X_DIR_PIN, (dir)^1)
#endif
#ifdef	X_MIN_PIN
	#ifndef X_INVERT_MIN
		#define x_min()						(READ(X_MIN_PIN)?1:0)
	#else
		#define x_min()						(READ(X_MIN_PIN)?0:1)
	#endif
#else
	#define	x_min()							(0)
#endif
#ifdef	X_MAX_PIN
	#ifndef X_INVERT_MAX
		#define x_max()						(READ(X_MAX_PIN)?1:0)
	#else
		#define x_max()						(READ(X_MAX_PIN)?0:1)
	#endif
#else
	#define	x_max()							(0)
#endif

/*
Y Stepper
*/

#define	_y_step(st)						WRITE(Y_STEP_PIN, st)
#define	y_step()							_y_step(1);
#ifndef	Y_INVERT_DIR
	#define	y_direction(dir)		WRITE(Y_DIR_PIN, dir)
#else
	#define	y_direction(dir)		WRITE(Y_DIR_PIN, (dir)^1)
#endif
#ifdef	Y_MIN_PIN
	#ifndef Y_INVERT_MIN
		#define y_min()						(READ(Y_MIN_PIN)?1:0)
	#else
		#define y_min()						(READ(Y_MIN_PIN)?0:1)
	#endif
#else
	#define	y_min()							(0)
#endif
#ifdef	Y_MAX_PIN
	#ifndef Y_INVERT_MAX
		#define y_max()						(READ(Y_MAX_PIN)?1:0)
	#else
		#define y_max()						(READ(Y_MAX_PIN)?0:1)
	#endif
#else
	#define	y_max()							(0)
#endif

/*
Z Stepper
*/

#if defined Z_STEP_PIN && defined Z_DIR_PIN
	#define	_z_step(st)					WRITE(Z_STEP_PIN, st)
	#define	z_step()						_z_step(1);
	#ifndef	Z_INVERT_DIR
		#define	z_direction(dir)	WRITE(Z_DIR_PIN, dir)
	#else
		#define	z_direction(dir)	WRITE(Z_DIR_PIN, (dir)^1)
	#endif
#else
	#define	_z_step(x)					do { } while (0)
	#define	z_step()						do { } while (0)
	#define	z_direction(x)			do { } while (0)
#endif
#ifdef	Z_MIN_PIN
	#ifndef Z_INVERT_MIN
		#define z_min()						(READ(Z_MIN_PIN)?1:0)
	#else
		#define z_min()						(READ(Z_MIN_PIN)?0:1)
	#endif
#else
	#define	z_min()							(0)
#endif
#ifdef	Z_MAX_PIN
	#ifndef Z_INVERT_MAX
		#define z_max()						(READ(Z_MAX_PIN)?1:0)
	#else
		#define z_max()						(READ(Z_MAX_PIN)?0:1)
	#endif
#else
	#define	z_max()							(0)
#endif

/*
Extruder
*/

#if defined E_STEP_PIN && defined E_DIR_PIN
	#define	_e_step(st)					WRITE(E_STEP_PIN, st)
	#define	e_step()						_e_step(1);
	#ifndef	E_INVERT_DIR
		#define	e_direction(dir)	WRITE(E_DIR_PIN, dir)
	#else
		#define	e_direction(dir)	WRITE(E_DIR_PIN, (dir)^1)
	#endif
#else
	#define	_e_step(st)					do { } while (0)
	#define	e_step()						do { } while (0)
	#define	e_direction(dir)		do { } while (0)
#endif

/*
End Step - All Steppers
(so we don't have to delay in interrupt context)
*/

#define unstep() 							do { _x_step(0); _y_step(0); _z_step(0); _e_step(0); } while (0)

/*
Stepper Enable Pins
*/

#ifdef	STEPPER_ENABLE_PIN
	#ifdef	STEPPER_INVERT_ENABLE
		#define stepper_enable()	do { WRITE(STEPPER_ENABLE_PIN, 0); } while (0)
		#define stepper_disable()	do { WRITE(STEPPER_ENABLE_PIN, 1); } while (0)
	#else
		#define stepper_enable()	do { WRITE(STEPPER_ENABLE_PIN, 1); } while (0)
		#define stepper_disable()	do { WRITE(STEPPER_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	stepper_enable()		do { } while (0)
	#define	stepper_disable()		do { } while (0)
#endif

#ifdef	X_ENABLE_PIN
	#ifdef	X_INVERT_ENABLE
		#define	x_enable()				do { WRITE(X_ENABLE_PIN, 0); } while (0)
		#define	x_disable()				do { WRITE(X_ENABLE_PIN, 1); } while (0)
	#else
		#define	x_enable()				do { WRITE(X_ENABLE_PIN, 1); } while (0)
		#define	x_disable()				do { WRITE(X_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	x_enable()					do { } while (0)
	#define	x_disable()					do { } while (0)
#endif

#ifdef	Y_ENABLE_PIN
	#ifdef	Y_INVERT_ENABLE
		#define	y_enable()				do { WRITE(Y_ENABLE_PIN, 0); } while (0)
		#define	y_disable()				do { WRITE(Y_ENABLE_PIN, 1); } while (0)
	#else
		#define	y_enable()				do { WRITE(Y_ENABLE_PIN, 1); } while (0)
		#define	y_disable()				do { WRITE(Y_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	y_enable()					do { } while (0)
	#define	y_disable()					do { } while (0)
#endif

#ifdef	Z_ENABLE_PIN
	#ifdef	Z_INVERT_ENABLE
		#define	z_enable()				do { WRITE(Z_ENABLE_PIN, 0); } while (0)
		#define	z_disable()				do { WRITE(Z_ENABLE_PIN, 1); } while (0)
	#else
		#define	z_enable()				do { WRITE(Z_ENABLE_PIN, 1); } while (0)
		#define	z_disable()				do { WRITE(Z_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	z_enable()					do { } while (0)
	#define	z_disable()					do { } while (0)
#endif

#ifdef	E_ENABLE_PIN
	#ifdef	E_INVERT_ENABLE
		#define	e_enable()				do { WRITE(E_ENABLE_PIN, 0); } while (0)
		#define	e_disable()				do { WRITE(E_ENABLE_PIN, 1); } while (0)
	#else
		#define	e_enable()				do { WRITE(E_ENABLE_PIN, 1); } while (0)
		#define	e_disable()				do { WRITE(E_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	e_enable()					do { } while (0)
	#define	e_disable()					do { } while (0)
#endif

/*
Internal pullup resistors for endstops
*/
static void endstops_on(void) __attribute__ ((always_inline));
inline void endstops_on(void) {
	#ifdef USE_INTERNAL_PULLUPS
		#ifdef X_MIN_PIN
			WRITE(X_MIN_PIN, 1);
		#endif
		#ifdef X_MAX_PIN
			WRITE(X_MAX_PIN, 1);
		#endif
		#ifdef Y_MIN_PIN
			WRITE(Y_MIN_PIN, 1);
		#endif
		#ifdef Y_MAX_PIN
			WRITE(Y_MAX_PIN, 1);
		#endif
		#ifdef Z_MIN_PIN
			WRITE(Z_MIN_PIN, 1);
		#endif
		#ifdef Z_MAX_PIN
			WRITE(Z_MAX_PIN, 1);
		#endif
	#endif
}

static void endstops_off(void) __attribute__ ((always_inline));
inline void endstops_off(void) {
	#ifdef USE_INTERNAL_PULLUPS
		#ifdef X_MIN_PIN
			WRITE(X_MIN_PIN, 0);
		#endif
		#ifdef X_MAX_PIN
			WRITE(X_MAX_PIN, 0);
		#endif
		#ifdef Y_MIN_PIN
			WRITE(Y_MIN_PIN, 0);
		#endif
		#ifdef Y_MAX_PIN
			WRITE(Y_MAX_PIN, 0);
		#endif
		#ifdef Z_MIN_PIN
			WRITE(Z_MIN_PIN, 0);
		#endif
		#ifdef Z_MAX_PIN
			WRITE(Z_MAX_PIN, 0);
		#endif
	#endif
}


#endif	/* _PINIO_H */
