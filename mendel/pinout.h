#ifndef	_PINOUT_H
#define	_PINOUT_H

#include	"arduino.h"

#ifndef		MASK
#define		MASK(PIN)				(1 << PIN)
#endif

#define		READ(IO)				(RPORT_ ## IO & MASK(PIN_ ## IO))
#define		WRITE(IO, v)		if (v) { WPORT_ ## IO |= MASK(PIN_ ## IO); } else { WPORT_ ## IO &= ~MASK(PIN_ ## IO); }

#define		SET_INPUT(IO)		(DDR_ ## IO |= MASK(PIN_ ## IO))
#define		SET_OUTPUT(IO)	(DDR_ ## IO &= ~MASK(PIN ## IO))

#define	_x_step(st)					WRITE(AIO0, st)
#define	x_step()						_x_step(1);
#define	x_direction(dir)		WRITE(AIO1, dir)
#define	x_min()							READ(AIO2)
#ifdef	MAX_ENDSTOPS
#define	x_max()							READ(AIO3)
#else
#define	x_max()							(0)
#endif

#define	_y_step(st)					WRITE(DIO2, st)
#define	y_step()						_y_step(1);
#define	y_direction(dir)		WRITE(DIO3, dir)
#define	y_min()							READ(DIO4)
#ifdef	MAX_ENDSTOPS
#define	y_max()							READ(DIO5)
#else
#define	y_max()							(0)
#endif

#define	_z_step(st)					WRITE(DIO6, st)
#define	z_step()						_z_step(1);
#define	z_direction(dir)		WRITE(DIO7, dir)
#define	z_min()							READ(DIO8)
#ifdef	MAX_ENDSTOPS
#define	z_max()							READ(DIO9)
#else
#define	z_max()							(0)
#endif

#define	_e_step(st)					WRITE(AIO4, st)
#define	e_step()						_e_step(1);
#define	e_direction(dir)		WRITE(AIO5, dir)

#define	enable_steppers()		WRITE(DIO10, 1)
#define	disable_steppers()	WRITE(DIO10, 0)

inline void unstep(void) {
	_x_step(0);
	_y_step(0);
	_z_step(0);
	_e_step(0);
}

#endif	/* _PINOUT_H */
