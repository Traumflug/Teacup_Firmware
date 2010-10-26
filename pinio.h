#ifndef	_PINIO_H
#define	_PINIO_H

/*
Power
*/

#ifdef	STEPPER_ENABLE_PIN
	#define	power_on()					do { WRITE(STEPPER_ENABLE_PIN, 0); SET_OUTPUT(STEPPER_ENABLE_PIN); } while (0)
#elif defined PS_ON_PIN
	#define	power_on()					do { WRITE(PS_ON_PIN, 0); SET_OUTPUT(PS_ON_PIN); } while (0)
#else
	#define	power_on()					do { } while (0)
#endif

void power_off(void);

/*
Stepper Enable Pins
*/

#ifdef	X_ENABLE_PIN
	#define	x_enable()					do { WRITE(X_ENABLE_PIN, 0); SET_OUTPUT(X_ENABLE_PIN); } while (0)
	#define	x_disable()					do { WRITE(X_ENABLE_PIN, 1); SET_OUTPUT(X_ENABLE_PIN); } while (0)
#else
	#define	x_enable()					do { } while (0)
	#define	x_disable()					do { } while (0)
#endif

#ifdef	Y_ENABLE_PIN
	#define	y_enable()					do { WRITE(Y_ENABLE_PIN, 0); SET_OUTPUT(Y_ENABLE_PIN); } while (0)
	#define	y_disable()					do { WRITE(Y_ENABLE_PIN, 1); SET_OUTPUT(Y_ENABLE_PIN); } while (0)
#else
	#define	y_enable()					do { } while (0)
	#define	y_disable()					do { } while (0)
#endif

#ifdef	Z_ENABLE_PIN
	#define	z_enable()					do { WRITE(Z_ENABLE_PIN, 0); SET_OUTPUT(Z_ENABLE_PIN); } while (0)
	#define	z_disable()					do { WRITE(Z_ENABLE_PIN, 1); SET_OUTPUT(Z_ENABLE_PIN); } while (0)
#else
	#define	z_enable()					do { } while (0)
	#define	z_disable()					do { } while (0)
#endif

#endif	/* _PINIO_H */
