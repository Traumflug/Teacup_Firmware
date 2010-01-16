#ifndef	_PINOUT_H
#define	_PINOUT_H

#include	"arduino.h"

#ifndef		MASK
#define		MASK(PIN)				(1 << PIN)
#endif

#define		_PIN(P)		#P

#define		READ(IO)				(RPORT_ ## IO & MASK(PIN_ ## IO))
#define		WRITE(IO, v)		if (v) { WPORT_ ## IO |= MASK(PIN_ ## IO); } else { WPORT_ ## IO &= ~MASK(PIN_ ## IO); }
#define		SET_INPUT(IO)		(DDR_ ## IO |= MASK(PIN_ ## IO))
#define		SET_OUTPUT(IO)	(DDR_ ## IO &= ~MASK(PIN ## IO))

// #define		X_STEP_PIN	DIO0
// #define		X_DIR_PIN		DIO1
// #define		X_MIN_MIN		DIO2
// #define		X_MAX_PIN		DIO3

// #define		Y_STEP_PIN	DIO4
// #define		Y_DIR_PIN		DIO5
// #define		Y_MIN_MIN		DIO6
// #define		Y_MAX_PIN		DIO7

// #define		Z_STEP_PIN	DIO8
// #define		Z_DIR_PIN		DIO9
// #define		Z_MIN_MIN		DIO10
// #define		Z_MAX_PIN		DIO11

// #define		E_STEP_PIN	DIO12
// #define		E_DIR_PIN		DIO13

void x_step(void);
void y_step(void);
void z_step(void);
void e_step(void);

inline void x_direction(uint8_t dir) {
	WRITE(DIO1, dir);
}

inline uint8_t x_min(void) {
	return READ(DIO2);
}

inline uint8_t x_max(void) {
	return READ(DIO3);
}

inline void y_direction(uint8_t dir) {
	WRITE(DIO5, dir);
}

inline uint8_t y_min(void) {
	return READ(DIO6);
}

inline uint8_t y_max(void) {
	return READ(DIO7);
}

inline void z_direction(uint8_t dir) {
	WRITE(DIO9, dir);
}

inline uint8_t z_min(void) {
	return READ(DIO10);
}

inline uint8_t z_max(void) {
	return READ(DIO11);
}

inline void e_direction(uint8_t dir) {
	WRITE(DIO13, dir);
}

#endif	/* _PINOUT_H */
