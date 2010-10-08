#ifndef	_HEATER_H
#define	_HEATER_H

#include	"config.h"

#ifdef HEATER_PIN

#ifdef	HEATER_PWM
	#define	enable_heater()			do { TCCR0A |=  MASK(COM0A1); } while (0)
	#define	disable_heater()		do { TCCR0A &= ~MASK(COM0A1); } while (0)
#else
	#define	enable_heater()			WRITE(HEATER_PIN, 1)
	#define	disable_heater()		WRITE(HEATER_PIN, 0)
#endif

// extruder heater PID factors
// google "PID without a PHD" if you don't understand this PID stuff
extern int32_t p_factor;
extern int32_t i_factor;
extern int32_t d_factor;
extern int16_t i_limit;

#define		PID_SCALE			1024L
#define		I_LIMIT				4000

void heater_init(void);
void heater_save_settings(void);
void heater_tick(int16_t current_temp, int16_t target_temp);

#else	/* HEATER_PIN */

// if there is no heater pin, there is no heater
#define enable_heater()					/* empty */
#define disable_heater()				/* empty */
#define heater_init()						/* empty */
#define heater_save_settings()	/* empty */
#define heater_tick(p1, p2)			/* empty */

#endif 	/* HEATER_PIN */
#endif	/* _HEATER_H */
