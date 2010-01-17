/*
	temp.c

	This file currently reads temp from a MAX6675 on the SPI bus
*/

#include "temp.h"

#include	"machine.h"
#include	"pinout.h"
#include	"clock.h"

uint16_t	current_temp = 0;
uint16_t	target_temp  = 0;

int16_t		heater_p     = 0;
int16_t		heater_i     = 0;
int16_t		heater_d     = 0;

int32_t		p_factor			= 680;
int32_t		i_factor			= 18;
int32_t		d_factor			= 200;

#define		PID_SCALE			1024L

uint16_t temp_read() {
	uint16_t temp;
	SPCR = MASK(MSTR) | MASK(SPE);

	WRITE(SS, 1);

	SPDR = 0;
	for (;(SPSR & MASK(SPIF)) == 0;);
	temp = SPDR << 8;

	SPDR = 0;
	for (;(SPSR & MASK(SPIF)) == 0;);
	temp |= SPDR;

	if ((temp & 0x8002) == 0) {
		// got "device id"
		if (temp & 4) {
			// thermocouple open
		}
		else {
			current_temp = temp >> 3;
			return current_temp;
		}
	}

	WRITE(SS, 0);

	return 0;
}

void temp_set(uint16_t t) {
	target_temp = t;
}

void temp_tick() {
	uint16_t last_temp = current_temp;
	temp_read();

	int16_t	t_delta = target_temp - current_temp;

	// PID stuff
	heater_p = t_delta;
	heater_i += t_delta;
	// note: D follows temp rather than error so there's no large derivative when the target temperature changes
	heater_d = (current_temp - last_temp);

	uint8_t pid_output = (
			(
				(((int32_t) heater_p) * p_factor) +
				(((int32_t) heater_i) * i_factor) +
				(((int32_t) heater_d) * d_factor)
			) / PID_SCALE
		);

#ifdef	HEATER_PIN_PWMABLE
	HEATER_PIN_PWMABLE = pid_output
#else
	if (pid_output >= 128) {
		enable_heater();
	}
	else {
		disable_heater();
	}
#endif
}
