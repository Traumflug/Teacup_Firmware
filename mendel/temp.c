/*
	temp.c

	This file currently reads temp from a MAX6675 on the SPI bus.

	ALL VALUES are in units of 0.25 degrees celsius, so temp_set(500) will set the temperature to 125 celsius, and temp_get() = 600 is reporting a temperature of 150 celsius.

	the conversion to/from this unit is done in gcode.c, near:
					if (next_target.M == 104)
						next_target.S = decfloat_to_int(&read_digit, 4, 1);
	and
			// M105- get temperature
			case 105:
				uint16_t t = temp_get();

	note that the MAX6675 can't do more than approx 4 conversions per second
*/

#include "temp.h"

#include	"machine.h"
#include	"pinout.h"
#include	"clock.h"
#include	"serial.h"
#include	"sermsg.h"

uint16_t	current_temp = 0;
uint16_t	target_temp  = 0;

int16_t		heater_p     = 0;
int16_t		heater_i     = 0;
int16_t		heater_d     = 0;

int32_t		p_factor			= 680;
int32_t		i_factor			= 18;
int32_t		d_factor			= 200;

uint8_t		temp_flags		= 0;
#define		TEMP_FLAG_PRESENT		1
#define		TEMP_FLAG_TCOPEN		2

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

	temp_flags = 0;
	if ((temp & 0x8002) == 0) {
		// got "device id"
		temp_flags |= TEMP_FLAG_PRESENT;
		if (temp & 4) {
			// thermocouple open
			temp_flags |= TEMP_FLAG_TCOPEN;
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

uint16_t temp_get() {
	return current_temp;
}

void temp_print() {
	serial_writestr_P(PSTR("T: "));
	if (temp_flags & TEMP_FLAG_TCOPEN) {
		serial_writestr_P(PSTR("no thermocouple!"));
	}
	else {
		serwrite_uint16(temp_get());
		serial_writestr_P(PSTR("°C"));
	}
	serial_writechar('\n');
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
