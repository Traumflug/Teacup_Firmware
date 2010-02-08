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
#include	"timer.h"

uint16_t	current_temp = 0;
uint16_t	target_temp  = 0;

int16_t		heater_p     = 0;
int16_t		heater_i     = 0;
int16_t		heater_d     = 0;

int32_t		p_factor			= 680;
int32_t		i_factor			= 18;
int32_t		d_factor			= 200;
int16_t		i_limit	= 500;

uint8_t		temp_flags		= 0;
#define		TEMP_FLAG_PRESENT		1
#define		TEMP_FLAG_TCOPEN		2

uint16_t temp_read() {
	uint16_t temp;

	SPCR = MASK(MSTR) | MASK(SPE) | MASK(SPR0);

	// enable MAX6675
	WRITE(SS, 0);

	// ensure 100ns delay - a bit extra is fine
	delay(1);

	// read MSB
	SPDR = 0;
	for (;(SPSR & MASK(SPIF)) == 0;);
	temp = SPDR;
	temp <<= 8;

	// read LSB
	SPDR = 0;
	for (;(SPSR & MASK(SPIF)) == 0;);
	temp |= SPDR;

	// disable MAX6675
	WRITE(SS, 1);

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

// 	if (DEBUG)
// 		serial_writechar(']');

	return 0;
}

void temp_set(uint16_t t) {
	target_temp = t;
}

uint16_t temp_get() {
	return current_temp;
}

uint16_t temp_get_target() {
	return target_temp;
}

uint8_t	temp_achieved() {
	if (current_temp >= target_temp)
		if ((current_temp - target_temp) < TEMP_HYSTERESIS)
			return 255;
	if (current_temp < target_temp)
		if ((target_temp - current_temp) < TEMP_HYSTERESIS)
			return 255;
	return 0;
}

void temp_print() {
	serial_writestr_P(PSTR("T: "));
	if (temp_flags & TEMP_FLAG_TCOPEN) {
		serial_writestr_P(PSTR("no thermocouple!"));
	}
	else {
		serwrite_uint16(temp_get() >> 2);
		serial_writechar('.');
		if (current_temp) {
			if ((current_temp & 3) == 3)
				serial_writechar('7');
			else if ((current_temp & 3) == 1)
				serial_writechar('2');
			serial_writechar('5');
		}
		else {
			serial_writechar('0');
		}
// 		serial_writestr_P(PSTR("°C"));
	}
	serial_writechar('\n');
}

void temp_tick() {
	if (target_temp) {
		uint16_t last_temp = current_temp;
		temp_read();

	// 	if (DEBUG)
	// 		serial_writestr_P(PSTR("T{"));

		int16_t	t_error = target_temp - current_temp;

	// 	if (DEBUG) {
	// 		serial_writestr_P(PSTR("E:"));
	// 		serwrite_int16(t_error);
	// 	}

		// PID stuff
		// proportional
		heater_p = t_error;

		// integral
		heater_i += t_error;
		// prevent integrator wind-up
		if (heater_i > i_limit)
			heater_i = i_limit;
		else if (heater_i < -i_limit)
			heater_i = -i_limit;

		// derivative
		// note: D follows temp rather than error so there's no large derivative when the target changes
		heater_d = (current_temp - last_temp);

		// combine factors
		int32_t pid_output_intermed = (
				(
					(((int32_t) heater_p) * p_factor) +
					(((int32_t) heater_i) * i_factor) +
					(((int32_t) heater_d) * d_factor)
				) / PID_SCALE
			);

		// rebase and limit factors
		uint8_t pid_output;
		if (pid_output_intermed > 127)
			pid_output = 255;
		else if (pid_output_intermed < -128)
			pid_output = 0;
		else
			pid_output = (pid_output_intermed + 128);

	// 	if (DEBUG) {
	// 		serial_writestr_P(PSTR(",O:"));
	// 		serwrite_uint8(pid_output);
	// 	}

		#ifdef	HEATER_PWM
			HEATER_PWM = pid_output;
		#else
			if (pid_output >= 128) {
				enable_heater();
			}
			else {
				disable_heater();
			}
		#endif

	// 	if (DEBUG)
	// 		serial_writechar('}');
	}
}
