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

#include	<avr/eeprom.h>

#include	"machine.h"
#include	"pinout.h"
#include	"clock.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"timer.h"
#include	"dda.h"
#include	"sersendf.h"
#include	"debug.h"

uint16_t	current_temp = 0;
uint16_t	target_temp  = 0;

int16_t		heater_p     = 0;
int16_t		heater_i     = 0;
int16_t		heater_d     = 0;

#define		DEFAULT_P				4096
#define		DEFAULT_I				256
#define		DEFAULT_D				-14336
#define		DEFAULT_I_LIMIT	768
int32_t		p_factor			= 0;
int32_t		i_factor			= 0;
int32_t		d_factor			= 0;
int16_t		i_limit				= 0;

int32_t		EEMEM EE_p_factor;
int32_t		EEMEM EE_i_factor;
int32_t		EEMEM	EE_d_factor;
int16_t		EEMEM EE_i_limit;

uint8_t		temp_flags		= 0;
#define		TEMP_FLAG_PRESENT		1
#define		TEMP_FLAG_TCOPEN		2

uint8_t		temp_residency	= 0;

#define		TH_COUNT	8
uint16_t	temp_history[TH_COUNT] __attribute__ ((__section__ (".bss")));
uint8_t		th_p = 0;

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

void temp_init() {
	p_factor = eeprom_read_dword((uint32_t *) &EE_p_factor);
	i_factor = eeprom_read_dword((uint32_t *) &EE_i_factor);
	d_factor = eeprom_read_dword((uint32_t *) &EE_d_factor);
	i_limit = eeprom_read_word((uint16_t *) &EE_i_limit);

	if ((p_factor == 0) && (i_factor == 0) && (d_factor == 0) && (i_limit == 0)) {
		p_factor = DEFAULT_P;
		i_factor = DEFAULT_I;
		d_factor = DEFAULT_D;
		i_limit = DEFAULT_I_LIMIT;
	}
}

void temp_save_settings() {
	eeprom_write_dword((uint32_t *) &EE_p_factor, p_factor);
	eeprom_write_dword((uint32_t *) &EE_i_factor, i_factor);
	eeprom_write_dword((uint32_t *) &EE_d_factor, d_factor);
	eeprom_write_word((uint16_t *) &EE_i_limit, i_limit);
}

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

	return 0;
}

void temp_set(uint16_t t) {
	if (t) {
		steptimeout = 0;
		power_on();
	}
	target_temp = t;
}

uint16_t temp_get() {
	return current_temp;
}

uint16_t temp_get_target() {
	return target_temp;
}

uint8_t	temp_achieved() {
	if (temp_residency >= TEMP_RESIDENCY_TIME)
		return 255;
	return 0;
}

void temp_print() {
// 	serial_writestr_P(PSTR("T: "));
	if (temp_flags & TEMP_FLAG_TCOPEN) {
		serial_writestr_P(PSTR("T: no thermocouple!\n"));
	}
	else {
// 		serwrite_uint16(current_temp >> 2);
// 		serial_writechar('.');
// 		if (current_temp & 3) {
// 			if ((current_temp & 3) == 3)
// 				serial_writechar('7');
// 			else if ((current_temp & 3) == 1)
// 				serial_writechar('2');
// 			serial_writechar('5');
// 		}
// 		else {
// 			serial_writechar('0');
// 		}
// // 		serial_writestr_P(PSTR("°C"));
// 		serial_writechar('/');
// 		serwrite_uint16(target_temp >> 2);
// 		serial_writechar('.');
// 		if (target_temp & 3) {
// 			if ((target_temp & 3) == 3)
// 				serial_writechar('7');
// 			else if ((target_temp & 3) == 1)
// 				serial_writechar('2');
// 			serial_writechar('5');
// 		}
// 		else {
// 			serial_writechar('0');
// 		}
//
// 		serial_writestr_P(PSTR(" :"));
// 		serwrite_uint8(temp_residency);
		uint8_t c = 0, t = 0;

		if (current_temp & 3)
			c = 5;
		if ((current_temp & 3) == 1)
			c += 20;
		else if ((current_temp & 3) == 3)
			c += 70;

		if (target_temp & 3)
			t = 5;
		if ((target_temp & 3) == 1)
			t += 20;
		else if ((target_temp & 3) == 3)
			t += 70;

		sersendf_P(PSTR("%u.%u/%u.%u :%u\n"), current_temp >> 2, c, target_temp >> 2, t, temp_residency);
	}
// 	serial_writechar('\n');
}

void temp_tick() {
	if (target_temp) {
		steptimeout = 0;

// 		uint16_t last_temp = current_temp;
		temp_read();

		temp_history[th_p++] = current_temp;
		th_p &= (TH_COUNT - 1);

		if (ABSDELTA(current_temp, target_temp) > TEMP_HYSTERESIS)
			temp_residency = 0;
		else if (temp_residency < TEMP_RESIDENCY_TIME)
			temp_residency++;

// 		if (debug_flags & DEBUG_PID)
// 			serial_writestr_P(PSTR("T{"));

		int16_t	t_error = target_temp - current_temp;

// 		if (debug_flags & DEBUG_PID) {
// 			serial_writestr_P(PSTR("E:"));
// 			serwrite_int16(t_error);
// 		}

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
// 		heater_d = (current_temp - last_temp);
		heater_d = current_temp - temp_history[th_p];

// 		if (debug_flags & DEBUG_PID) {
// 			serial_writestr_P(PSTR(", P:"));
// 			serwrite_int16(heater_p);
// 			serial_writestr_P(PSTR(" * "));
// 			serwrite_int32(p_factor);
// 			serial_writestr_P(PSTR(" = "));
// 			serwrite_int32((int32_t) heater_p * p_factor / PID_SCALE);
// 			serial_writestr_P(PSTR(" / I:"));
// 			serwrite_int16(heater_i);
// 			serial_writestr_P(PSTR(" * "));
// 			serwrite_int32(i_factor);
// 			serial_writestr_P(PSTR(" = "));
// 			serwrite_int32((int32_t) heater_i * i_factor / PID_SCALE);
// 			serial_writestr_P(PSTR(" / D:"));
// 			serwrite_int16(heater_d);
// 			serial_writestr_P(PSTR(" * "));
// 			serwrite_int32(d_factor);
// 			serial_writestr_P(PSTR(" = "));
// 			serwrite_int32((int32_t) heater_d * d_factor / PID_SCALE);
// 		}

		// combine factors
		int32_t pid_output_intermed = (
				(
					(((int32_t) heater_p) * p_factor) +
					(((int32_t) heater_i) * i_factor) +
					(((int32_t) heater_d) * d_factor)
				) / PID_SCALE
			);

// 		if (debug_flags & DEBUG_PID) {
// 			serial_writestr_P(PSTR(" # O: "));
// 			serwrite_int32(pid_output_intermed);
// 		}

		// rebase and limit factors
		uint8_t pid_output;
		if (pid_output_intermed > 255)
			pid_output = 255;
		else if (pid_output_intermed < 0)
			pid_output = 0;
		else
			pid_output = pid_output_intermed & 0xFF;

// 		if (debug_flags & DEBUG_PID) {
// 			serial_writestr_P(PSTR(" = "));
// 			serwrite_uint8(pid_output);
// 		}

		if (debug_flags & DEBUG_PID)
			sersendf_P(PSTR("T{E:%d, P:%d * %ld = %ld / I:%d * %ld = %ld / D:%d * %ld = %ld # O: %ld = %u}\n"), t_error, heater_p, p_factor, (int32_t) heater_p * p_factor / PID_SCALE, heater_i, i_factor, (int32_t) heater_i * i_factor / PID_SCALE, heater_d, d_factor, (int32_t) heater_d * d_factor / PID_SCALE, pid_output_intermed, pid_output);

		#ifdef	HEATER_PWM
			HEATER_PWM = pid_output;
		#else
			if (pid_output >= 8)
				enable_heater();
			else
				disable_heater();
		#endif

//  	 if (debug_flags & DEBUG_PID)
// 			serial_writestr_P(PSTR("}\n"));
	}
}
