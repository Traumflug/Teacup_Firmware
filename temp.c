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

	note that the MAX6675 can't do more than approx 5 conversions per second- we go for 4 so the timing isn't too tight
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
#include	"heater.h"

uint16_t	current_temp = 0;
uint16_t	target_temp  = 0;

uint8_t		temp_flags		= 0;
#define		TEMP_FLAG_PRESENT		1
#define		TEMP_FLAG_TCOPEN		2

uint8_t		temp_residency	= 0;

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

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

void temp_print() {
	if (temp_flags & TEMP_FLAG_TCOPEN) {
		serial_writestr_P(PSTR("T: no thermocouple!\n"));
	}
	else {
		uint8_t c = 0, t = 0;

		c = (current_temp & 3) * 25;
		t = (target_temp & 3) * 25;
		#ifdef REPRAP_HOST_COMPATIBILITY
		sersendf_P(PSTR("T: %u.%u\n"), current_temp >> 2, c);
		#else
		sersendf_P(PSTR("T: %u.%u/%u.%u :%u\n"), current_temp >> 2, c, target_temp >> 2, t, temp_residency);
		#endif
	}
}

void temp_tick() {
	if (target_temp) {
		steptimeout = 0;

		temp_read();

		heater_tick(current_temp, target_temp);

		if (ABSDELTA(current_temp, target_temp) > TEMP_HYSTERESIS)
			temp_residency = 0;
		else if (temp_residency < TEMP_RESIDENCY_TIME)
			temp_residency++;
	}
}

uint8_t	temp_achieved() {
	if (temp_residency >= TEMP_RESIDENCY_TIME)
		return 255;
	return 0;
}
