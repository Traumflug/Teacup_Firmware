/*
	temp.c

	This file currently reads temp from a MAX6675 on the SPI bus.

	temp fields are 14.2 fixed point, so temp_set(500) will set the temperature to 125 celsius, and temp_get() = 600 is reporting a temperature of 150 celsius.

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

#ifdef	TEMP_MAX6675
#endif

#ifdef	TEMP_THERMISTOR
#include	"analog.h"

#define NUMTEMPS 20
uint16_t temptable[NUMTEMPS][2] PROGMEM = {
	{1, 841},
	{54, 255},
	{107, 209},
	{160, 184},
	{213, 166},
	{266, 153},
	{319, 142},
	{372, 132},
	{425, 124},
	{478, 116},
	{531, 108},
	{584, 101},
	{637, 93},
	{690, 86},
	{743, 78},
	{796, 70},
	{849, 61},
	{902, 50},
	{955, 34},
	{1008, 3}
};
#endif

#ifdef	TEMP_AD595
#include	"analog.h"
#endif

#ifndef	TEMP_MAX6675
	#ifndef	TEMP_THERMISTOR
		#ifndef	TEMP_AD595
			#error none of TEMP_MAX6675, TEMP_THERMISTOR or TEMP_AD595 are defined! What type of temp sensor are you using?
		#endif
	#endif
#endif

uint16_t	current_temp = 0;
uint16_t	target_temp  = 0;

uint8_t		temp_flags		= 0;
uint8_t		temp_residency	= 0;

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

uint16_t temp_read() {
	uint16_t temp;

#ifdef	TEMP_MAX6675
	#ifdef	PRR
		PRR &= ~MASK(PRSPI);
	#elif defined PRR0
		PRR0 &= ~MASK(PRSPI);
	#endif

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
#endif	/* TEMP_MAX6675	*/

#ifdef	TEMP_THERMISTOR
	uint8_t i;

	//Read current temperature
	temp = analog_read(TEMP_PIN_CHANNEL);

	//Calculate real temperature based on lookup table
	for (i = 1; i < NUMTEMPS; i++) {
		if (pgm_read_word(&(temptable[i][0])) > temp) {
			// multiply by 4 because internal temp is stored as 14.2 fixed point
			temp = pgm_read_word(&(temptable[i][1])) + (pgm_read_word(&(temptable[i][0])) - temp) * 4 * (pgm_read_word(&(temptable[i-1][1])) - pgm_read_word(&(temptable[i][1]))) / (pgm_read_word(&(temptable[i][0])) - pgm_read_word(&(temptable[i-1][0])));
			break;
		}
	}

	//Clamp for overflows
	if (i == NUMTEMPS)
		temp = temptable[NUMTEMPS-1][1];

	return temp;

#endif	/* TEMP_THERMISTOR */

#ifdef	TEMP_AD595
	temp = analog_read(TEMP_PIN_CHANNEL);

	// convert
	// >>8 instead of >>10 because internal temp is stored as 14.2 fixed point
	temp = (temp * 500L) >> 8;
	
	return temp;
#endif	/* TEMP_AD595 */

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
