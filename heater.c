#include	"heater.h"

#include	<stdlib.h>
#include	<avr/eeprom.h>
#include	<avr/pgmspace.h>

#include	"arduino.h"
#include	"debug.h"
#ifndef	EXTRUDER
#include	"sersendf.h"
#endif
#include	"temp.h"

typedef struct {
	volatile uint8_t *heater_port;
	uint8_t						heater_pin;
	volatile uint8_t *heater_pwm;
} heater_definition_t;

#undef DEFINE_HEATER
#define DEFINE_HEATER(name, port, pin, pwm) { &(port), (pin), &(pwm) },
static const heater_definition_t heaters[NUM_HEATERS] =
{
	#include	"config.h"
};
#undef DEFINE_HEATER

// this struct holds the heater PID factors that are stored in the EEPROM during poweroff
struct {
	int32_t						p_factor;
	int32_t						i_factor;
	int32_t						d_factor;
	int16_t						i_limit;
} heaters_pid[NUM_HEATERS];

// this struct holds the runtime heater data- PID integrator history, temperature history, sanity checker
struct {
	int16_t						heater_i;

	uint16_t					temp_history[TH_COUNT];
	uint8_t						temp_history_pointer;

	#ifdef	HEATER_SANITY_CHECK
		uint16_t					sanity_counter;
		uint16_t					sane_temperature;
	#endif
} heaters_runtime[NUM_HEATERS];

#define		DEFAULT_P				8192
#define		DEFAULT_I				512
#define		DEFAULT_D				24576
#define		DEFAULT_I_LIMIT	384

// this lives in the eeprom so we can save our PID settings for each heater
typedef struct {
	int32_t		EE_p_factor;
	int32_t		EE_i_factor;
	int32_t		EE_d_factor;
	int16_t		EE_i_limit;
} EE_factor;

EE_factor EEMEM EE_factors[NUM_HEATERS];

void heater_init() {
	heater_t i;
	// setup pins
	for (i = 0; i < NUM_HEATERS; i++) {
		*(heaters[i].heater_port) &= ~MASK(heaters[i].heater_pin);
		// DDR is always 1 address below PORT. ugly code but saves ram and an extra field in heaters[] which will never be used anywhere but here
		*(heaters[i].heater_port - 1) |= MASK(heaters[i].heater_pin);
		if (heaters[i].heater_pwm) {
			*heaters[i].heater_pwm = 0;
			// this is somewhat ugly too, but switch() won't accept pointers for reasons unknown
			switch((uint16_t) heaters[i].heater_pwm) {
				case (uint16_t) &OCR0A:
					TCCR0A |= MASK(COM0A1);
					break;
				case (uint16_t) &OCR0B:
					TCCR0A |= MASK(COM0B1);
					break;
				case (uint16_t) &OCR2A:
					TCCR2A |= MASK(COM2A1);
					break;
				case (uint16_t) &OCR2B:
					TCCR2A |= MASK(COM2B1);
					break;
			}
		}

		#ifdef	HEATER_SANITY_CHECK
			// 0 is a "sane" temperature when we're trying to cool down
			heaters_runtime[i].sane_temperature = 0;
		#endif

		#ifndef BANG_BANG
			// read factors from eeprom
			heaters_pid[i].p_factor = eeprom_read_dword((uint32_t *) &EE_factors[i].EE_p_factor);
			heaters_pid[i].i_factor = eeprom_read_dword((uint32_t *) &EE_factors[i].EE_i_factor);
			heaters_pid[i].d_factor = eeprom_read_dword((uint32_t *) &EE_factors[i].EE_d_factor);
			heaters_pid[i].i_limit = eeprom_read_word((uint16_t *) &EE_factors[i].EE_i_limit);

			if ((heaters_pid[i].p_factor == 0) && (heaters_pid[i].i_factor == 0) && (heaters_pid[i].d_factor == 0) && (heaters_pid[i].i_limit == 0)) {
				heaters_pid[i].p_factor = DEFAULT_P;
				heaters_pid[i].i_factor = DEFAULT_I;
				heaters_pid[i].d_factor = DEFAULT_D;
				heaters_pid[i].i_limit = DEFAULT_I_LIMIT;
			}
		#endif /* BANG_BANG */
	}
}

void heater_save_settings() {
	#ifndef BANG_BANG
		heater_t i;
		for (i = 0; i < NUM_HEATERS; i++) {
			eeprom_write_dword((uint32_t *) &EE_factors[i].EE_p_factor, heaters_pid[i].p_factor);
			eeprom_write_dword((uint32_t *) &EE_factors[i].EE_i_factor, heaters_pid[i].i_factor);
			eeprom_write_dword((uint32_t *) &EE_factors[i].EE_d_factor, heaters_pid[i].d_factor);
			eeprom_write_word((uint16_t *) &EE_factors[i].EE_i_limit, heaters_pid[i].i_limit);
		}
	#endif /* BANG_BANG */
}

void heater_tick(heater_t h, temp_sensor_t t, uint16_t current_temp, uint16_t target_temp) {
	uint8_t		pid_output;

	#ifndef	BANG_BANG
		int16_t		heater_p;
		int16_t		heater_d;
		int16_t		t_error = target_temp - current_temp;
	#endif	/* BANG_BANG */

	if (h >= NUM_HEATERS || t >= NUM_TEMP_SENSORS)
		return;

	#ifndef	BANG_BANG
		heaters_runtime[h].temp_history[heaters_runtime[h].temp_history_pointer++] = current_temp;
		heaters_runtime[h].temp_history_pointer &= (TH_COUNT - 1);

		// PID stuff
		// proportional
		heater_p = t_error;

		// integral
		heaters_runtime[h].heater_i += t_error;
		// prevent integrator wind-up
		if (heaters_runtime[h].heater_i > heaters_pid[h].i_limit)
			heaters_runtime[h].heater_i = heaters_pid[h].i_limit;
		else if (heaters_runtime[h].heater_i < -heaters_pid[h].i_limit)
			heaters_runtime[h].heater_i = -heaters_pid[h].i_limit;

		// derivative
		// note: D follows temp rather than error so there's no large derivative when the target changes
		heater_d = heaters_runtime[h].temp_history[heaters_runtime[h].temp_history_pointer] - current_temp;

		// combine factors
		int32_t pid_output_intermed = (
			(
				(((int32_t) heater_p) * heaters_pid[h].p_factor) +
				(((int32_t) heaters_runtime[h].heater_i) * heaters_pid[h].i_factor) +
				(((int32_t) heater_d) * heaters_pid[h].d_factor)
			) / PID_SCALE
		);

		// rebase and limit factors
		if (pid_output_intermed > 255)
			pid_output = 255;
		else if (pid_output_intermed < 0)
			pid_output = 0;
		else
			pid_output = pid_output_intermed & 0xFF;
	#else
		if (current_temp >= target_temp)
			pid_output = BANG_BANG_ON;
		else
			pid_output = BANG_BANG_OFF;
	#endif
	
	#ifdef	DEBUG
	if (debug_flags & DEBUG_PID)
		sersendf_P(PSTR("T{E:%d, P:%d * %ld = %ld / I:%d * %ld = %ld / D:%d * %ld = %ld # O: %ld = %u}\n"), t_error, heater_p, heaters_pid[h].p_factor, (int32_t) heater_p * heaters_pid[h].p_factor / PID_SCALE, heaters_runtime[h].heater_i, heaters_pid[h].i_factor, (int32_t) heaters_runtime[h].heater_i * heaters_pid[h].i_factor / PID_SCALE, heater_d, heaters_pid[h].d_factor, (int32_t) heater_d * heaters_pid[h].d_factor / PID_SCALE, pid_output_intermed, pid_output);
	#endif

	#ifdef	HEATER_SANITY_CHECK
	// check heater sanity
	// implementation is a moving window with some slow-down to compensate for thermal mass
	if (target_temp > (current_temp + TEMP_HYSTERESIS)) {
		// heating
		if (current_temp > heaters_runtime[h].sane_temperature)
			// hotter than sane- good since we're heating unless too hot
			heaters_runtime[h].sane_temperature = current_temp;
		else {
			if (heaters_runtime[h].sanity_counter < 40)
				heaters_runtime[h].sanity_counter++;
			else {
				heaters_runtime[h].sanity_counter = 0;
				// ratchet up expected temp
				heaters_runtime[h].sane_temperature++;
			}
		}
		// limit to target, so if we overshoot by too much for too long an error is flagged
		if (heaters_runtime[h].sane_temperature > target_temp)
			heaters_runtime[h].sane_temperature = target_temp;
	}
	else if (target_temp < (current_temp - TEMP_HYSTERESIS)) {
		// cooling
		if (current_temp < heaters_runtime[h].sane_temperature)
			// cooler than sane- good since we're cooling
			heaters_runtime[h].sane_temperature = current_temp;
		else {
			if (heaters_runtime[h].sanity_counter < 125)
				heaters_runtime[h].sanity_counter++;
			else {
				heaters_runtime[h].sanity_counter = 0;
				// ratchet down expected temp
				heaters_runtime[h].sane_temperature--;
			}
		}
		// if we're at or below 60 celsius, don't freak out if we can't drop any more.
		if (current_temp <= 240)
			heaters_runtime[h].sane_temperature = current_temp;
		// limit to target, so if we don't cool down for too long an error is flagged
		else if (heaters_runtime[h].sane_temperature < target_temp)
			heaters_runtime[h].sane_temperature = target_temp;
	}
	// we're within HYSTERESIS of our target
	else {
		heaters_runtime[h].sane_temperature = current_temp;
		heaters_runtime[h].sanity_counter = 0;
	}

	// compare where we're at to where we should be
	if (labs(current_temp - heaters_runtime[h].sane_temperature) > TEMP_HYSTERESIS) {
		// no change, or change in wrong direction for a long time- heater is broken!
		pid_output = 0;
		sersendf_P(PSTR("!! heater %d or temp sensor %d broken- temp is %d.%dC, target is %d.%dC, didn't reach %d.%dC in %d0 milliseconds\n"), h, t, current_temp >> 2, (current_temp & 3) * 25, target_temp >> 2, (target_temp & 3) * 25, heaters_runtime[h].sane_temperature >> 2, (heaters_runtime[h].sane_temperature & 3) * 25, heaters_runtime[h].sanity_counter);
	}
	#endif /* HEATER_SANITY_CHECK */

	heater_set(h, pid_output);
}

void heater_set(heater_t index, uint8_t value) {
	if (index >= NUM_HEATERS)
		return;

	if (heaters[index].heater_pwm) {
		*(heaters[index].heater_pwm) = value;
		#ifdef	DEBUG
		if (debug_flags & DEBUG_PID)
			sersendf_P(PSTR("PWM{%u = %u}\n"), index, OCR0A);
		#endif
	}
	else {
		if (value >= 8)
			*(heaters[index].heater_port) |= MASK(heaters[index].heater_pin);
		else
			*(heaters[index].heater_port) &= ~MASK(heaters[index].heater_pin);
	}
}

void pid_set_p(heater_t index, int32_t p) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].p_factor = p;
	#endif /* BANG_BANG */
}

void pid_set_i(heater_t index, int32_t i) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].i_factor = i;
	#endif /* BANG_BANG */
}

void pid_set_d(heater_t index, int32_t d) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].d_factor = d;
	#endif /* BANG_BANG */
}

void pid_set_i_limit(heater_t index, int32_t i_limit) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].i_limit = i_limit;
	#endif /* BANG_BANG */
}
