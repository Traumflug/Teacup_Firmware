#include	"heater.h"

/** \file
	\brief Manage heaters
*/

#include	<stdlib.h>
#include	<avr/eeprom.h>
#include	<avr/pgmspace.h>

#include	"arduino.h"
#include	"debug.h"
#include	"temp.h"
#include	"crc.h"

#ifndef	EXTRUDER
	#include	"sersendf.h"
#endif

/// \struct heater_definition_t
/// \brief simply holds pinout data- port, pin, pwm channel if used
typedef struct {
	volatile uint8_t *heater_port; ///< pointer to port. DDR is inferred from this pointer too
	uint8_t						heater_pin;  ///< heater pin, not masked. eg for PB3 enter '3' here, or PB3_PIN or similar
	volatile uint8_t *heater_pwm;  ///< pointer to 8-bit PWM register, eg OCR0A (8-bit) or ORC3L (low byte, 16-bit)
} heater_definition_t;

#undef DEFINE_HEATER
/// \brief helper macro to fill heater definition struct from config.h
// #define DEFINE_HEATER(name, port, pin, pwm) { &(port), (pin), &(pwm) },
#define	DEFINE_HEATER(name, pin) { &(pin ## _WPORT), pin ## _PIN, (pin ## _PWM) },
static const heater_definition_t heaters[NUM_HEATERS] =
{
	#include	"config.h"
};
#undef DEFINE_HEATER

/**
	\var heaters_pid
	\brief this struct holds the heater PID factors

	PID is a fascinating way to control any closed loop control, combining the error (P), cumulative error (I) and rate at which we're approacing the setpoint (D) in such a way that when correctly tuned, the system will achieve target temperature quickly and with little to no overshoot

	At every sample, we calculate \f$OUT = k_P (S - T) + k_I \int (S - T) + k_D \frac{dT}{dt}\f$ where S is setpoint and T is temperature.

	The three factors kP, kI, kD are chosen to give the desired behaviour given the dynamics of the system.

	See http://www.eetimes.com/design/embedded/4211211/PID-without-a-PhD for the full story
*/
struct {
	int32_t						p_factor; ///< scaled P factor
	int32_t						i_factor; ///< scaled I factor
	int32_t						d_factor; ///< scaled D factor
	int16_t						i_limit;  ///< scaled I limit, such that \f$-i_{limit} < i_{factor} < i_{limit}\f$
} heaters_pid[NUM_HEATERS];

/// \brief this struct holds the runtime heater data- PID integrator history, temperature history, sanity checker
struct {
	int16_t						heater_i; ///< integrator, \f$-i_{limit} < \sum{\Delta t} < i_{limit}\f$

	uint16_t					temp_history[TH_COUNT]; ///< store last TH_COUNT readings in a ring, so we can smooth out our differentiator
	uint8_t						temp_history_pointer;   ///< pointer to last entry in ring

	#ifdef	HEATER_SANITY_CHECK
		uint16_t					sanity_counter;				///< how long things haven't seemed sane
		uint16_t					sane_temperature;			///< a temperature we consider sane given the heater settings
	#endif

	uint8_t						heater_output;					///< this is the PID value we eventually send to the heater
} heaters_runtime[NUM_HEATERS];

/// default scaled P factor, equivalent to 8.0
#define		DEFAULT_P				8192
/// default scaled I factor, equivalent to 0.5
#define		DEFAULT_I				512
/// default scaled D factor, equivalent to 24
#define		DEFAULT_D				24576
/// default scaled I limit
#define		DEFAULT_I_LIMIT	384

/// this lives in the eeprom so we can save our PID settings for each heater
typedef struct {
	int32_t		EE_p_factor;
	int32_t		EE_i_factor;
	int32_t		EE_d_factor;
	int16_t		EE_i_limit;
	uint16_t	crc; ///< crc so we can use defaults if eeprom data is invalid
} EE_factor;

EE_factor EEMEM EE_factors[NUM_HEATERS];

/// \brief initialise heater subsystem
/// Set directions, initialise PWM timers, read PID factors from eeprom, etc
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
				#ifdef TCCR3A
				case (uint16_t) &OCR3AL:
					TCCR3A |= MASK(COM3A1);
					break;
				case (uint16_t) &OCR3BL:
					TCCR3A |= MASK(COM3B1);
					break;
				case (uint16_t) &OCR3CL:
					TCCR3A |= MASK(COM3C1);
					break;
				#endif
				#ifdef	TCCR4A
				case (uint16_t) &OCR4AL:
					TCCR4A |= MASK(COM4A1);
					break;
				case (uint16_t) &OCR4BL:
					TCCR4A |= MASK(COM4B1);
					break;
				case (uint16_t) &OCR4CL:
					TCCR4A |= MASK(COM4C1);
					break;
				#endif
				#ifdef	TCCR5A
				case (uint16_t) &OCR5AL:
					TCCR5A |= MASK(COM5A1);
					break;
				case (uint16_t) &OCR5BL:
					TCCR5A |= MASK(COM5B1);
					break;
				case (uint16_t) &OCR5CL:
					TCCR5A |= MASK(COM5C1);
					break;
				#endif
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

// 			if ((heaters_pid[i].p_factor == 0) && (heaters_pid[i].i_factor == 0) && (heaters_pid[i].d_factor == 0) && (heaters_pid[i].i_limit == 0)) {
			if (crc_block(&heaters_pid[i].p_factor, 14) != eeprom_read_word((uint16_t *) &EE_factors[i].crc)) {
				heaters_pid[i].p_factor = DEFAULT_P;
				heaters_pid[i].i_factor = DEFAULT_I;
				heaters_pid[i].d_factor = DEFAULT_D;
				heaters_pid[i].i_limit = DEFAULT_I_LIMIT;
			}
		#endif /* BANG_BANG */
	}
}

/// \brief Write PID factors to eeprom
void heater_save_settings() {
	#ifndef BANG_BANG
		heater_t i;
		for (i = 0; i < NUM_HEATERS; i++) {
			eeprom_write_dword((uint32_t *) &EE_factors[i].EE_p_factor, heaters_pid[i].p_factor);
			eeprom_write_dword((uint32_t *) &EE_factors[i].EE_i_factor, heaters_pid[i].i_factor);
			eeprom_write_dword((uint32_t *) &EE_factors[i].EE_d_factor, heaters_pid[i].d_factor);
			eeprom_write_word((uint16_t *) &EE_factors[i].EE_i_limit, heaters_pid[i].i_limit);
			eeprom_write_word((uint16_t *) &EE_factors[i].crc, crc_block(&heaters_pid[i].p_factor, 14));
		}
	#endif /* BANG_BANG */
}

/** \brief run heater PID algorithm
	\param h which heater we're running the loop for
	\param t which temp sensor this heater is attached to
	\param current_temp the temperature that the associated temp sensor is reporting
	\param target_temp the temperature we're trying to achieve
*/
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

		#ifdef	DEBUG
		if (debug_flags & DEBUG_PID)
			sersendf_P(PSTR("T{E:%d, P:%d * %ld = %ld / I:%d * %ld = %ld / D:%d * %ld = %ld # O: %ld = %u}\n"), t_error, heater_p, heaters_pid[h].p_factor, (int32_t) heater_p * heaters_pid[h].p_factor / PID_SCALE, heaters_runtime[h].heater_i, heaters_pid[h].i_factor, (int32_t) heaters_runtime[h].heater_i * heaters_pid[h].i_factor / PID_SCALE, heater_d, heaters_pid[h].d_factor, (int32_t) heater_d * heaters_pid[h].d_factor / PID_SCALE, pid_output_intermed, pid_output);
		#endif
	#else
		if (current_temp >= target_temp)
			pid_output = BANG_BANG_OFF;
		else
			pid_output = BANG_BANG_ON;
	#endif

	#ifdef	HEATER_SANITY_CHECK
	// check heater sanity
	// implementation is a moving window with some slow-down to compensate for thermal mass
	if (target_temp > (current_temp + (TEMP_HYSTERESIS*4))) {
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
	else if (target_temp < (current_temp - (TEMP_HYSTERESIS*4))) {
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
	if (labs((int16_t)(current_temp - heaters_runtime[h].sane_temperature)) > (TEMP_HYSTERESIS*4)) {
		// no change, or change in wrong direction for a long time- heater is broken!
		pid_output = 0;
		sersendf_P(PSTR("!! heater %d or temp sensor %d broken- temp is %d.%dC, target is %d.%dC, didn't reach %d.%dC in %d0 milliseconds\n"), h, t, current_temp >> 2, (current_temp & 3) * 25, target_temp >> 2, (target_temp & 3) * 25, heaters_runtime[h].sane_temperature >> 2, (heaters_runtime[h].sane_temperature & 3) * 25, heaters_runtime[h].sanity_counter);
	}
	#endif /* HEATER_SANITY_CHECK */

	heater_set(h, pid_output);
}

/** \brief manually set PWM output
	\param index the heater we're setting the output for
	\param value the PWM value to write

	anything done by this function is overwritten by heater_tick above if the heater has an associated temp sensor
*/
void heater_set(heater_t index, uint8_t value) {
	if (index >= NUM_HEATERS)
		return;

	heaters_runtime[index].heater_output = value;

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

/** \brief turn off all heaters

	for emergency stop
*/
uint8_t heaters_all_off() {
	uint8_t i;
	for (i = 0; i < NUM_HEATERS; i++) {
		if (heaters_runtime[i].heater_output > 0)
			return 0;
	}

	return 255;
}

/** \brief set heater P factor
	\param index heater to change factor for
	\param p scaled P factor
*/
void pid_set_p(heater_t index, int32_t p) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].p_factor = p;
	#endif /* BANG_BANG */
}

/** \brief set heater I factor
	\param index heater to change I factor for
	\param i scaled I factor
*/
void pid_set_i(heater_t index, int32_t i) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].i_factor = i;
	#endif /* BANG_BANG */
}

/** \brief set heater D factor
	\param index heater to change D factor for
	\param d scaled D factor
*/
void pid_set_d(heater_t index, int32_t d) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].d_factor = d;
	#endif /* BANG_BANG */
}

/** \brief set heater I limit
	\param index heater to set I limit for
	\param i_limit scaled I limit
*/
void pid_set_i_limit(heater_t index, int32_t i_limit) {
	#ifndef	BANG_BANG
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].i_limit = i_limit;
	#endif /* BANG_BANG */
}

#ifndef	EXTRUDER
/** \brief send heater debug info to host
	\param i index of heater to send info for
*/
void heater_print(uint16_t i) {
	sersendf_P(PSTR("P:%ld I:%ld D:%ld Ilim:%u crc:%u "), heaters_pid[i].p_factor, heaters_pid[i].i_factor, heaters_pid[i].d_factor, heaters_pid[i].i_limit, crc_block(&heaters_pid[i].p_factor, 14));
}
#endif
