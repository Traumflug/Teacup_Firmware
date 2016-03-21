
/** \file
  \brief Manage heaters, including PID and PWM.

  Code for heater_init() and heater_set() is in the platform dependant include
  file and should pass six test cases when operating the heater via M106, temp
  sensors disabled:

   - PWM used on PWM-able pin, not inverted.
   - PWM pin used as on/off pin, not inverted.
   - Non-PWM-able pin, not inverted.
   - The three above, but inverted.

  In each test it should pass these tests:

   - Heater full on with M106 S255.
   - Heater full off with M106 S0.
   - Heater 10% on with M106 S25 on PWM pins.
   - Heater full off after reset, power supply turned on by other means.
   - For testing the inverted cases it's OK to check for behaving the opposite
     of the M106 command.
*/

#include "heater.h"

#define TEACUP_C_INCLUDE
#include "heater-avr.c"
#include "heater-arm_lpc11xx.c"
#undef TEACUP_C_INCLUDE

#include	<stdlib.h>
#include	"arduino.h"
#include	"debug.h"
#include	"crc.h"
#ifndef	EXTRUDER
	#include	"sersendf.h"
#endif
#ifdef EECONFIG
  #include <avr/eeprom.h>
#endif

/**
	\var heaters_pid
	\brief this struct holds the heater PID factors

	PID is a fascinating way to control any closed loop control, combining the error (P), cumulative error (I) and rate at which we're approacing the setpoint (D) in such a way that when correctly tuned, the system will achieve target temperature quickly and with little to no overshoot

	At every sample, we calculate \f$OUT = k_P (S - T) + k_I \int (S - T) + k_D \frac{dT}{dt}\f$ where S is setpoint and T is temperature.

	The three factors kP, kI, kD are chosen to give the desired behaviour given the dynamics of the system.

	See http://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD for the full story
*/
struct {
	int32_t						p_factor; ///< scaled P factor: mibicounts/qc
	int32_t						i_factor; ///< scaled I factor: mibicounts/(qC*qs)
	int32_t						d_factor; ///< scaled D factor: mibicounts/(qc/(TH_COUNT*qs))
	int16_t						i_limit;  ///< scaled I limit, such that \f$-i_{limit} < i_{factor} < i_{limit}\f$
} heaters_pid[NUM_HEATERS];

#ifdef EECONFIG
/// this lives in the eeprom so we can save our PID settings for each heater
typedef struct {
	int32_t		EE_p_factor;
	int32_t		EE_i_factor;
	int32_t		EE_d_factor;
	int16_t		EE_i_limit;
	uint16_t	crc; ///< crc so we can use defaults if eeprom data is invalid
} EE_factor;

EE_factor EEMEM EE_factors[NUM_HEATERS];
#endif /* EECONFIG */


heater_runtime_t heaters_runtime[NUM_HEATERS];

/** Inititalise PID data structures.

  \param i Index of the heater to initialise by Teacup numbering.
*/
void pid_init() {
  uint8_t i;

  for (i = 0; i < NUM_HEATERS; i++) {
    #ifdef HEATER_SANITY_CHECK
      // 0 is a "sane" temperature when we're trying to cool down.
      heaters_runtime[i].sane_temperature = 0;
    #endif

    #ifndef BANG_BANG
      #ifdef EECONFIG
        // Read factors from EEPROM.
        heaters_pid[i].p_factor =
          eeprom_read_dword((uint32_t *)&EE_factors[i].EE_p_factor);
        heaters_pid[i].i_factor =
          eeprom_read_dword((uint32_t *)&EE_factors[i].EE_i_factor);
        heaters_pid[i].d_factor =
          eeprom_read_dword((uint32_t *)&EE_factors[i].EE_d_factor);
        heaters_pid[i].i_limit =
          eeprom_read_word((uint16_t *)&EE_factors[i].EE_i_limit);

      if (crc_block(&heaters_pid[i].p_factor, 14) !=
          eeprom_read_word((uint16_t *)&EE_factors[i].crc))
      #endif /* EECONFIG */
      {
        heaters_pid[i].p_factor = DEFAULT_P;
        heaters_pid[i].i_factor = DEFAULT_I;
        heaters_pid[i].d_factor = DEFAULT_D;
        heaters_pid[i].i_limit = DEFAULT_I_LIMIT;
      }
    #endif /* BANG_BANG */
  }
}

/** \brief run heater PID algorithm
	\param h which heater we're running the loop for
	\param type which temp sensor type this heater is attached to
	\param current_temp the temperature that the associated temp sensor is reporting
	\param target_temp the temperature we're trying to achieve
*/
void heater_tick(heater_t h, temp_type_t type, uint16_t current_temp, uint16_t target_temp) {
  // Static, so it's not mandatory to calculate a new value, see BANG_BANG.
  static uint8_t pid_output;

	#ifndef	BANG_BANG
		int16_t		heater_p;
		int16_t		heater_d;
		int16_t		t_error = target_temp - current_temp;
	#endif	/* BANG_BANG */

	if (h >= NUM_HEATERS)
		return;

	if (target_temp == 0) {
		heater_set(h, 0);
		return;
	}

	#ifndef	BANG_BANG
		heaters_runtime[h].temp_history[heaters_runtime[h].temp_history_pointer++] = current_temp;
		heaters_runtime[h].temp_history_pointer &= (TH_COUNT - 1);

		// PID stuff
		// proportional
		heater_p = t_error; // Units: qC where 4qC=1C

		// integral
		heaters_runtime[h].heater_i += t_error;  // Units: qC*qs where 16qC*qs=1C*s
		// prevent integrator wind-up
		if (heaters_runtime[h].heater_i > heaters_pid[h].i_limit)
			heaters_runtime[h].heater_i = heaters_pid[h].i_limit;
		else if (heaters_runtime[h].heater_i < -heaters_pid[h].i_limit)
			heaters_runtime[h].heater_i = -heaters_pid[h].i_limit;

		// derivative.  Units: qC/(TH_COUNT*qs) where 1C/s=TH_COUNT*4qC/4qs=8qC/qs)
		// note: D follows temp rather than error so there's no large derivative when the target changes
		heater_d = heaters_runtime[h].temp_history[heaters_runtime[h].temp_history_pointer] - current_temp;

		// combine factors
		int32_t pid_output_intermed = ( // Units: counts
									   (
										(((int32_t) heater_p) * heaters_pid[h].p_factor) +
										(((int32_t) heaters_runtime[h].heater_i) * heaters_pid[h].i_factor) +
										(((int32_t) heater_d) * heaters_pid[h].d_factor)
										) / PID_SCALE
									   );

    // rebase and limit factors
    if (pid_output_intermed > 255) {
      if (t_error > 0)
        heaters_runtime[h].heater_i -= t_error; // un-integrate
      pid_output = 255;
    }
    else if (pid_output_intermed < 0) {
      if (t_error < 0)
        heaters_runtime[h].heater_i -= t_error; // un-integrate
      pid_output = 0;
    }
		else
			pid_output = pid_output_intermed & 0xFF;

		if (DEBUG_PID && (debug_flags & DEBUG_PID))
			sersendf_P(PSTR("T{E:%d, P:%d * %ld = %ld / I:%d * %ld = %ld / D:%d * %ld = %ld # O: %ld = %u}\n"), t_error, heater_p, heaters_pid[h].p_factor, (int32_t) heater_p * heaters_pid[h].p_factor / PID_SCALE, heaters_runtime[h].heater_i, heaters_pid[h].i_factor, (int32_t) heaters_runtime[h].heater_i * heaters_pid[h].i_factor / PID_SCALE, heater_d, heaters_pid[h].d_factor, (int32_t) heater_d * heaters_pid[h].d_factor / PID_SCALE, pid_output_intermed, pid_output);
	#else
    if (current_temp >= target_temp + (TEMP_HYSTERESIS))
			pid_output = BANG_BANG_OFF;
    else if (current_temp <= target_temp - (TEMP_HYSTERESIS))
			pid_output = BANG_BANG_ON;
    // else keep pid_output
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
		sersendf_P(PSTR("!! heater %d or its temp sensor broken - temp is %d.%dC, target is %d.%dC, didn't reach %d.%dC in %d0 milliseconds\n"), h, current_temp >> 2, (current_temp & 3) * 25, target_temp >> 2, (target_temp & 3) * 25, heaters_runtime[h].sane_temperature >> 2, (heaters_runtime[h].sane_temperature & 3) * 25, heaters_runtime[h].sanity_counter);
	}
	#endif /* HEATER_SANITY_CHECK */

	heater_set(h, pid_output);
}

/** \brief check whether all heaters are off
*/
uint8_t heaters_all_zero() {
  uint8_t i;

  for (i = 0; i < NUM_HEATERS; i++) {
    if (heaters_runtime[i].heater_output)
      return 0;
  }
  return 255;
}

#ifdef EECONFIG
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
#endif /* EECONFIG */

#ifndef	EXTRUDER
/** \brief send heater debug info to host
	\param i index of heater to send info for
*/
void heater_print(uint16_t i) {
	sersendf_P(PSTR("P:%ld I:%ld D:%ld Ilim:%u crc:%u\n"), heaters_pid[i].p_factor, heaters_pid[i].i_factor, heaters_pid[i].d_factor, heaters_pid[i].i_limit, crc_block(&heaters_pid[i].p_factor, 14));
}
#endif
