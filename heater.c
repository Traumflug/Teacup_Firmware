
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
#include "heater-arm.c"
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

    #ifdef PID
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
    #endif /* PID */
  }
  #undef DEFINE_PID_VALUE
  #define DEFINE_PID_VALUE(name, kp, ki, kd, ilimit)  \
    heaters_pid[HEATER_ ## name].p_factor = kp; \
    heaters_pid[HEATER_ ## name].i_factor = ki; \
    heaters_pid[HEATER_ ## name].d_factor = kd; \
    heaters_pid[HEATER_ ## name].i_limit = ilimit;
  #include "config_wrapper.h"
  #undef DEFINE_PID_VALU
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

	#ifdef	PID
		int16_t		heater_p;
		int16_t		heater_d;
		int16_t		t_error = target_temp - current_temp;
	#endif	/* PID */

	if (h >= NUM_HEATERS)
		return;

	if (target_temp == 0) {
		heater_set(h, 0);
    #ifdef PID_AUTOTUNE
      heaters_runtime[h].autotune_stop = 0;
    #endif
		return;
	}

	#ifdef	PID
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
	#elif defined(BANG_BANG)
    if (current_temp >= target_temp + (TEMP_HYSTERESIS))
			pid_output = BANG_BANG_OFF;
    else if (current_temp <= target_temp - (TEMP_HYSTERESIS))
			pid_output = BANG_BANG_ON;
    // else keep pid_output
  #elif defined(PID_AUTOTUNE)
    int16_t pid_output_tune = pid_autotune(h, current_temp, target_temp);
    if (pid_output_tune >= 0)
      pid_output = (uint8_t)pid_output_tune;
  #else
    // there is no else
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
	#ifdef PID
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].p_factor = p;
	#endif /* PID */
}

/** \brief set heater I factor
	\param index heater to change I factor for
	\param i scaled I factor
*/
void pid_set_i(heater_t index, int32_t i) {
	#ifdef PID
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].i_factor = i;
	#endif /* PID */
}

/** \brief set heater D factor
	\param index heater to change D factor for
	\param d scaled D factor
*/
void pid_set_d(heater_t index, int32_t d) {
	#ifdef PID
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].d_factor = d;
	#endif /* PID */
}

/** \brief set heater I limit
	\param index heater to set I limit for
	\param i_limit scaled I limit
*/
void pid_set_i_limit(heater_t index, int32_t i_limit) {
	#ifdef PID
		if (index >= NUM_HEATERS)
			return;

		heaters_pid[index].i_limit = i_limit;
	#endif /* PID */
}

/// \brief Write PID factors to eeprom
void heater_save_settings() {
  #ifdef PID
    heater_t i;
    for (i = 0; i < NUM_HEATERS; i++) {
      eeprom_write_dword((uint32_t *) &EE_factors[i].EE_p_factor, heaters_pid[i].p_factor);
      eeprom_write_dword((uint32_t *) &EE_factors[i].EE_i_factor, heaters_pid[i].i_factor);
      eeprom_write_dword((uint32_t *) &EE_factors[i].EE_d_factor, heaters_pid[i].d_factor);
      eeprom_write_word((uint16_t *) &EE_factors[i].EE_i_limit, heaters_pid[i].i_limit);
      eeprom_write_word((uint16_t *) &EE_factors[i].crc, crc_block(&heaters_pid[i].p_factor, 14));
    }
  #endif /* PID */
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

#ifdef PID_AUTOTUNE

#ifndef MAX
  #define MAX(a,b)  (((a)>(b))?(a):(b))
  #define MIN(a,b)  (((a)<(b))?(a):(b))
#endif

#define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

int16_t pid_autotune(heater_t h, uint16_t current_temp, uint16_t target_temp) {
  int16_t pid_output = -1;  // -1 is used as no update of pid_output
  
  /** This is our time
    We count this up any time we go in here. While this runs in
    heater_tick this is +10ms.
  */
  heaters_runtime[h].tune_counter_10ms++;
  if (!heaters_runtime[h].autotune_stop) {
    // Init all values when starting
    if (!heaters_runtime[h].autotune_active) {
      // sersendf_P(PSTR("start autotune\n"));
      heaters_runtime[h].tune_counter_10ms = 0;

      heaters_runtime[h].autotune_active = 1;
      heaters_runtime[h].cycles = 0;
      heaters_runtime[h].heating = 1;

      heaters_runtime[h].temp_10ms = 0;
      heaters_runtime[h].t1 = 0;
      heaters_runtime[h].t2 = 0;
      heaters_runtime[h].t_high = 0;

      #ifndef PID_MAX
        #define PID_MAX 80
      #endif
      heaters_runtime[h].bias = PID_MAX >> 1;
      heaters_runtime[h].d = PID_MAX >> 1;

      heaters_runtime[h].max_temp = 80;  // 20°C
      heaters_runtime[h].min_temp = 80;  // 20°C

      pid_output = PID_MAX;
    } 
    else {
      heaters_runtime[h].max_temp = MAX(heaters_runtime[h].max_temp, current_temp);
      heaters_runtime[h].min_temp = MIN(heaters_runtime[h].min_temp, current_temp);

      if (heaters_runtime[h].heating && current_temp > target_temp) {
        if (heaters_runtime[h].tune_counter_10ms - heaters_runtime[h].t2 > 150) {
          heaters_runtime[h].heating = 0;
          pid_output = (heaters_runtime[h].bias - heaters_runtime[h].d);
          heaters_runtime[h].t1 = heaters_runtime[h].tune_counter_10ms;
          heaters_runtime[h].t_high = heaters_runtime[h].t1 - heaters_runtime[h].t2;
          heaters_runtime[h].max_temp = target_temp;
        }
      }

      if (!heaters_runtime[h].heating && current_temp < target_temp) {
        if (heaters_runtime[h].tune_counter_10ms - heaters_runtime[h].t1 > 300) {
          heaters_runtime[h].heating = 1;
          heaters_runtime[h].t2 = heaters_runtime[h].tune_counter_10ms;
          int32_t t_low = heaters_runtime[h].t2 - heaters_runtime[h].t1;
          if (heaters_runtime[h].cycles > 0) {
            heaters_runtime[h].bias += (heaters_runtime[h].d * (heaters_runtime[h].t_high - t_low))/(t_low + heaters_runtime[h].t_high);
            heaters_runtime[h].bias = CONSTRAIN(heaters_runtime[h].bias, 20, PID_MAX - 20);
            if (heaters_runtime[h].bias > PID_MAX/2) heaters_runtime[h].d = PID_MAX - 1 - heaters_runtime[h].bias;
            else heaters_runtime[h].d = heaters_runtime[h].bias;

            if (DEBUG_AUTOTUNE && (debug_flags & DEBUG_AUTOTUNE)) {
              sersendf_P(PSTR("Bias: %ld\n"), heaters_runtime[h].bias);
              sersendf_P(PSTR("d: %ld\n"), heaters_runtime[h].d);
              sersendf_P(PSTR("min_temp: %u.%u\n"), heaters_runtime[h].min_temp >> 2, (heaters_runtime[h].min_temp & 0x3) * 25);
              sersendf_P(PSTR("max_temp: %u.%u\n"), heaters_runtime[h].max_temp >> 2, (heaters_runtime[h].max_temp & 0x3) * 25);
            }
            if (heaters_runtime[h].cycles > 2) {
              uint16_t Ku = (uint16_t)(((uint32_t)heaters_runtime[h].d << 17) / (201 * ((heaters_runtime[h].max_temp - heaters_runtime[h].min_temp))));
              /** Ku is 9.7 fixpoint
                Ku = (4 * d) / ( pi *(max_temp - min_temp))
                4 = 1 << 2
                pi ~ 201 / 1 << 6
                max and min_temp are .2 fixpoint ( 1 << 2)
                2 + 6 + 2 = 10
                17 - 10 = .7 fixpoint
              */
              uint16_t Tu = ((t_low + heaters_runtime[h].t_high) * 41) >> 5;
              /** Tu is 9.7 fixpoint.
                Tu = t_low + t_high (in seconds)
                t_low and t_high are in 10ms-ticks
                t_x * 10 = 1ms-ticks
                t_x * 10 / 1000 = 1s-ticks
                10 / 1000 ~ 41 / 2^12
                with >> 5 we have now 9.7 fixpoint
              */
              sersendf_P(PSTR("Ku: %u, Tu: %u\n"), Ku, Tu);
            }
          }
        }
        pid_output = (heaters_runtime[h].bias + heaters_runtime[h].d);
        heaters_runtime[h].cycles++;
        heaters_runtime[h].min_temp = target_temp;
      }

      #ifndef PID_CYCLES
        #define PID_CYCLES 5
      #endif
      if (heaters_runtime[h].cycles > PID_CYCLES) {
        heaters_runtime[h].autotune_active = 0;
        heaters_runtime[h].autotune_stop = 1;
        return 0;
      }
    }
  }
  return pid_output;
}
#endif
