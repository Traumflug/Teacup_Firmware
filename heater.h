#ifndef	_HEATER_H
#define	_HEATER_H

#include "config_wrapper.h"
#include	<stdint.h>
#include "simulator.h"
#include "temp.h"

/// Default scaled P factor, equivalent to 8.0 counts/qC or 32 counts/C.
#define DEFAULT_P         8192
/// Default scaled I factor, equivalent to 0.5 counts/(qC*qs) or 8 counts/C*s.
#define DEFAULT_I         512
/// Default scaled D factor, equivalent to 24 counts/(qc/(TH_COUNT*qs)) or
/// 192 counts/(C/s).
#define DEFAULT_D         24576
/// Default scaled I limit, equivalent to 384 qC*qs, or 24 C*s.
#define DEFAULT_I_LIMIT   384

/** \def HEATER_THRESHOLD

  Defines the threshold when to turn a non-PWM heater on and when to turn it
  off. Applies only to heaters which have only two states, on and off.
  Opposed to those heaters which allow to turn them on gradually as needed,
  usually by using PWM.
*/
#ifdef BANG_BANG
  #define HEATER_THRESHOLD ((BANG_BANG_ON + BANG_BANG_OFF) / 2)
#else
  #define HEATER_THRESHOLD 8
#endif


#undef DEFINE_HEATER
#define DEFINE_HEATER(name, pin, invert, pwm) HEATER_ ## name,
typedef enum
{
	#include "config_wrapper.h"
	NUM_HEATERS,
	HEATER_noheater
} heater_t;
#undef DEFINE_HEATER

/** This struct holds the runtime heater data.

  PID integrator history, temperature history, sanity checker.
*/
typedef struct {
  /// Integrator, \f$-i_{limit} < \sum{4*eC*\Delta t} < i_{limit}\f$
  int16_t heater_i;

  /// Store last TH_COUNT readings in a ring, so we can smooth out our
  /// differentiator.
  uint16_t temp_history[TH_COUNT];
  /// Pointer to last entry in ring.
  uint8_t temp_history_pointer;

  #ifdef HEATER_SANITY_CHECK
    /// How long things haven't seemed sane.
    uint16_t sanity_counter;
    /// A temperature we consider sane given the heater settings.
    uint16_t sane_temperature;
  #endif

  /// This is the PID value we eventually send to the heater.
  uint8_t heater_output;
} heater_runtime_t;


extern heater_runtime_t heaters_runtime[];

void heater_init(void);
void pid_init(void);

void heater_set(heater_t index, uint8_t value);
void heater_tick(heater_t h, temp_type_t type, uint16_t current_temp, uint16_t target_temp);

uint8_t heaters_all_zero(void);

#ifdef EECONFIG
void pid_set_p(heater_t index, int32_t p);
void pid_set_i(heater_t index, int32_t i);
void pid_set_d(heater_t index, int32_t d);
void pid_set_i_limit(heater_t index, int32_t i_limit);
void heater_save_settings(void);
#endif /* EECONFIG */

void heater_print(uint16_t i);

#endif	/* _HEATER_H */
