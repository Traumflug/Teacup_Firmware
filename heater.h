#ifndef	_HEATER_H
#define	_HEATER_H

#include "config_wrapper.h"
#include	<stdint.h>
#include "simulator.h"
#include "temp.h"

#undef DEFINE_HEATER
#define DEFINE_HEATER(name, pin, pwm) HEATER_ ## name,
typedef enum
{
	#include "config_wrapper.h"
	NUM_HEATERS,
	HEATER_noheater
} heater_t;
#undef DEFINE_HEATER

void heater_init(void);

void heater_set(heater_t index, uint8_t value);
void heater_tick(heater_t h, temp_type_t type, uint16_t current_temp, uint16_t target_temp);

uint8_t heaters_all_zero(void);
uint8_t heaters_all_off(void);

#ifdef EECONFIG
void pid_set_p(heater_t index, int32_t p);
void pid_set_i(heater_t index, int32_t i);
void pid_set_d(heater_t index, int32_t d);
void pid_set_i_limit(heater_t index, int32_t i_limit);
void heater_save_settings(void);
#endif /* EECONFIG */

void heater_print(uint16_t i);

#endif	/* _HEATER_H */
