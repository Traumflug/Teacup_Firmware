#ifndef	_HEATER_H
#define	_HEATER_H

#include "config_wrapper.h"
#include	<stdint.h>
#include "temp.h"

#define	enable_heater()		heater_set(0, 64)
#define	disable_heater()	heater_set(0, 0)

#undef DEFINE_HEATER
#define DEFINE_HEATER(name, pin) HEATER_ ## name,
typedef enum
{
	#include "config_wrapper.h"
	NUM_HEATERS,
	HEATER_noheater
} heater_t;
#undef DEFINE_HEATER

void heater_init(void);
void heater_save_settings(void);

void heater_set(heater_t index, uint8_t value);
void heater_tick(heater_t h, temp_sensor_t t, uint16_t current_temp, uint16_t target_temp);

uint8_t heaters_all_off(void);

void pid_set_p(heater_t index, int32_t p);
void pid_set_i(heater_t index, int32_t i);
void pid_set_d(heater_t index, int32_t d);
void pid_set_i_limit(heater_t index, int32_t i_limit);

void heater_print(uint16_t i);

#endif	/* _HEATER_H */
