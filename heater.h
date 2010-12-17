#ifndef	_HEATER_H
#define	_HEATER_H

#include	<stdint.h>

#define	enable_heater()		heater_set(0, 64)
#define	disable_heater()	heater_set(0, 0)

void heater_init(void);
void heater_save_settings(void);

void heater_set(uint8_t index, uint8_t value);
void heater_tick(uint8_t h, uint8_t t, uint16_t current_temp, uint16_t target_temp);

void pid_set_p(uint8_t index, int32_t p);
void pid_set_i(uint8_t index, int32_t i);
void pid_set_d(uint8_t index, int32_t d);
void pid_set_i_limit(uint8_t index, int32_t i_limit);

#endif	/* _HEATER_H */
