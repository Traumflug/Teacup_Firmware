#ifndef	_HEATER_H
#define	_HEATER_H

#include	<stdint.h>

// extruder heater PID factors
// google "PID without a PHD" if you don't understand this PID stuff
extern int32_t p_factor;
extern int32_t i_factor;
extern int32_t d_factor;
extern int16_t i_limit;

#define		PID_SCALE			1024L
#define		I_LIMIT				4000

void heater_init(void);
void heater_save_settings(void);
void heater_tick(int16_t current_temp, int16_t target_temp);
uint8_t	temp_achieved(void);

#endif	/* _HEATER_H */
