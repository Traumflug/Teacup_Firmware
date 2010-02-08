#ifndef	_TEMP_H
#define	_TEMP_H

#include	<stdint.h>

// extruder heater PID factors
// google "PID without a PHD" if you don't understand this PID stuff
extern int32_t p_factor;
extern int32_t i_factor;
extern int32_t d_factor;
extern int16_t i_limit;

#define		PID_SCALE			1024L
#define		I_LIMIT				4000

typedef union {
	struct {
		uint8_t				high;
		uint8_t				low;
	} buf;
	struct {
		uint16_t			dummy				:1;
		uint16_t			reading			:12;
		uint16_t			tc_open			:1;
		uint16_t			device_id		:1;
		uint16_t			tristate		:1;
	} interpret;
} max6675_data_format;

// read temperature from sensor
uint16_t temp_read(void);

// set target temperature
void temp_set(uint16_t t);

// return last read temperature
uint16_t temp_get(void);

// return target temperature
uint16_t temp_get_target(void);

// true if last read temp is close to target temp, false otherwise
uint8_t	temp_achieved(void);

// send current temperature to host
void temp_print(void);

// periodically read temperature and update heater with PID
void temp_tick(void);

#endif	/* _TIMER_H */
