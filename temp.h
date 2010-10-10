#ifndef	_TEMP_H
#define	_TEMP_H

#include	<stdint.h>

#include	"config.h"

#define		TEMP_FLAG_PRESENT		1
#define		TEMP_FLAG_TCOPEN		2

#ifdef	TEMP_MAX6675
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
#endif

#ifdef	TEMP_THERMISTOR
#include	<avr/pgmspace.h>
#endif

#ifdef	TEMP_AD595
#endif

// setup temperature system
void temp_init(void);

// save PID factors to EEPROM
void temp_save_settings(void);

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
