#ifndef	_TEMP_H
#define	_TEMP_H

#include	"config.h"
#include	<stdint.h>

/*
NOTES

no point in specifying a port- all the different temp sensors we have must be on a particular port. The MAX6675 must be on the SPI, and the thermistor and AD595 must be on an analog port.

we still need to specify which analog pins we use in machine.h for the analog sensors however, otherwise the analog subsystem won't read them.
*/

#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) TEMP_SENSOR_ ## name,
typedef enum {
	#include "config.h"
	NUM_TEMP_SENSORS,
	TEMP_SENSOR_none
} temp_sensor_t;
#undef DEFINE_TEMP_SENSOR

typedef enum {
	TT_THERMISTOR,
	TT_MAX6675,
	TT_AD595,
	TT_PT100,
	TT_INTERCOM,
	TT_DUMMY,
} temp_type_t;

#define	temp_tick temp_sensor_tick

void temp_init(void);

void temp_sensor_tick(void);

uint8_t	temp_achieved(void);

void temp_set(temp_sensor_t index, uint16_t temperature);
uint16_t temp_get(temp_sensor_t index);

void temp_print(temp_sensor_t index);

#endif	/* _TEMP_H */
