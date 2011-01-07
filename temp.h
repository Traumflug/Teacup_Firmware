#ifndef	_TEMP_H
#define	_TEMP_H

#include	<stdint.h>

/*
NOTES

no point in specifying a port- all the different temp sensors we have must be on a particular port. The MAX6675 must be on the SPI, and the thermistor and AD595 must be on an analog port.

we still need to specify which analog pins we use in machine.h for the analog sensors however, otherwise the analog subsystem won't read them.
*/

#define	temp_tick temp_sensor_tick

void temp_init(void);

void temp_sensor_tick(void);

uint8_t	temp_achieved(void);

void temp_set(uint8_t index, uint16_t temperature);
uint16_t temp_get(uint8_t index);

void temp_print(uint8_t index);

uint16_t	temp_read(uint8_t index);

#endif	/* _TIMER_H */
