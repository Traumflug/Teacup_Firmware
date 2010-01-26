#ifndef	_TEMP_H
#define	_TEMP_H

#include	<stdint.h>

uint16_t temp_read(void);
void temp_set(uint16_t t);
uint16_t temp_get(void);
void temp_print(void);
void temp_tick(void);

#endif	/* _TIMER_H */
