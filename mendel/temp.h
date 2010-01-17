#ifndef	_TEMP_H
#define	_TEMP_H

#include	<stdint.h>

void temp_setup(void);
uint16_t temp_read(void);
void temp_set(uint16_t t);
void temp_tick(void);

#endif	/* _TIMER_H */
