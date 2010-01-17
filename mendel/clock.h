#ifndef	_CLOCK_H
#define	_CLOCK_H

#include	<stdint.h>

void			clock_setup(void);
uint32_t	clock_read(void);

extern volatile uint8_t	clock_flag_250ms;

#define	CLOCK_FLAG_250MS_TEMPCHECK	1

#endif	/* _CLOCK_H */
