#ifndef	_CLOCK_H
#define	_CLOCK_H

#include	<stdint.h>

void			clock_setup(void);

#ifdef	GLOBAL_CLOCK
uint32_t	clock_read(void);
#endif

extern volatile uint8_t	clock_flag_250ms;

#define	CLOCK_FLAG_250MS_TEMPCHECK	1
#define	CLOCK_FLAG_250MS_REPORT			2

#endif	/* _CLOCK_H */
