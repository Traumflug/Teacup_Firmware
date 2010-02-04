#ifndef	_CLOCK_H
#define	_CLOCK_H

#include	<stdint.h>

void			clock_setup(void);

#ifdef	GLOBAL_CLOCK
uint32_t	clock_read(void);
#endif

extern volatile uint8_t	clock_flag;

#define	CLOCK_FLAG_250MS							1
#define	CLOCK_FLAG_1S									2

/*
	ifclock() {}

	so we can do stuff like:
	ifclock(CLOCK_FLAG_250MS) {
		report();
	}

	or:
	ifclock(CLOCK_FLAG_1S)
		disable_steppers();
*/

#define	ifclock(F)	for (;clock_flag & (F);clock_flag &= ~(F))

#endif	/* _CLOCK_H */
