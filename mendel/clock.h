#ifndef	_CLOCK_H
#define	_CLOCK_H

#include	<stdint.h>

void			clock_setup(void);

#ifdef	GLOBAL_CLOCK
uint32_t	clock_read(void);
#endif

extern volatile uint8_t	clock_flag_250ms;

#define	CLOCK_FLAG_250MS							1
// #define	CLOCK_FLAG_250MS_TEMPCHECK		1
// #define	CLOCK_FLAG_250MS_REPORT				2
// #define	CLOCK_FLAG_250MS_STEPTIMEOUT	4

/*
	ifclock() {}

	so we can do stuff like:
	ifclock(CLOCK_FLAG_250MS_REPORT) {
		report();
	}

	or:
	ifclock(CLOCK_FLAG_250MS_STEPTIMEOUT)
		disable_steppers();
*/
#define	ifclock(F)	for (;clock_flag_250ms & (F);clock_flag_250ms &= ~(F))

#endif	/* _CLOCK_H */
