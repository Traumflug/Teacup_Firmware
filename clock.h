#ifndef	_CLOCK_H
#define	_CLOCK_H

#include <stdint.h>


extern volatile uint8_t clock_flag_10ms;
extern volatile uint8_t clock_flag_250ms;
extern volatile uint8_t clock_flag_1s;


// If the specific bit is set, execute the following block exactly once
// and then clear the flag.
#define ifclock(F) for ( ; F; F = 0)

// Should be called every TICK_TIME (currently 2 ms).
void clock_tick(void);

void clock(void);

#endif	/* _CLOCK_H */
