#ifndef	_DELAY_H
#define	_DELAY_H

#include	<stdint.h>
#include 	<avr/builtins.h>
#include	"watchdog.h"
#include	<math.h>

// WAITING_DELAY is expressed in microseconds
#define		WAITING_DELAY		10000

// Force delay functions to be inlined. 
// Otherwise they will not work correctly. 
static inline void delay_us(uint16_t us) __attribute__((always_inline));
static inline void delay(uint32_t delay) __attribute__((always_inline));
static inline void delay_ms( uint16_t delay ) __attribute__((always_inline));

// Delay for a minimum of us microseconds. 
// The parameter us MUST be a compile time constant.
// A compiler error will be issued in the case that 
// it is not.

void delay_us(uint16_t us)
{
	// The floating point calculation will 
	// be completed during compilation, so
	// there is no runtime floating point
	// code generated.
    uint32_t cycles = ceil( (double)F_CPU * us / 1000000.0 );
    __builtin_avr_delay_cycles(cycles);
}

// Delay for a minimum of us microseconds.
// If the watchdog functionality is enabled
// this function will reset the timer before
// and after the delay (and at least once every
// 65536 microseconds).
//
// This function is forced (see declaration above) to be inlined.
// The parameter us MUST be a compile time constant.
// A compiler error will be issued in the case that 
// it is not.
void delay(uint32_t us)
{
    wd_reset();
    for( int i = 0; i < us/65536; i++ )
    {
        delay_us(65535);
        delay_us(1);
        wd_reset();
    }
    if( us%65536 )
    {
        delay_us(us%65536);
        wd_reset();
    }
}

// Delay for a minimum of ms milliseconds.
// If the watchdog functionality is enabled
// this function will reset the timer before
// and after the delay (and at least once every
// 65000 microseconds).
//
// This function is forced (see declaration above) to be inlined.
// The parameter us MUST be a compile time constant.
// A compiler error will be issued in the case that 
// it is not.
void delay_ms( uint16_t ms )
{
    wd_reset();
    for( uint16_t i = 0; i < ms/65; i++ )
    {
        delay_us(65000);
        wd_reset();
    }
    
    if( ms%65 )
    {
        delay_us((ms%65536)*1000);
        wd_reset();
    }
}

#endif	/* _DELAY_H */
