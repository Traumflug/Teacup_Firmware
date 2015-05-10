#ifndef	_ANALOG_H
#define	_ANALOG_H

#include	<stdint.h>

#define	REFERENCE_AREF	0
#define	REFERENCE_AVCC	64
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
	#define	REFERENCE_1V1		192
#elif defined (__AVR_ATmega_644__) || defined (__AVR_ATmega644p__)
	#define	REFERENCE_1V1		128
	#define	REFERENCE_2V56	192
#endif

/** \def REFERENCE
  Which analog reference to use. As none of the known controllers provides a
  fixed voltage on the Aref pin and all of them have a thermistor measurement
  range of 0..5 volts, let's clamp this here to the only choice, AVcc, instead
  of confusing the user with a choice.
*/
//#include "config_wrapper.h"
#define REFERENCE REFERENCE_AVCC

#ifndef	REFERENCE
#warning	define REFERENCE as one of
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
	#warning	REFERENCE_AREF, REFERENCE_AVCC or REFERENCE_1V1
#elif defined (__AVR_ATmega_644__) || defined (__AVR_ATmega644p__)
	#warning	REFERENCE_AREF, REFERENCE_AVCC, REFERENCE_1V1 or REFERENCE_2V56
#endif
#warning	in your config.h
#error REFERENCE undefined
#endif

void 			analog_init(void);

uint16_t	analog_read(uint8_t channel);

#endif	/* _ANALOG_H */
