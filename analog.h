#ifndef	_ANALOG_H
#define	_ANALOG_H

#include	<stdint.h>

#ifdef __AVR__
  // TODO: these reference selectors should go away. A nice feature, but
  //       none of the RepRap controllers has use for it.

/** \def REFERENCE_AREF
  This compares the voltage to be measured against the voltage on the Aref pin.
  This also requires a voltage to be actually provided on the Aref pin, which
  none of the commonly available controllers or Arduinos do.
*/
#define	REFERENCE_AREF	0

/** \def REFERENCE_AVCC
  This compares the voltage to be measured against the voltage on the Aref pin,
  but also connects AVcc to Aref, so no external voltage is required. Using
  this is said to be more accurate than doing this connection externally, on
  the pins of the chip, so this is the most commonly used option.
*/
#define	REFERENCE_AVCC	64

/** \def REFERENCE_1V1
    \def REFERENCE_2V56
  These compare the voltage to be measured against internally created
  1.1 or 2.56 volts. Not useful on commonly available RepRap controllers,
  but might be a good choice for custom devices providing only a low voltage.
*/
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega168P__) || \
    defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
	#define	REFERENCE_1V1		192
#else
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
  #error REFERENCE undefined. See analog.h on how to choose it.
#endif

#endif /* __AVR__ */

void 			analog_init(void);

uint16_t	analog_read(uint8_t index);

#endif	/* _ANALOG_H */
