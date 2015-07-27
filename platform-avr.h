#ifdef __AVR__

/**
  AVRs have a Harvard Architecture  which has distinct address spaces for RAM,
  Flash and EEPROM. Include macros to access these different regions.

  See /usr/lib/avr/include/avr/pgmspace.h on Linux.
*/
#include <avr/pgmspace.h>

/*
  Include appropriate header for target chip.
*/
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328__) || \
    defined (__AVR_ATmega328P__)
	#include	"arduino_168_328p.h"
#endif

#if defined (__AVR_ATmega644__) || defined (__AVR_ATmega644P__) || \
    defined (__AVR_ATmega644PA__) || defined (__AVR_ATmega1284__) || \
    defined (__AVR_ATmega1284P__)
	#include	"arduino_644.h"
#endif

#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
	#include	"arduino_1280.h"
#endif

#if defined (__AVR_AT90USB1286__)
  #include "arduino_usb1286.h"
#endif

#if defined (__AVR_AT90USB1287__)
  #include "arduino_usb1287.h"
#endif

#if defined (__AVR_ATmega32U4__)
	#include    "arduino_32U4.h"
#endif

#endif /* __AVR__ */
