
/** \file
  \brief Pin definitions supervisor.

  Here we map to the pin definition file of the architecture at hand and also
  do some fundamental platform related stuff.
*/

#ifndef	_ARDUINO_H
#define	_ARDUINO_H


/**
  Only AVRs have a Harvard Architecture, which has distinct address spaces
  for RAM, Flash and EEPROM. All other supported targets have a single address
  space, so all the macros PROGMEM, PSTR() & co. are obsolete. Define them to
  do nothing.

  For the AVR definitions, see /usr/lib/avr/include/avr/pgmspace.h on Linux.
*/
#ifdef __AVR__

  #include <avr/pgmspace.h>

#else

  #define PROGMEM
  #define PGM_P const char *
  #define PSTR(s) ((const PROGMEM char *)(s))
  #define pgm_read_byte(x) (*((uint8_t *)(x)))
  #define pgm_read_word(x) (*((uint16_t *)(x)))
  #define pgm_read_dword(x) (*((uint32_t *)(x)))

#endif /* __AVR__, ! __AVR__ */

/*
	ports and functions

	added as necessary or if I feel like it- not a comprehensive list!
*/
#if defined __AVR__

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

#elif defined __ARMEL__

  #if defined (__ARM_LPC1114__)
    #include "arduino_lpc1114.h"
  #endif

#endif /* __AVR__, __ARMEL__, SIMULATOR */

#ifndef SIMULATOR
  #if ! defined DIO0_PIN && ! defined PIO0_1_PIN
    #error Pins for this chip not defined in arduino.h! If you write an \
           appropriate pin definition and have this firmware work on your chip, \
           please tell us via Github or the forum thread.
  #endif
#endif

#ifndef BSS
  #define BSS __attribute__ ((__section__ (".bss")))
#endif

#endif /* _ARDUINO_H */
