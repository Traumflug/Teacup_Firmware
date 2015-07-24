/*!
	\file
	\brief pin definitions and I/O macros

	why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

#ifndef	_ARDUINO_H
#define	_ARDUINO_H

#ifdef __AVR__
#include	<avr/io.h>
#endif

/*
	utility functions
*/

#ifndef		MASK
/// MASKING- returns \f$2^PIN\f$
	#define		MASK(PIN)				(1 << PIN)
#endif

/*
	magic I/O routines

	now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
*/

/// Read a pin
#define		_READ(IO)					(IO ## _RPORT & MASK(IO ## _PIN))
/// write to a pin
#define		_WRITE(IO, v)			do { if (v) { IO ## _WPORT |= MASK(IO ## _PIN); } else { IO ## _WPORT &= ~MASK(IO ## _PIN); }; } while (0)
/// toggle a pin
#define		_TOGGLE(IO)				do { IO ## _RPORT = MASK(IO ## _PIN); } while (0)

/// set pin as input
#define		_SET_INPUT(IO)		do { IO ## _DDR &= ~MASK(IO ## _PIN); } while (0)
/// set pin as output
#define		_SET_OUTPUT(IO)		do { IO ## _DDR |=  MASK(IO ## _PIN); } while (0)

/// check if pin is an input
#define		_GET_INPUT(IO)		((IO ## _DDR & MASK(IO ## _PIN)) == 0)
/// check if pin is an output
#define		_GET_OUTPUT(IO)		((IO ## _DDR & MASK(IO ## _PIN)) != 0)

//	why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html

/// Read a pin wrapper
#define		READ(IO)					_READ(IO)
/// Write to a pin wrapper
#define		WRITE(IO, v)			_WRITE(IO, v)
/// toggle a pin wrapper
#define		TOGGLE(IO)				_TOGGLE(IO)

/// set pin as input wrapper
#define		SET_INPUT(IO)			_SET_INPUT(IO)
/// set pin as output wrapper
#define		SET_OUTPUT(IO)		_SET_OUTPUT(IO)

/// check if pin is an input wrapper
#define		GET_INPUT(IO)			_GET_INPUT(IO)
/// check if pin is an output wrapper
#define		GET_OUTPUT(IO)		_GET_OUTPUT(IO)

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

  #define DIO0_PIN remove when actually defined.

#elif defined SIMULATOR

  #include "simulator.h"

#endif /* __AVR__, __ARMEL__, SIMULATOR */

#ifndef	DIO0_PIN
#error pins for this chip not defined in arduino.h! If you write an appropriate pin definition and have this firmware work on your chip, please tell us via the forum thread
#endif

#ifndef BSS
  #define BSS __attribute__ ((__section__ (".bss")))
#endif

#endif /* _ARDUINO_H */
