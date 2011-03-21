/*!
	\file
	\brief pin definitions and I/O macros

	why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

#ifndef	_ARDUINO_H
#define	_ARDUINO_H

#include	<avr/io.h>

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

/*
	ports and functions

	added as necessary or if I feel like it- not a comprehensive list!
*/

#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
	#include	"arduino_168_328p.h"
#endif	/*	_AVR_ATmega{168,328,328P}__ */

#if defined (__AVR_ATmega644__) || defined (__AVR_ATmega644P__) || defined (__AVR_ATmega644PA__)
	#include	"arduino_644.h"
#endif	/*	_AVR_ATmega{644,644P,644PA}__ */

#if defined (__AVR_ATmega1280__)
	#include	"arduino_1280.h"
#endif	/* __AVR_ATmega1280__ */

#if defined (__AVR_ATmega2560__)
	#include    "arduino_1280.h" //2560 has the same pins and ports so we can reuse the 1280 file.
#endif

#if defined (__AVR_AT90USB1287__)
	#include    "arduino_usb1287.h"
#endif

#ifndef	DIO0_PIN
#error pins for this chip not defined in arduino.h! If you write an appropriate pin definition and have this firmware work on your chip, please tell us via the forum thread
#endif

#endif /* _ARDUINO_H */
