#ifndef	_ARDUINO_H
#define	_ARDUINO_H

#include	<avr/io.h>

/*
	utility functions
*/

#ifndef		MASK
#define		MASK(PIN)				(1 << PIN)
#endif

/*
	magic I/O routines

	now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
*/

#define		_READ(IO)					(IO ## _RPORT & MASK(IO ## _PIN))
#define		_WRITE(IO, v)			do { if (v) { IO ## _WPORT |= MASK(IO ## _PIN); } else { IO ## _WPORT &= ~MASK(IO ## _PIN); }; } while (0)
#define		_TOGGLE(IO)				do { IO ## _RPORT = MASK(IO ## _PIN); } while (0)

#define		_SET_INPUT(IO)		do { IO ## _DDR &= ~MASK(IO ## _PIN); } while (0)
#define		_SET_OUTPUT(IO)		do { IO ## _DDR |=  MASK(IO ## _PIN); } while (0)

#define		_GET_INPUT(IO)		((IO ## _DDR & MASK(IO ## _PIN)) == 0)
#define		_GET_OUTPUT(IO)		((IO ## _DDR & MASK(IO ## _PIN)) != 0)

// why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html

#define		READ(IO)					_READ(IO)
#define		WRITE(IO, v)			_WRITE(IO, v)
#define		TOGGLE(IO)				_TOGGLE(IO)
#define		SET_INPUT(IO)			_SET_INPUT(IO)
#define		SET_OUTPUT(IO)		_SET_OUTPUT(IO)
#define		GET_INPUT(IO)			_GET_INPUT(IO)
#define		GET_OUTPUT(IO)		_GET_OUTPUT(IO)

/*
	ports and functions

	added as necessary or if I feel like it- not a comprehensive list!

	probably needs some #ifdefs for various chip types
*/

#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
	#include	"arduino_168_328p.h"
#endif	/*	_AVR_ATmega{168,328,328P}__) */

#if defined (__AVR_ATmega644__) || defined (__AVR_ATmega644P__) || defined (__AVR_ATmega644PA__)
	#include	"arduino_644.h"
#endif

#if defined (__AVR_ATmega1280__)
	#include	"arduino_1280.h"
#endif	/* __AVR_ATmega1280__) */

#ifndef	DIO0_PIN
#error pins for this chip not defined in arduino.h! If you write an appropriate pin definition and have this firmware work on your chip, please tell us via the forum thread
#endif

#endif /* _ARDUINO_H */
