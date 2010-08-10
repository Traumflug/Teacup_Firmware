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
#define		_TOGGLE(IO)				(IO ## _RPORT = MASK(IO ## _PIN))

#define		_SET_INPUT(IO)		(IO ## _DDR &= ~MASK(IO ## _PIN))
#define		_SET_OUTPUT(IO)		(IO ## _DDR |=  MASK(IO ## _PIN))

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

// UART
#define	RXD									DIO0
#define	TXD									DIO1

// SPI
#define	SCK									DIO13
#define	MISO								DIO12
#define	MOSI								DIO11
#define	SS									DIO10

// TWI (I2C)
#define	SCL									AIO5
#define	SDA									AIO4

// timers and PWM
#define	OC0A								DIO6
#define	OC0B								DIO5
#define	OC1A								DIO9
#define	OC1B								DIO10
#define	OC2A								DIO11
#define	OC2B								DIO3

#define	ICP1								DIO8

/*
	pins
*/

#define DIO0_PIN		PIND0
#define DIO0_RPORT	PIND
#define DIO0_WPORT	PORTD
#define DIO0_DDR		DDRD

#define DIO1_PIN		PIND1
#define DIO1_RPORT	PIND
#define DIO1_WPORT	PORTD
#define DIO1_DDR		DDRD

#define DIO2_PIN		PIND2
#define DIO2_RPORT	PIND
#define DIO2_WPORT	PORTD
#define DIO2_DDR		DDRD

#define DIO3_PIN		PIND3
#define DIO3_RPORT	PIND
#define DIO3_WPORT	PORTD
#define DIO3_DDR		DDRD

#define DIO4_PIN		PIND4
#define DIO4_RPORT	PIND
#define DIO4_WPORT	PORTD
#define DIO4_DDR		DDRD

#define DIO5_PIN  PIND5
#define DIO5_RPORT	PIND
#define DIO5_WPORT	PORTD
#define DIO5_DDR		DDRD

#define DIO6_PIN  PIND6
#define DIO6_RPORT	PIND
#define DIO6_WPORT	PORTD
#define DIO6_DDR		DDRD

#define DIO7_PIN  PIND7
#define DIO7_RPORT	PIND
#define DIO7_WPORT	PORTD
#define DIO7_DDR		DDRD

#define DIO8_PIN  PINB0
#define DIO8_RPORT	PINB
#define DIO8_WPORT	PORTB
#define DIO8_DDR		DDRB

#define DIO9_PIN  PINB1
#define DIO9_RPORT	PINB
#define DIO9_WPORT	PORTB
#define DIO9_DDR		DDRB

#define DIO10_PIN  PINB2
#define DIO10_RPORT	PINB
#define DIO10_WPORT	PORTB
#define DIO10_DDR		DDRB

#define DIO11_PIN  PINB3
#define DIO11_RPORT	PINB
#define DIO11_WPORT	PORTB
#define DIO11_DDR		DDRB

#define DIO12_PIN  PINB4
#define DIO12_RPORT	PINB
#define DIO12_WPORT	PORTB
#define DIO12_DDR		DDRB

#define DIO13_PIN  PINB5
#define DIO13_RPORT	PINB
#define DIO13_WPORT	PORTB
#define DIO13_DDR		DDRB

#define AIO0_PIN		PINC0
#define AIO0_RPORT	PINC
#define AIO0_WPORT	PORTC
#define AIO0_DDR		DDRC

#define AIO1_PIN		PINC1
#define AIO1_RPORT	PINC
#define AIO1_WPORT	PORTC
#define AIO1_DDR		DDRC

#define AIO2_PIN  PINC2
#define AIO2_RPORT	PINC
#define AIO2_WPORT	PORTC
#define AIO2_DDR		DDRC

#define AIO3_PIN  PINC3
#define AIO3_RPORT	PINC
#define AIO3_WPORT	PORTC
#define AIO3_DDR		DDRC

#define AIO4_PIN  PINC4
#define AIO4_RPORT	PINC
#define AIO4_WPORT	PORTC
#define AIO4_DDR		DDRC

#define AIO5_PIN  PINC5
#define AIO5_RPORT	PINC
#define AIO5_WPORT	PORTC
#define AIO5_DDR		DDRC

/*
	timers
*/

// list of PWM-able pins and corresponding timers
// OC0A											DIO6
// OC0B											DIO5
// OC1A											DIO9
// OC1B											DIO10
// OC2A											DIO11
// OC2B											DIO3


#define	OC0A_PIN				DIO6
#define	OC0A_CMR				OCR0A
#define	OC0A_COM_PORT		TCCR0A
#define	OC0A_COM0				COM0A0
#define	OC0A_COM1				COM0A1
#define	OC0A_WGM0_PORT	TCCR0A
#define	OC0A_WGM0_PIN		WGM00
#define	OC0A_WGM1_PORT	TCCR0A
#define	OC0A_WGM1_PIN		WGM01
#define	OC0A_WGM2_PORT	TCCR0B
#define	OC0A_WGM2_PIN		WGM02
#define	OC0A_MSK_PORT		TIMSK0
#define	OC0A_MSK_PIN		OCIE0A
#define	OC0A_INT_PORT		TIFR0
#define	OC0A_INT_PIN		OCF0A

#define	OC0B_PIN		DIO5
#define	OC1A_PIN		DIO9
#define	OC1B_PIN		DIO10
#define	OC2A_PIN		DIO11
#define	OC2B_PIN		DIO3


#endif /* _ARDUINO_H */