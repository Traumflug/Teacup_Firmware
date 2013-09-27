#include	"serial.h"

/** \file
	\brief Serial subsystem

	Teacup's serial subsystem is a powerful, thoroughly tested and highly modular serial management system.

	It uses ringbuffers for both transmit and receive, and intelligently decides whether to wait or drop transmitted characters if the buffer is full.

	It also supports XON/XOFF flow control of the receive buffer, to help avoid overruns.
*/

#include	<avr/interrupt.h>
#include	"memory_barrier.h"

#include	"arduino.h"

/// size of TX and RX buffers. MUST be a \f$2^n\f$ value
#define		BUFSIZE			64

/// ascii XOFF character
#define		ASCII_XOFF	19
/// ascii XON character
#define		ASCII_XON		17

#ifndef USB_SERIAL
/// rx buffer head pointer. Points to next available space.
volatile uint8_t rxhead = 0;
/// rx buffer tail pointer. Points to last character in buffer
volatile uint8_t rxtail = 0;
/// rx buffer
volatile uint8_t rxbuf[BUFSIZE];

/// tx buffer head pointer. Points to next available space.
volatile uint8_t txhead = 0;
/// tx buffer tail pointer. Points to last character in buffer
volatile uint8_t txtail = 0;
/// tx buffer
volatile uint8_t txbuf[BUFSIZE];

/// check if we can read from this buffer
#define	buf_canread(buffer)			((buffer ## head - buffer ## tail    ) & (BUFSIZE - 1))
/// read from buffer
#define	buf_pop(buffer, data)		do { data = buffer ## buf[buffer ## tail]; buffer ## tail = (buffer ## tail + 1) & (BUFSIZE - 1); } while (0)

/// check if we can write to this buffer
#define	buf_canwrite(buffer)		((buffer ## tail - buffer ## head - 1) & (BUFSIZE - 1))
/// write to buffer
#define	buf_push(buffer, data)	do { buffer ## buf[buffer ## head] = data; buffer ## head = (buffer ## head + 1) & (BUFSIZE - 1); } while (0)

/*
	ringbuffer logic:
	head = written data pointer
	tail = read data pointer

	when head == tail, buffer is empty
	when head + 1 == tail, buffer is full
	thus, number of available spaces in buffer is (tail - head) & bufsize

	can write:
	(tail - head - 1) & (BUFSIZE - 1)

	write to buffer:
	buf[head++] = data; head &= (BUFSIZE - 1);

	can read:
	(head - tail) & (BUFSIZE - 1)

	read from buffer:
	data = buf[tail++]; tail &= (BUFSIZE - 1);
*/

#ifdef	XONXOFF
#define		FLOWFLAG_STATE_XOFF	0
#define		FLOWFLAG_SEND_XON		1
#define		FLOWFLAG_SEND_XOFF	2
#define		FLOWFLAG_STATE_XON	4
// initially, send an XON
volatile uint8_t flowflags = FLOWFLAG_SEND_XON;
#endif

/// initialise serial subsystem
///
/// set up baud generator and interrupts, clear buffers
void serial_init()
{
#if BAUD > 38401
	UCSR0A = MASK(U2X0);
	UBRR0 = (((F_CPU / 8) / BAUD) - 0.5);
#else
	UCSR0A = 0;
	UBRR0 = (((F_CPU / 16) / BAUD) - 0.5);
#endif

	UCSR0B = MASK(RXEN0) | MASK(TXEN0);
	UCSR0C = MASK(UCSZ01) | MASK(UCSZ00);

	UCSR0B |= MASK(RXCIE0) | MASK(UDRIE0);
}

/*
	Interrupts
*/

/// receive interrupt
///
/// we have received a character, stuff it in the rx buffer if we can, or drop it if we can't
// Using the pragma inside the function is incompatible with Arduinos' gcc.
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#ifdef	USART_RX_vect
ISR(USART_RX_vect)
#else
ISR(USART0_RX_vect)
#endif
{
	// save status register
	uint8_t sreg_save = SREG;

	if (buf_canwrite(rx))
		buf_push(rx, UDR0);
	else {
    // Not reading the character makes the interrupt logic to swamp us with
    // retries, so better read it and throw it away.
    // #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
		uint8_t trash;
    // #pragma GCC diagnostic pop

		trash = UDR0;
	}

	#ifdef	XONXOFF
	if (flowflags & FLOWFLAG_STATE_XON && buf_canwrite(rx) <= 16) {
		// the buffer has only 16 free characters left, so send an XOFF
		// more characters might come in until the XOFF takes effect
		flowflags = FLOWFLAG_SEND_XOFF | FLOWFLAG_STATE_XON;
		// enable TX interrupt so we can send this character
		UCSR0B |= MASK(UDRIE0);
	}
	#endif

	// restore status register
	MEMORY_BARRIER();
	SREG = sreg_save;
}
#pragma GCC diagnostic pop

/// transmit buffer ready interrupt
///
/// provide the next character to transmit if we can, otherwise disable this interrupt
#ifdef	USART_UDRE_vect
ISR(USART_UDRE_vect)
#else
ISR(USART0_UDRE_vect)
#endif
{
	// save status register
	uint8_t sreg_save = SREG;

	#ifdef	XONXOFF
	if (flowflags & FLOWFLAG_SEND_XON) {
		UDR0 = ASCII_XON;
		flowflags = FLOWFLAG_STATE_XON;
	}
	else if (flowflags & FLOWFLAG_SEND_XOFF) {
		UDR0 = ASCII_XOFF;
		flowflags = FLOWFLAG_STATE_XOFF;
	}
	else
	#endif
	if (buf_canread(tx))
		buf_pop(tx, UDR0);
	else
		UCSR0B &= ~MASK(UDRIE0);

	// restore status register
	MEMORY_BARRIER();
	SREG = sreg_save;
}

/*
	Read
*/

/// check how many characters can be read
uint8_t serial_rxchars()
{
	return buf_canread(rx);
}

/// read one character
uint8_t serial_popchar()
{
	uint8_t c = 0;

	// it's imperative that we check, because if the buffer is empty and we pop, we'll go through the whole buffer again
	if (buf_canread(rx))
		buf_pop(rx, c);

	#ifdef	XONXOFF
	if ((flowflags & FLOWFLAG_STATE_XON) == 0 && buf_canread(rx) <= 16) {
		// the buffer has (BUFSIZE - 16) free characters again, so send an XON
		flowflags = FLOWFLAG_SEND_XON;
		UCSR0B |= MASK(UDRIE0);
	}
	#endif

	return c;
}

/*
	Write
*/

/// send one character
void serial_writechar(uint8_t data)
{
	// check if interrupts are enabled
	if (SREG & MASK(SREG_I)) {
		// if they are, we should be ok to block since the tx buffer is emptied from an interrupt
		for (;buf_canwrite(tx) == 0;);
		buf_push(tx, data);
	}
	else {
		// interrupts are disabled- maybe we're in one?
		// anyway, instead of blocking, only write if we have room
		if (buf_canwrite(tx))
			buf_push(tx, data);
	}
	// enable TX interrupt so we can send this character
	UCSR0B |= MASK(UDRIE0);
}
#endif /* USB_SERIAL */

/// send a whole block
void serial_writeblock(void *data, int datalen)
{
	int i;

	for (i = 0; i < datalen; i++)
		serial_writechar(((uint8_t *) data)[i]);
}

/// send a string- look for null byte instead of expecting a length
void serial_writestr(uint8_t *data)
{
	uint8_t i = 0, r;
	// yes, this is *supposed* to be assignment rather than comparison, so we break when r is assigned zero
	while ((r = data[i++]))
		serial_writechar(r);
}

/**
	Write block from FLASH

	Extensions to output flash memory pointers. This prevents the data to
	become part of the .data segment instead of the .code segment. That means
	less memory is consumed for multi-character writes.

	For single character writes (i.e. '\n' instead of "\n"), using
	serial_writechar() directly is the better choice.
*/
void serial_writeblock_P(PGM_P data, int datalen)
{
	int i;

	for (i = 0; i < datalen; i++)
		serial_writechar(pgm_read_byte(&data[i]));
}

/// Write string from FLASH
void serial_writestr_P(PGM_P data)
{
	uint8_t r, i = 0;
	// yes, this is *supposed* to be assignment rather than comparison, so we break when r is assigned zero
	while ((r = pgm_read_byte(&data[i++])))
		serial_writechar(r);
}
