#include	"serial.h"

#include	"ringbuffer.h"
#include	"arduino.h"

#define		BUFSIZE		64 + sizeof(ringbuffer)
#define		BAUD		57600

#define		ASCII_XOFF	19
#define		ASCII_XON		17

volatile uint8_t _rx_buffer[BUFSIZE];
volatile uint8_t _tx_buffer[BUFSIZE];

volatile uint8_t flowflags = 0;
#define		FLOWFLAG_SEND_XOFF	1
#define		FLOWFLAG_SEND_XON		2
#define		FLOWFLAG_SENT_XOFF	4
#define		FLOWFLAG_SENT_XON		8

#define	rx_buffer	((ringbuffer *) _rx_buffer)
#define	tx_buffer	((ringbuffer *) _tx_buffer)

void serial_init()
{
	ringbuffer_init(rx_buffer, BUFSIZE);
	ringbuffer_init(tx_buffer, BUFSIZE);

	UCSR0A = 0;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

	UBRR0 = ((F_CPU / 16) / BAUD) - 1;

	UCSR0B |= (1 << RXCIE0) | (1 << UDRIE0);
}

ISR(USART_RX_vect)
{
	ringbuffer_writechar(rx_buffer, UDR0);
}

ISR(USART_UDRE_vect)
{
	if (flowflags & FLOWFLAG_SEND_XOFF) {
		UDR0 = ASCII_XOFF;
		flowflags = (flowflags & ~FLOWFLAG_SEND_XOFF) | FLOWFLAG_SENT_XOFF;
	}
	else if (flowflags & FLOWFLAG_SEND_XON) {
		UDR0 = ASCII_XON;
		flowflags = (flowflags & ~FLOWFLAG_SEND_XON) | FLOWFLAG_SENT_XON;
	}
	else if (ringbuffer_canread(tx_buffer))
		UDR0 = ringbuffer_readchar(tx_buffer);
	else
		UCSR0B &= ~(1 << UDRIE0);
}

uint16_t serial_rxchars()
{
	return ringbuffer_canread(rx_buffer);
}

uint16_t serial_txchars()
{
	return ringbuffer_canread(tx_buffer);
}

uint8_t serial_popchar()
{
	return ringbuffer_readchar(rx_buffer);
}

uint16_t serial_recvblock(uint8_t *block, int blocksize)
{
	return ringbuffer_readblock(rx_buffer, block, blocksize);
}

void serial_writechar(uint8_t data)
{
	// check if interrupts are enabled
	if (SREG & MASK(SREG_I)) {
		// if they are, we should be ok to block
		for (;ringbuffer_canwrite(tx_buffer) == 0;);
		ringbuffer_writechar(tx_buffer, data);
	}
	else {
		// interrupts are disabled- maybe we're in one?
		// anyway, instead of blocking, only write if we have room
		if (ringbuffer_canwrite(tx_buffer))
			ringbuffer_writechar(tx_buffer, data);
	}
	// enable TX interrupt so we can send this character
	UCSR0B |= (1 << UDRIE0);
}

void serial_writeblock(void *data, int datalen)
{
	for (int i = 0; i < datalen; i++)
		serial_writechar(((uint8_t *) data)[i]);
}

void serial_writechar_P(PGM_P data)
{
	for (;ringbuffer_canwrite(tx_buffer) == 0;);
	ringbuffer_writechar(tx_buffer, pgm_read_byte(data));
	UCSR0B |= (1 << UDRIE0);
}

void serial_writeblock_P(PGM_P data, int datalen)
{
	for (int i = 0; i < datalen; i++)
		serial_writechar_P(&data[i]);
}

void serial_writestr_P(PGM_P data)
{
	uint8_t i = 0;
	// yes, this is *supposed* to be assignment rather than comparison
	for (uint8_t r; (r = pgm_read_byte(&data[i])); i++)
		serial_writechar(r);
}

void xoff() {
	flowflags = FLOWFLAG_SEND_XOFF;
	// enable TX interrupt so we can send this character
	UCSR0B |= (1 << UDRIE0);
}

void xon() {
	if (flowflags & FLOWFLAG_SENT_XOFF)
		flowflags = FLOWFLAG_SEND_XON;
	// enable TX interrupt so we can send this character
	UCSR0B |= (1 << UDRIE0);
}
