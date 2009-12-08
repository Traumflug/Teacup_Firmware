#include	"serial.h"

#include	"ringbuffer.h"

#define		BUFSIZE		64 + sizeof(ringbuffer)
#define		BAUD		19200

volatile uint8_t _rx_buffer[BUFSIZE];
volatile uint8_t _tx_buffer[BUFSIZE];

void serial_init(uint16_t baud)
{
	ringbuffer_init(rx_buffer, BUFSIZE);
	ringbuffer_init(tx_buffer, BUFSIZE);

	UCSR0A = 0;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

	UBRR0 = ((F_CPU / 16) / baud) - 1;

	UCSR0B |= (1 << RXCIE0) | (1 << UDRIE0);
}

ISR(USART_RX_vect)
{
	ringbuffer_writechar(rx_buffer, UDR0);
}

ISR(USART_UDRE_vect)
{
	if (ringbuffer_canread(tx_buffer))
	{
		UDR0 = ringbuffer_readchar(tx_buffer);
	}
	else
	{
		UCSR0B &= ~(1 << UDRIE0);
	}
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
	ringbuffer_writechar(tx_buffer, data);
	UCSR0B |= (1 << UDRIE0);
}

void serial_writeblock(uint8_t *data, int datalen)
{
	ringbuffer_writeblock(tx_buffer, data, datalen);
	UCSR0B |= (1 << UDRIE0);
}
