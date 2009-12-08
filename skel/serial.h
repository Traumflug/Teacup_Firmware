#ifndef	_SERIAL_H
#define	_SERIAL_H

#include	<stdint.h>
#include	<avr/io.h>

#define	rx_buffer	((ringbuffer *) _rx_buffer)
#define	tx_buffer	((ringbuffer *) _tx_buffer)

extern volatile uint8_t _rx_buffer[];
extern volatile uint8_t _tx_buffer[];

void serial_init(uint16_t baud);

uint16_t serial_rxchars(void);
uint16_t serial_txchars(void);

uint8_t serial_popchar(void);
void serial_writechar(uint8_t data);

uint16_t serial_recvblock(uint8_t *block, int blocksize);
void serial_writeblock(uint8_t *data, int datalen);

#endif	/* _SERIAL_H */
