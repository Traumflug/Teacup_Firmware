#ifndef	_SERIAL_H
#define	_SERIAL_H

#include	<stdint.h>
#include	<avr/io.h>
#include	<avr/pgmspace.h>

void serial_init(void);

uint16_t serial_rxchars(void);
uint16_t serial_txchars(void);

uint8_t serial_popchar(void);
void serial_writechar(uint8_t data);

uint16_t serial_recvblock(uint8_t *block, int blocksize);
void serial_writeblock(void *data, int datalen);

void serial_writechar_P(PGM_P data);
void serial_writeblock_P(PGM_P data, int datalen);

void serial_writestr_P(PGM_P data);

#endif	/* _SERIAL_H */
