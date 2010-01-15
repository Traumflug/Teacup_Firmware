#ifndef	_RINGBUFFER_H
#define	_RINGBUFFER_H

#include	<stdint.h>
#include	<avr/interrupt.h>

typedef struct {
	uint16_t	read_pointer;
	uint16_t	write_pointer;
	uint16_t	size;
	uint8_t		data[];
} ringbuffer;

void ringbuffer_init(ringbuffer *buf, int bufsize);

uint16_t ringbuffer_canread(ringbuffer *buf);
uint16_t ringbuffer_canwrite(ringbuffer *buf);

uint8_t ringbuffer_readchar(ringbuffer *buf);
uint8_t ringbuffer_peekchar(ringbuffer *buf, uint16_t index);
uint16_t ringbuffer_readblock(ringbuffer *buf, uint8_t *newbuf, int size);

void ringbuffer_writechar(ringbuffer *buf, uint8_t data);
uint16_t ringbuffer_writeblock(ringbuffer *buf, uint8_t *data, int size);

#endif	/* _RINGBUFFER_H */
