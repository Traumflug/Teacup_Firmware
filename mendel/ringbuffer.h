#ifndef	_RINGBUFFER_H
#define	_RINGBUFFER_H

#include	<stdint.h>
#include	<avr/interrupt.h>

#define	RB_BITS	uint8_t

typedef struct {
	volatile RB_BITS		read_pointer;
	volatile RB_BITS		write_pointer;
	volatile RB_BITS		size;
	volatile RB_BITS		data[];
} ringbuffer;

void ringbuffer_init(ringbuffer *buf, RB_BITS bufsize);

RB_BITS ringbuffer_canread(ringbuffer *buf);
RB_BITS ringbuffer_canwrite(ringbuffer *buf);

uint8_t ringbuffer_readchar(ringbuffer *buf);
uint8_t ringbuffer_peekchar(ringbuffer *buf, RB_BITS index);
RB_BITS ringbuffer_readblock(ringbuffer *buf, uint8_t *newbuf, RB_BITS size);

void ringbuffer_writechar(ringbuffer *buf, uint8_t data);
RB_BITS ringbuffer_writeblock(ringbuffer *buf, uint8_t *data, RB_BITS size);

#endif	/* _RINGBUFFER_H */
