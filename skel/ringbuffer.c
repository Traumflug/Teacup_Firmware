#include	"ringbuffer.h"

uint16_t _rb_mod(uint16_t num, uint16_t denom)
{
	for (; num >= denom; num -= denom);
	return num;
}

void ringbuffer_init(ringbuffer *buf, int bufsize)
{
	buf->read_pointer = 0;
	buf->write_pointer = 0;
	buf->size = bufsize - sizeof(ringbuffer);
}

uint16_t ringbuffer_canread(ringbuffer *buf)
{
	return _rb_mod(buf->write_pointer + buf->size + buf->size - buf->read_pointer, buf->size);
}

uint16_t ringbuffer_canwrite(ringbuffer *buf)
{
	return _rb_mod(buf->read_pointer + buf->size + buf->size - buf->write_pointer - 1, buf->size);
}

uint8_t ringbuffer_readchar(ringbuffer *buf)
{
	uint8_t r = 0;
	if (ringbuffer_canread(buf))
	{
		r = buf->data[buf->read_pointer];
		buf->read_pointer = _rb_mod(buf->read_pointer + 1, buf->size);
	}
	return r;
}

void ringbuffer_writechar(ringbuffer *buf, uint8_t data)
{
	if (ringbuffer_canwrite(buf))
	{
		buf->data[buf->write_pointer] = data;
		buf->write_pointer = _rb_mod(buf->write_pointer + 1, buf->size);
	}
}


uint8_t ringbuffer_peekchar(ringbuffer *buf, uint16_t index)
{
	return buf->data[_rb_mod(buf->read_pointer + index, buf->size)];
}

uint16_t ringbuffer_readblock(ringbuffer *buf, uint8_t *newbuf, int size)
{
	uint16_t nc, i;
	uint8_t *rp, *ms;
	if ((nc = ringbuffer_canread(buf)) < size)
		size = nc;
	if (size)
	{
		for (i = 0, rp = buf->data + buf->read_pointer, ms = buf->data + buf->size; i < size; i++, rp++)
		{
			if (rp >= ms)
				rp = buf->data;
			newbuf[i] = *rp;
		}
		buf->read_pointer = rp - buf->data;
	}
	return size;
}

uint16_t ringbuffer_writeblock(ringbuffer *buf, uint8_t *data, int size)
{
	uint16_t nc, i;
	uint8_t	*wp, *ms;

	if ((nc = ringbuffer_canwrite(buf)) < size)
		size = nc;
	if (size)
	{
		for (i = 0, wp = buf->write_pointer + buf->data, ms = buf->data + buf->size; i < size; i++, wp++)
		{
			if (wp >= ms)
				wp = buf->data;
			*wp = data[i];
		}
		buf->write_pointer = wp - buf->data;
	}
	return size;
}
