#include	"ringbuffer.h"

RB_BITS _rb_mod(RB_BITS num, RB_BITS denom)
{
	for (; num >= denom; num -= denom);
	return num;
}

void ringbuffer_init(ringbuffer *buf, RB_BITS bufsize)
{
	buf->read_pointer = 0;
	buf->write_pointer = 0;
	buf->size = bufsize - sizeof(ringbuffer);
}

RB_BITS ringbuffer_canread(ringbuffer *buf)
{
	return _rb_mod(buf->size + buf->write_pointer - buf->read_pointer, buf->size);
}

RB_BITS ringbuffer_canwrite(ringbuffer *buf)
{
	return _rb_mod(buf->size + buf->size + buf->read_pointer - buf->write_pointer - 1, buf->size);
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


uint8_t ringbuffer_peekchar(ringbuffer *buf, RB_BITS index)
{
	return buf->data[_rb_mod(buf->read_pointer + index, buf->size)];
}

RB_BITS ringbuffer_readblock(ringbuffer *buf, uint8_t *newbuf, RB_BITS size)
{
	RB_BITS nc, i;
	uint8_t *rp, *ms;
	if ((nc = ringbuffer_canread(buf)) < size)
		size = nc;
	if (size)
	{
		for (i = 0, rp = ((uint8_t *) buf->data + buf->read_pointer), ms = ((uint8_t *) buf->data + buf->size); i < size; i++, rp++)
		{
			if (rp >= ms)
				rp = (uint8_t *) buf->data;
			newbuf[i] = *rp;
		}
		buf->read_pointer = rp - buf->data;
	}
	return size;
}

RB_BITS ringbuffer_writeblock(ringbuffer *buf, uint8_t *data, RB_BITS size)
{
	RB_BITS nc, i;
	uint8_t	*wp, *ms;

	if ((nc = ringbuffer_canwrite(buf)) < size)
		size = nc;
	if (size)
	{
		for (i = 0, wp = (uint8_t *) (buf->write_pointer + buf->data), ms = (uint8_t *) (buf->data + buf->size); i < size; i++, wp++)
		{
			if (wp >= ms)
				wp = (uint8_t *) buf->data;
			*wp = data[i];
		}
		buf->write_pointer = wp - buf->data;
	}
	return size;
}
