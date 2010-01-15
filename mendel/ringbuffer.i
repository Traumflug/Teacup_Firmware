# 1 "ringbuffer.c"
# 1 "/home/triffid/ATmega-Skeleton/mendel//"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "ringbuffer.c"
# 1 "ringbuffer.h" 1



# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdint.h" 1 3
# 121 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdint.h" 3
typedef int int8_t __attribute__((__mode__(__QI__)));
typedef unsigned int uint8_t __attribute__((__mode__(__QI__)));
typedef int int16_t __attribute__ ((__mode__ (__HI__)));
typedef unsigned int uint16_t __attribute__ ((__mode__ (__HI__)));
typedef int int32_t __attribute__ ((__mode__ (__SI__)));
typedef unsigned int uint32_t __attribute__ ((__mode__ (__SI__)));

typedef int int64_t __attribute__((__mode__(__DI__)));
typedef unsigned int uint64_t __attribute__((__mode__(__DI__)));
# 142 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdint.h" 3
typedef int16_t intptr_t;




typedef uint16_t uintptr_t;
# 159 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdint.h" 3
typedef int8_t int_least8_t;




typedef uint8_t uint_least8_t;




typedef int16_t int_least16_t;




typedef uint16_t uint_least16_t;




typedef int32_t int_least32_t;




typedef uint32_t uint_least32_t;







typedef int64_t int_least64_t;






typedef uint64_t uint_least64_t;
# 213 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdint.h" 3
typedef int8_t int_fast8_t;




typedef uint8_t uint_fast8_t;




typedef int16_t int_fast16_t;




typedef uint16_t uint_fast16_t;




typedef int32_t int_fast32_t;




typedef uint32_t uint_fast32_t;







typedef int64_t int_fast64_t;






typedef uint64_t uint_fast64_t;
# 273 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdint.h" 3
typedef int64_t intmax_t;




typedef uint64_t uintmax_t;
# 5 "ringbuffer.h" 2
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/interrupt.h" 1 3
# 38 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/interrupt.h" 3
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 1 3
# 99 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 3
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/sfr_defs.h" 1 3
# 126 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/sfr_defs.h" 3
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/inttypes.h" 1 3
# 77 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/inttypes.h" 3
typedef int32_t int_farptr_t;



typedef uint32_t uint_farptr_t;
# 127 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/sfr_defs.h" 2 3
# 100 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 2 3
# 224 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 3
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/iom168.h" 1 3
# 36 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/iom168.h" 3
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/iomx8.h" 1 3
# 37 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/iom168.h" 2 3
# 225 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 2 3
# 334 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 3
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/portpins.h" 1 3
# 335 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 2 3

# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/common.h" 1 3
# 337 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 2 3

# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/version.h" 1 3
# 339 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 2 3


# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/fuse.h" 1 3
# 234 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/fuse.h" 3
typedef struct
{
    unsigned char low;
    unsigned char high;
    unsigned char extended;
} __fuse_t;
# 342 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 2 3


# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/lock.h" 1 3
# 345 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 2 3
# 39 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/interrupt.h" 2 3
# 6 "ringbuffer.h" 2

typedef struct {
 uint16_t read_pointer;
 uint16_t write_pointer;
 uint16_t size;
 uint8_t data[];
} ringbuffer;

void ringbuffer_init(ringbuffer *buf, int bufsize);

uint16_t ringbuffer_canread(ringbuffer *buf);
uint16_t ringbuffer_canwrite(ringbuffer *buf);

uint8_t ringbuffer_readchar(ringbuffer *buf);
uint8_t ringbuffer_peekchar(ringbuffer *buf, uint16_t index);
uint16_t ringbuffer_readblock(ringbuffer *buf, uint8_t *newbuf, int size);

void ringbuffer_writechar(ringbuffer *buf, uint8_t data);
uint16_t ringbuffer_writeblock(ringbuffer *buf, uint8_t *data, int size);
# 2 "ringbuffer.c" 2

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
 uint8_t *wp, *ms;

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
