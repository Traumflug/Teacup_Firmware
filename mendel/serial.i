# 1 "serial.c"
# 1 "/home/triffid/ATmega-Skeleton/mendel//"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "serial.c"
# 1 "serial.h" 1



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
# 5 "serial.h" 2
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
# 6 "serial.h" 2

# 1 "ringbuffer.h" 1




# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/interrupt.h" 1 3
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
# 8 "serial.h" 2




extern volatile uint8_t _rx_buffer[];
extern volatile uint8_t _tx_buffer[];

void serial_init(void);

uint16_t serial_rxchars(void);
uint16_t serial_txchars(void);

uint8_t serial_popchar(void);
void serial_writechar(uint8_t data);

uint16_t serial_recvblock(uint8_t *block, int blocksize);
void serial_writeblock(uint8_t *data, int datalen);
# 2 "serial.c" 2






volatile uint8_t _rx_buffer[64 + sizeof(ringbuffer)];
volatile uint8_t _tx_buffer[64 + sizeof(ringbuffer)];

void serial_init()
{
 ringbuffer_init(((ringbuffer *) _rx_buffer), 64 + sizeof(ringbuffer));
 ringbuffer_init(((ringbuffer *) _tx_buffer), 64 + sizeof(ringbuffer));

 (*(volatile uint8_t *)(0xC0)) = 0;
 (*(volatile uint8_t *)(0xC1)) = (1 << 4) | (1 << 3);
 (*(volatile uint8_t *)(0xC2)) = (1 << 2) | (1 << 1);

 (*(volatile uint16_t *)(0xC4)) = ((16000000L / 16) / 19200) - 1;

 (*(volatile uint8_t *)(0xC1)) |= (1 << 7) | (1 << 5);
}

void __vector_18 (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_18 (void)
{
 ringbuffer_writechar(((ringbuffer *) _rx_buffer), (*(volatile uint8_t *)(0xC6)));
}

void __vector_19 (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_19 (void)
{
 if (ringbuffer_canread(((ringbuffer *) _tx_buffer)))
 {
  (*(volatile uint8_t *)(0xC6)) = ringbuffer_readchar(((ringbuffer *) _tx_buffer));
 }
 else
 {
  (*(volatile uint8_t *)(0xC1)) &= ~(1 << 5);
 }
}

uint16_t serial_rxchars()
{
 return ringbuffer_canread(((ringbuffer *) _rx_buffer));
}

uint16_t serial_txchars()
{
 return ringbuffer_canread(((ringbuffer *) _tx_buffer));
}

uint8_t serial_popchar()
{
 return ringbuffer_readchar(((ringbuffer *) _rx_buffer));
}

uint16_t serial_recvblock(uint8_t *block, int blocksize)
{
 return ringbuffer_readblock(((ringbuffer *) _rx_buffer), block, blocksize);
}

void serial_writechar(uint8_t data)
{
 ringbuffer_writechar(((ringbuffer *) _tx_buffer), data);
 (*(volatile uint8_t *)(0xC1)) |= (1 << 5);
}

void serial_writeblock(uint8_t *data, int datalen)
{
 ringbuffer_writeblock(((ringbuffer *) _tx_buffer), data, datalen);
 (*(volatile uint8_t *)(0xC1)) |= (1 << 5);
}
