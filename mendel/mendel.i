# 1 "mendel.c"
# 1 "/home/triffid/ATmega-Skeleton/mendel//"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "mendel.c"
# 1 "/usr/lib/gcc/avr/4.4.1/include/stddef.h" 1 3 4
# 149 "/usr/lib/gcc/avr/4.4.1/include/stddef.h" 3 4
typedef int ptrdiff_t;
# 211 "/usr/lib/gcc/avr/4.4.1/include/stddef.h" 3 4
typedef unsigned int size_t;
# 323 "/usr/lib/gcc/avr/4.4.1/include/stddef.h" 3 4
typedef int wchar_t;
# 2 "mendel.c" 2
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 1 3
# 44 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/inttypes.h" 1 3
# 37 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/inttypes.h" 3
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
# 38 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/inttypes.h" 2 3
# 77 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/inttypes.h" 3
typedef int32_t int_farptr_t;



typedef uint32_t uint_farptr_t;
# 45 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 2 3
# 1 "/usr/lib/gcc/avr/4.4.1/include/stdarg.h" 1 3 4
# 40 "/usr/lib/gcc/avr/4.4.1/include/stdarg.h" 3 4
typedef __builtin_va_list __gnuc_va_list;
# 102 "/usr/lib/gcc/avr/4.4.1/include/stdarg.h" 3 4
typedef __gnuc_va_list va_list;
# 46 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 2 3



# 1 "/usr/lib/gcc/avr/4.4.1/include/stddef.h" 1 3 4
# 50 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 2 3
# 242 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
struct __file {
 char *buf;
 unsigned char unget;
 uint8_t flags;
# 261 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
 int size;
 int len;
 int (*put)(char, struct __file *);
 int (*get)(struct __file *);
 void *udata;
};
# 405 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern struct __file *__iob[];
# 417 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern struct __file *fdevopen(int (*__put)(char, struct __file*), int (*__get)(struct __file*));
# 434 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern int fclose(struct __file *__stream);
# 608 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern int vfprintf(struct __file *__stream, const char *__fmt, va_list __ap);





extern int vfprintf_P(struct __file *__stream, const char *__fmt, va_list __ap);






extern int fputc(int __c, struct __file *__stream);




extern int putc(int __c, struct __file *__stream);


extern int putchar(int __c);
# 649 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern int printf(const char *__fmt, ...);





extern int printf_P(const char *__fmt, ...);







extern int vprintf(const char *__fmt, va_list __ap);





extern int sprintf(char *__s, const char *__fmt, ...);





extern int sprintf_P(char *__s, const char *__fmt, ...);
# 685 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern int snprintf(char *__s, size_t __n, const char *__fmt, ...);





extern int snprintf_P(char *__s, size_t __n, const char *__fmt, ...);





extern int vsprintf(char *__s, const char *__fmt, va_list ap);





extern int vsprintf_P(char *__s, const char *__fmt, va_list ap);
# 713 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern int vsnprintf(char *__s, size_t __n, const char *__fmt, va_list ap);





extern int vsnprintf_P(char *__s, size_t __n, const char *__fmt, va_list ap);




extern int fprintf(struct __file *__stream, const char *__fmt, ...);





extern int fprintf_P(struct __file *__stream, const char *__fmt, ...);






extern int fputs(const char *__str, struct __file *__stream);




extern int fputs_P(const char *__str, struct __file *__stream);





extern int puts(const char *__str);




extern int puts_P(const char *__str);
# 762 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern size_t fwrite(const void *__ptr, size_t __size, size_t __nmemb,
         struct __file *__stream);







extern int fgetc(struct __file *__stream);




extern int getc(struct __file *__stream);


extern int getchar(void);
# 810 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern int ungetc(int __c, struct __file *__stream);
# 822 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern char *fgets(char *__str, int __size, struct __file *__stream);






extern char *gets(char *__str);
# 840 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern size_t fread(void *__ptr, size_t __size, size_t __nmemb,
        struct __file *__stream);




extern void clearerr(struct __file *__stream);
# 857 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern int feof(struct __file *__stream);
# 868 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
extern int ferror(struct __file *__stream);






extern int vfscanf(struct __file *__stream, const char *__fmt, va_list __ap);




extern int vfscanf_P(struct __file *__stream, const char *__fmt, va_list __ap);







extern int fscanf(struct __file *__stream, const char *__fmt, ...);




extern int fscanf_P(struct __file *__stream, const char *__fmt, ...);






extern int scanf(const char *__fmt, ...);




extern int scanf_P(const char *__fmt, ...);







extern int vscanf(const char *__fmt, va_list __ap);







extern int sscanf(const char *__buf, const char *__fmt, ...);




extern int sscanf_P(const char *__buf, const char *__fmt, ...);
# 938 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/stdio.h" 3
static __inline__ int fflush(struct __file *stream __attribute__((unused)))
{
 return 0;
}
# 3 "mendel.c" 2


# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 1 3
# 99 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/io.h" 3
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/sfr_defs.h" 1 3
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
# 6 "mendel.c" 2
# 1 "/usr/lib/gcc/avr/4.4.1/../../../../avr/include/avr/interrupt.h" 1 3
# 7 "mendel.c" 2

# 1 "serial.h" 1






# 1 "ringbuffer.h" 1






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
# 9 "mendel.c" 2

int serial_putc_fdev(char c, struct __file *stream)
{
 serial_writechar((uint8_t) c);
 return 0;
}

int serial_getc_fdev(struct __file *stream)
{
 for (;serial_rxchars() == 0;);
 return (int) serial_popchar();
}

static struct __file serio = { .put = serial_putc_fdev, .get = serial_getc_fdev, .flags = (0x0001|0x0002), .udata = 0, };

int main (void)
{

 (__iob[0]) = &serio;
 (__iob[1]) = &serio;
 (__iob[2]) = &serio;


 serial_init();

 __asm__ __volatile__ ("sei" ::);

 for (;;)
 {

 }
}
