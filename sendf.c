
/** \file sersendf.c
	\brief Simplified printf implementation
*/

#include	<stdarg.h>

#include "sendf.h"
#include "msg.h"


/** \brief Simplified printf

  \param writechar  The function to use for writing a character. Typically
                    serial_writechar() or display_writechar().

  \param format     Pointer to output format specifier string stored in FLASH.

  \param ...        Data according to the placeholders in format.

	Implements only a tiny subset of printf's format specifiers :-

	%[ls][udcx%]

	l - following data is (32 bits)\n
	s - following data is short (8 bits)\n
	none - following data is 16 bits.

	u - unsigned int\n
	d - signed int\n
	q - signed int with decimal before the third digit from the right\n
	c - character\n
	x - hex\n
	% - send a literal % character

	Example:

  \code sersendf_P(serial_writechar,
                   PSTR("X:%ld Y:%ld temp:%u.%d flags:%sx Q%su/%su%c\n"),
                   target.X, target.Y, current_temp >> 2,
                   (current_temp & 3) * 25, dda.allflags, mb_head, mb_tail,
                   (queue_full()?'F':(queue_empty()?'E':' '))) \endcode
*/

/**
  va_arg() takes "fully promoted types" only, see example in Linux' va_arg
  man page. This covers platforms >= 16 bits and arguments up to 32 bits.
  64 bit arguments on a 32 bit platform will produce a severe warning.
*/
#if __SIZEOF_INT__ == 2
  #define GET_ARG(T) (va_arg(args, T))
#elif __SIZEOF_INT__ >= 4
  #define GET_ARG(T) ((T)va_arg(args, int))
#endif

void sendf_P(void (*writechar)(uint8_t), PGM_P format_P, ...) {
	va_list args;
	va_start(args, format_P);

	uint16_t i = 0;
	uint8_t c = 1, j = 0;
	while ((c = pgm_read_byte(&format_P[i++]))) {
		if (j) {
			switch(c) {
				case 's':
					j = 1;
					break;
				case 'l':
					j = 4;
					break;
				case 'u':
          if (j == 1)
            write_uint8(writechar, (uint8_t)GET_ARG(uint16_t));
          else if (j == 2)
            write_uint16(writechar, (uint16_t)GET_ARG(uint16_t));
					else
            write_uint32(writechar, GET_ARG(uint32_t));
					j = 0;
					break;
				case 'd':
          if (j == 1)
            write_int8(writechar, (int8_t)GET_ARG(int16_t));
          else if (j == 2)
            write_int16(writechar, (int16_t)GET_ARG(int16_t));
					else
            write_int32(writechar, GET_ARG(int32_t));
					j = 0;
					break;
				case 'c':
          writechar((uint8_t)GET_ARG(uint16_t));
					j = 0;
					break;
				case 'x':
          writechar('0');
          writechar('x');
          if (j == 1)
            write_hex8(writechar, (uint8_t)GET_ARG(uint16_t));
          else if (j == 2)
            write_hex16(writechar, (uint16_t)GET_ARG(uint16_t));
					else
            write_hex32(writechar, GET_ARG(uint32_t));
					j = 0;
					break;
/*				case 'p':
          serwrite_hex16(writechar, GET_ARG(uint16_t));*/
				case 'q':
          write_int32_vf(writechar, GET_ARG(uint32_t), 3);
					j = 0;
					break;
				default:
          writechar(c);
					j = 0;
					break;
			}
		}
		else {
			if (c == '%') {
				j = 2;
			}
			else {
        writechar(c);
			}
		}
	}
	va_end(args);
}
