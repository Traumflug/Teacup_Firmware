#include	"sersendf.h"

/** \file sersendf.c
	\brief Simplified printf implementation
*/

#include	<stdarg.h>
#ifndef SIMULATOR
#include	<avr/pgmspace.h>
#endif

#include	"serial.h"
#include	"sermsg.h"

// void sersendf(char *format, ...) {
// 	va_list args;
// 	va_start(args, format);
//
// 	uint16_t i = 0;
// 	uint8_t c, j = 0;
// 	while ((c = format[i++])) {
// 		if (j) {
// 			switch(c) {
// 				case 'l':
// 					j = 4;
// 					break;
// 				case 'u':
// 					if (j == 4)
// 						serwrite_uint32(va_arg(args, uint32_t));
// 					else
// 						serwrite_uint16(va_arg(args, uint16_t));
// 					j = 0;
// 					break;
// 				case 'd':
// 					if (j == 4)
// 						serwrite_int32(va_arg(args, int32_t));
// 					else
// 						serwrite_int16(va_arg(args, int16_t));
// 					j = 0;
// 					break;
// 				case 'p':
// 				case 'x':
// 					serial_writestr_P(str_ox);
// 					if (j == 4)
// 						serwrite_hex32(va_arg(args, uint32_t));
// 					else
// 						serwrite_hex16(va_arg(args, uint16_t));
// 					j = 0;
// 					break;
// 				case 'c':
// 					serial_writechar(va_arg(args, uint16_t));
// 					j = 0;
// 					break;
// 				case 's':
// 					serial_writestr(va_arg(args, uint8_t *));
// 					j = 0;
// 					break;
// 				default:
// 					serial_writechar(c);
// 					j = 0;
// 					break;
// 			}
// 		}
// 		else {
// 			if (c == '%') {
// 				j = 2;
// 			}
// 			else {
// 				serial_writechar(c);
// 			}
// 		}
// 	}
// 	va_end(args);
// }

/** \brief Simplified printf
	\param format pointer to output format specifier string stored in FLASH.
	\param ... output data

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

	\code sersendf_P(PSTR("X:%ld Y:%ld temp:%u.%d flags:%sx Q%su/%su%c\n"), target.X, target.Y, current_temp >> 2, (current_temp & 3) * 25, dda.allflags, mb_head, mb_tail, (queue_full()?'F':(queue_empty()?'E':' '))) \endcode
*/

#ifdef SIMULATOR
  #define GET_ARG(T) (va_arg(args, int))
#else
  #define GET_ARG(T) (va_arg(args, T))
#endif

void sersendf_P(PGM_P format, ...) {
	va_list args;
	va_start(args, format);

	uint16_t i = 0;
	uint8_t c = 1, j = 0;
	while ((c = pgm_read_byte(&format[i++]))) {
		if (j) {
			switch(c) {
				case 's':
					j = 1;
					break;
				case 'l':
					j = 4;
					break;
				case 'u':
					if (j == 4)
            serwrite_uint32(GET_ARG(uint32_t));
					else
            serwrite_uint16(GET_ARG(uint16_t));
					j = 0;
					break;
				case 'd':
					if (j == 4)
            serwrite_int32(GET_ARG(uint32_t));
					else
            serwrite_int16(GET_ARG(uint16_t));
					j = 0;
					break;
				case 'c':
          serial_writechar(GET_ARG(uint16_t));
					j = 0;
					break;
				case 'x':
					serial_writestr_P(PSTR("0x"));
					if (j == 4)
            serwrite_hex32(GET_ARG(uint32_t));
					else if (j == 1)
            serwrite_hex8(GET_ARG(uint16_t));
					else
            serwrite_hex16(GET_ARG(uint16_t));
					j = 0;
					break;
/*				case 'p':
          serwrite_hex16(GET_ARG(uint16_t));*/
				case 'q':
          serwrite_int32_vf(GET_ARG(uint32_t), 3);
					j = 0;
					break;
				default:
					serial_writechar(c);
					j = 0;
					break;
			}
		}
		else {
			if (c == '%') {
				j = 2;
			}
			else {
				serial_writechar(c);
			}
		}
	}
	va_end(args);
}
