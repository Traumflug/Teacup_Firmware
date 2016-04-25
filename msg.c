
/** \file msg.c

  \brief Primitives for sending numbers over links.
*/

#include "msg.h"

/** write a single hex digit
	\param v hex digit to write, higher nibble ignored
*/
void write_hex4(void (*writechar)(uint8_t), uint8_t v) {
	v &= 0xF;
	if (v < 10)
    writechar('0' + v);
	else
    writechar('A' - 10 + v);
}

/** write a pair of hex digits
	\param v byte to write. One byte gives two hex digits
*/
void write_hex8(void (*writechar)(uint8_t), uint8_t v) {
  write_hex4(writechar, v >> 4);
  write_hex4(writechar, v & 0x0F);
}

/** write four hex digits
	\param v word to write
*/
void write_hex16(void (*writechar)(uint8_t), uint16_t v) {
  write_hex8(writechar, v >> 8);
  write_hex8(writechar, v & 0xFF);
}

/** write eight hex digits
	\param v long word to write
*/
void write_hex32(void (*writechar)(uint8_t), uint32_t v) {
  write_hex16(writechar, v >> 16);
  write_hex16(writechar, v & 0xFFFF);
}

/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
const uint32_t powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/** write decimal digits from a long unsigned int
	\param v number to send
*/
void write_uint32(void (*writechar)(uint8_t), uint32_t v) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
    writechar(t + '0');
	}
	while (e--);
}

/** write decimal digits from a long signed int
	\param v number to send
*/
void write_int32(void (*writechar)(uint8_t), int32_t v) {
	if (v < 0) {
    writechar('-');
		v = -v;
	}

  write_uint32(writechar, v);
}

/** write decimal digits from a long unsigned int
\param v number to send
\param fp number of decimal places to the right of the decimal point
*/
void write_uint32_vf(void (*writechar)(uint8_t), uint32_t v, uint8_t fp) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	if (e < fp)
		e = fp;

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
    writechar(t + '0');
		if (e == fp)
      writechar('.');
	}
	while (e--);
}

/** write decimal digits from a long signed int
\param v number to send
\param fp number of decimal places to the right of the decimal point
*/
void write_int32_vf(void (*writechar)(uint8_t), int32_t v, uint8_t fp) {
	if (v < 0) {
    writechar('-');
		v = -v;
	}

	write_uint32_vf(writechar, v, fp);
}
