#include	"sermsg.h"

/** \file sermsg.c
	\brief primitives for sending numbers over the serial link
*/

#include	"serial.h"

/** write a single hex digit
	\param v hex digit to write, higher nibble ignored
*/
void serwrite_hex4(uint8_t v) {
	v &= 0xF;
	if (v < 10)
		serial_writechar('0' + v);
	else
		serial_writechar('A' - 10 + v);
}

/** write a pair of hex digits
	\param v byte to write. One byte gives two hex digits
*/
void serwrite_hex8(uint8_t v) {
	serwrite_hex4(v >> 4);
	serwrite_hex4(v & 0x0F);
}

/** write four hex digits
	\param v word to write
*/
void serwrite_hex16(uint16_t v) {
	serwrite_hex8(v >> 8);
	serwrite_hex8(v & 0xFF);
}

/** write eight hex digits
	\param v long word to write
*/
void serwrite_hex32(uint32_t v) {
	serwrite_hex16(v >> 16);
	serwrite_hex16(v & 0xFFFF);
}

/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
const uint32_t powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/** write decimal digits from a long unsigned int
	\param v number to send
*/
void serwrite_uint32(uint32_t v) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
		serial_writechar(t + '0');
	}
	while (e--);
}

/** write decimal digits from a long signed int
	\param v number to send
*/
void serwrite_int32(int32_t v) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint32(v);
}

/** write decimal digits from a long unsigned int
\param v number to send
*/
void serwrite_uint32_vf(uint32_t v, uint8_t fp) {
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
		serial_writechar(t + '0');
		if (e == fp)
			serial_writechar('.');
	}
	while (e--);
}

/** write decimal digits from a long signed int
\param v number to send
*/
void serwrite_int32_vf(int32_t v, uint8_t fp) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint32_vf(v, fp);
}
