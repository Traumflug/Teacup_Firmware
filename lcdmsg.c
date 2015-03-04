
#ifdef LCD

#include	"lcdmsg.h"

/** \file lcdmsg.c
	\brief primitives for sending numbers over the serial link
*/

#include	"SimpleLCD.h"

/** write a single hex digit
	\param v hex digit to write, higher nibble ignored
*/
void lcdwrite_hex4(uint8_t v) {
        char ch;
	v &= 0xF;
	if (v < 10){
                ch='0'+v;
		lcdWriteChar(&ch);
        }
	else {
                ch='A' - 10 + v;
		lcdWriteChar(&ch);
}
}

/** write a pair of hex digits
	\param v byte to write. One byte gives two hex digits
*/
void lcdwrite_hex8(uint8_t v) {
	lcdwrite_hex4(v >> 4);
	lcdwrite_hex4(v & 0x0F);
}

/** write four hex digits
	\param v word to write
*/
void lcdwrite_hex16(uint16_t v) {
	lcdwrite_hex8(v >> 8);
	lcdwrite_hex8(v & 0xFF);
}

/** write eight hex digits
	\param v long word to write
*/
void lcdwrite_hex32(uint32_t v) {
	lcdwrite_hex16(v >> 16);
	lcdwrite_hex16(v & 0xFFFF);
}

/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
const uint32_t lcdpowers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/** write decimal digits from a long unsigned int
	\param v number to send
*/
void lcdwrite_uint32(uint32_t v) {
	uint8_t e, t;
        char ch;
	for (e = 9; e > 0; e--) {
		if (v >= lcdpowers[e])
			break;
	}

	do
	{
		for (t = 0; v >= lcdpowers[e]; v -= lcdpowers[e], t++);
                ch=t + '0';
                lcdWriteChar(&ch);
	}
	while (e--);
}

/** write decimal digits from a long signed int
	\param v number to send
*/
void lcdwrite_int32(int32_t v) {
        char ch;
	if (v < 0) {
                ch='-';
		lcdWriteChar(&ch);
		v = -v;
	}

	lcdwrite_uint32(v);
}

/** write decimal digits from a long unsigned int
\param v number to send
\param fp number of decimal places to the right of the decimal point
*/
void lcdwrite_uint32_vf(uint32_t v, uint8_t fp) {
	uint8_t e, t;
        char ch;

	for (e = 9; e > 0; e--) {
		if (v >= lcdpowers[e])
			break;
	}

	if (e < fp){
	    e = fp;
        }

	do
	{
		for (t = 0; v >= lcdpowers[e]; v -= lcdpowers[e], t++);
                ch=t + '0';
                lcdWriteChar(&ch);
		if (e == fp)
                {
                        ch='.';
			lcdWriteChar(&ch);
                }
                e--;
	}
	while (e);
}

/** write decimal digits from a long signed int
\param v number to send
\param fp number of decimal places to the right of the decimal point
*/
void lcdwrite_int32_vf(int32_t v, uint8_t fp) {
        char ch;
	if (v < 0) {
                ch='-';
		lcdWriteChar(&ch);
		v = -v;
	}

	lcdwrite_uint32_vf(v, fp);
}
#endif /* LCD */
