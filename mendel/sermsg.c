#include	"sermsg.h"

#include	"serial.h"

// void serwrite_uint8(uint8_t v) {
// 	serwrite_uint32(v);
// }
//
// void serwrite_int8(int8_t v) {
// 	if (v < 0) {
// 		serial_writechar('-');
// 		v = -v;
// 	}
//
// 	serwrite_uint32(v);
// }
//
// void serwrite_uint16(uint16_t v) {
// 	serwrite_uint32(v);
// }
//
// void serwrite_int16(int16_t v) {
// 	if (v < 0) {
// 		serial_writechar('-');
// 		v = -v;
// 	}
//
// 	serwrite_uint32(v);
// }

void serwrite_uint32(uint32_t v) {
	uint8_t t = 0;
	if (v >= 1000000000) {
		for (t = 0; v >= 1000000000; v -= 1000000000, t++);
		serial_writechar(t + '0');
	}

	if (v >= 100000000) {
		for (t = 0; v >= 100000000; v -= 100000000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 10000000) {
		for (t = 0; v >= 10000000; v -= 10000000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 1000000) {
		for (t = 0; v >= 1000000; v -= 1000000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 100000) {
		for (t = 0; v >= 100000; v -= 100000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 10000) {
		for (t = 0; v >= 10000; v -= 10000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 1000) {
		for (t = 0; v >= 1000; v -= 1000, t++);
		serial_writechar(t + '0');
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 100) {
		t = v / 100;
		serial_writechar(t + '0');
		v -= (t * 100);
	}
	else if (t != 0)
		serial_writechar('0');

	if (v >= 10) {
		t = v / 10;
		serial_writechar(t + '0');
		v -= (t * 10);
	}
	else if (t != 0)
		serial_writechar('0');

	serial_writechar(v + '0');
}

void serwrite_int32(int32_t v) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint32(v);
}
