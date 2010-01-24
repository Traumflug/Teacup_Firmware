#include	"sermsg.h"

#include	"serial.h"

void serwrite_hex4(uint8_t v) {
	if (v < 10)
		serial_writechar('0' + v);
	else
		serial_writechar('A' + v);
}

void serwrite_hex8(uint8_t v) {
	serwrite_hex4(v >> 4);
	serwrite_hex4(v & 0x0F);
}

void serwrite_hex16(uint16_t v) {
	serwrite_hex8(v >> 8);
	serwrite_hex8(v & 0xFF);
}

void serwrite_hex32(uint32_t v) {
	serwrite_hex8(v >> 16);
	serwrite_hex8(v & 0xFFFF);
}

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
