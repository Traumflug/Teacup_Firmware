#include	"sermsg.h"

#include	"serial.h"

void serwrite_uint8(uint8_t v) {
	uint8_t t;
	if (v > 100) {
		t = v / 100;
		serial_writechar(t + '0');
		v -= t;
	}
	if (v > 10) {
		t = v / 10;
		serial_writechar(t + '0');
		v -= t;
	}
	serial_writechar(v + '0');
}

void serwrite_uint16(uint16_t v) {
	uint16_t t;
	if (v > 10000) {
		t = v / 10000;
		serial_writechar(t + '0');
		v -= t;
	}
	if (v > 1000) {
		t = v / 1000;
		serial_writechar(t + '0');
		v -= t;
	}
	if (v > 100) {
		t = v / 100;
		serial_writechar(t + '0');
		v -= t;
	}
	if (v > 10) {
		t = v / 10;
		serial_writechar(t + '0');
		v -= t;
	}
	serial_writechar(v + '0');
}
