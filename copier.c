#ifdef	COPIER

#include	"copier.h"

#include	<avr/pgmspace.h>
#include	<avr/boot.h>

#include	"arduino.h"
#include	"delay.h"

uint32_t copier_xchange(uint32_t cmd) {
	uint32_t r = 0, c = cmd;
	uint8_t i = 0;

	for (i = 0; i < 32; i++) {
		WRITE(COPIER_MOSI, c & 0x80000000); delay_us(5);
		c <<= 1;
		WRITE(COPIER_SCK, 1); delay_us(5);
		r = (r << 1) | (READ(COPIER_MISO)?1:0);
		WRITE(COPIER_SCK, 0);
	}

	delay_us(5);

	return r;
}

void init_chip(void);
void init_chip() {
	do {
		WRITE(COPIER_SCK, 0);
		// power up
		WRITE(COPIER_RESET, 1);
		delay_ms(10);
		WRITE(COPIER_RESET, 0);
		delay_ms(10);
		// hopefully enter programming mode
	} while ((copier_xchange(CMD_PROGRAMMING_ENABLE) & (CMD_PROGRAMMING_ENABLE >> 8)) != (CMD_PROGRAMMING_ENABLE >> 8));
}

void copy() {
	// initialise
	WRITE(COPIER_RESET, 0); SET_OUTPUT(COPIER_RESET);
	WRITE(COPIER_SCK, 0); SET_OUTPUT(COPIER_SCK);
	WRITE(COPIER_MOSI, 0); SET_OUTPUT(COPIER_MOSI);
	SET_INPUT(COPIER_MISO); WRITE(COPIER_MISO, 1);

	delay_ms(50);

	init_chip();

	// verify device signature- should be same as current chip since we haven't the space for the functionality necessary to program anything else
	if ((copier_xchange(CMD_READ_SIGNATURE | 0x00) & 0xFF) != SIGNATURE_0)
		return;
	if ((copier_xchange(CMD_READ_SIGNATURE | 0x01) & 0xFF) != SIGNATURE_1)
		return;
	if ((copier_xchange(CMD_READ_SIGNATURE | 0x02) & 0xFF) != SIGNATURE_2)
		return;

	// erase chip
	copier_xchange(CMD_CHIP_ERASE);
	delay_ms(15); //minimum is 9.0ms

	// re-initialise
	init_chip();

	uint8_t f;
	// copy fuses
	// first low byte
	f = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
	copier_xchange(CMD_WRITE_FUSE_BITS | f);

	// high byte
	f = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
	copier_xchange(CMD_WRITE_FUSE_HIGH_BITS | f);

	// extended byte
	f = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
	copier_xchange(CMD_WRITE_FUSE_EXTENDED_BITS | f);

	// start copying flash
	uint16_t i;
	for (i = 0; i < (FLASHEND / 2); i += (SPM_PAGESIZE / 2)) {
		uint8_t j;
		for (j = 0; j < (SPM_PAGESIZE / 2); j++) {
			uint16_t w = pgm_read_word_near((i | j) << 1);
			copier_xchange(CMD_LOAD_PROGMEM_LOW_BYTE | (j << 8) | (w & 0xFF));
			copier_xchange(CMD_LOAD_PROGMEM_HIGH_BYTE | (j << 8) | (w >> 8));
		}
		copier_xchange(CMD_WRITE_PROGMEM_PAGE | ((i / (SPM_PAGESIZE / 2)) << 8));
		delay_ms(10); //minimum is 4.5ms
	}

	// reset
	delay_ms(10);
	SET_INPUT(MOSI);
	SET_INPUT(SCK);
	SET_INPUT(COPIER_RESET);
}

#endif	/* COPIER */
