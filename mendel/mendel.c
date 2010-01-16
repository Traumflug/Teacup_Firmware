#include	<stddef.h>
#include	<stdint.h>
#include	<string.h>

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"serial.h"

#include	"dda.h"
#include	"gcode.h"

#include	"machine.h"

uint8_t	mb_head = 0;
uint8_t	mb_tail = 0;
DDA movebuffer[16];

uint8_t	option_bitfield;

struct {
	volatile int32_t	X;
	volatile int32_t	Y;
	volatile int32_t	Z;
	volatile int32_t	E;
	volatile int32_t	F;
} current_position = { 0, 0, 0, 0, 0 };

int main (void)
{
	// set up serial
	serial_init();

	// enable interrupts
	sei();

	for (;;)
	{
		for (;serial_rxchars() == 0;);
		uint8_t c = serial_popchar();

		scan_char(c);
	}
}
