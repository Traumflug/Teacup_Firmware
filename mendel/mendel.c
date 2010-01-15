#include	<stddef.h>
#include	<stdio.h>
#include	<stdint.h>

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"serial.h"

int serial_putc_fdev(char c, FILE *stream)
{
	serial_writechar((uint8_t) c);
	return 0;
}

int serial_getc_fdev(FILE *stream)
{
	for (;serial_rxchars() == 0;);
	return (int) serial_popchar();
}

static FILE serio = FDEV_SETUP_STREAM(serial_putc_fdev, serial_getc_fdev, _FDEV_SETUP_RW);

volatile uint32_t xpos;
volatile uint32_t ypos;
volatile uint32_t zpos;
volatile uint32_t edelta;

uint32_t xtarget;
uint32_t ytarget;
uint32_t ztarget;

uint16_t xspeed;
uint16_t yspeed;
uint16_t zspeed;

int main (void)
{
	// set up STDIN/OUT/ERR
	stdin = &serio;
	stdout = &serio;
	stderr = &serio;

	// set up serial
	serial_init();

	sei();

	for (;;)
	{

	}
}
