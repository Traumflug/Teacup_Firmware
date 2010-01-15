#include	<stddef.h>
#include	<stdio.h>
#include	<stdint.h>

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"serial.h"
#include	"lcd.h"

// write to lcd function for fdev_setup_stream
// static int lcd_putc_fdev(char c, FILE *stream)
// {
// 	lcd_putc(c);
// 	return 0;
// }

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

// static FILE lcdo = FDEV_SETUP_STREAM(lcd_putc_fdev, NULL, _FDEV_SETUP_WRITE);
static FILE serio = FDEV_SETUP_STREAM(serial_putc_fdev, serial_getc_fdev, _FDEV_SETUP_RW);

int main (void)
{
	// set up STDIN/OUT/ERR
	stdin = &serio;
	stdout = &serio;
	stderr = &serio;

	// set up serial
	serial_init(19200);

	sei();

	for (;;)
	{

	}
}
