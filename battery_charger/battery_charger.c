#include	<stddef.h>
#include	<stdio.h>
#include	<stdint.h>

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"serial.h"
#include	"lcd.h"

// write to lcd function for fdev_setup_stream
static int lcd_putc_fdev(char c, FILE *stream)
{
	lcd_putc(c);
	return 0;
}

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

static FILE lcdo = FDEV_SETUP_STREAM(lcd_putc_fdev, NULL, _FDEV_SETUP_WRITE);
static FILE serio = FDEV_SETUP_STREAM(serial_putc_fdev, serial_getc_fdev, _FDEV_SETUP_RW);

int main (void)
{
	// set up LCD
	lcd_init(LCD_DISP_ON_CURSOR);

	lcd_puts_P("Starting...");

	// set up STDIN/OUT/ERR
	stdin = &serio;
	stdout = &lcdo;
	stderr = &lcdo;

	// set up serial
	serial_init(19200);

	sei();

	lcd_gotoxy(0, 0);
	fprintf(&lcdo, "Battery Charger OK");

	for (;;)
	{
		# switch off all currents
		# check battery presence
		# for (each battery) {
			# check ambient temperature
			# check battery temperature
			# calculate delta T
			# check battery voltage
			# calculate delta V
			# check charge cycle location
			# apply suitable current
			# update screen/indicators
		# }
		# wait
	}
}
