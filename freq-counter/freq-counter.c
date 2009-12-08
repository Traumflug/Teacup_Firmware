#include	<stddef.h>
#include	<stdio.h>
#include	<stdint.h>

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"ringbuffer.h"
#include	"serial.h"
#include	"lcd.h"

volatile uint16_t	last_count;
volatile uint32_t	freq;
volatile uint8_t	jiffies;

char *teststring = "chickens\n";

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

	// set up outputs
	PORTB &= ~(1 << PB5);
	DDRB |= (1 << PB5);

	// set up STDIN/OUT/ERR
	stdin = &serio;
	stdout = &lcdo;
	stderr = &lcdo;

	// set up timer 0
	// prescale 1024 = 15625khz
	// so set clear on compare mode and set OCRA to 156 for a ~100Hz (actually 100.160256Hz) rate
	// use interrupt to write value into a ringbuffer
	OCR0A = F_CPU / 1024 / 100;
	TCCR0A = (1 << WGM01) | (0 << WGM00);
	TCCR0B = (1 << CS02) | (0 << CS01) | (1 << CS00);
	TIMSK0 = (1 << OCIE0A);

	// set up timer 1
	// no prescale, use external input for counter, no compare, no interrupts
	TCCR1A = 0;
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (1 << CS11) | (0 << CS10);
	TCCR1C = 0;

	// set up globals
	last_count = 0;
	jiffies = 0;

	// set up serial
	serial_init(19200);

	// set up pull-up on T1 (PD5)
	PORTD |= (1 << PD5);

	sei();

	lcd_gotoxy(0, 0);
	fprintf(&lcdo, "Freq. Counter OK");

	for (;;)
	{
		if (jiffies >= 50)
		{
			lcd_gotoxy(0, 1);
			fprintf(&lcdo, "F: %liHz", freq << 1);
			if (serial_txchars() == 0)
			{
				fprintf(&serio, "%liHz\n", freq << 1);
			}
			PORTB ^= (1 << PB5);
			jiffies = 0;
			freq = 0;
		}
	}
}

ISR(TIMER0_COMPA_vect)
{
	last_count = TCNT1;
	TCNT1 = 0;
	freq += last_count;
	jiffies++;
}
