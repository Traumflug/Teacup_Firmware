#include	<stddef.h>
#include	<stdio.h>
#include	<stdint.h>

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"serial.h"
#include	"lcd.h"

#include	"arduino.h"

// *** pin assignments ***

// step input
#define		PIN_STEP			PIN_AIO2
#define		PORT_STEP			WPORT_AIO2
#define		READ_STEP			RPORT_AIO2
#define		DDR_STEP			DDR_AIO2

// direction input
#define		PIN_DIR				PIN_AIO3
#define		PORT_DIR			WPORT_AIO3
#define		READ_DIR			RPORT_AIO3
#define		DDR_DIR				DDR_AIO3

// step output
#define		PIN_STEPOUT		PIN_AIO4
#define		PORT_STEPOUT	WPORT_AIO4
#define		READ_STEPOUT	RPORT_AIO4
#define		DDR_STEPOUT		DDR_AIO4

// direction output
#define		PIN_DIROUT		PIN_AIO5
#define		PORT_DIROUT		WPORT_AIO5
#define		READ_DIROUT		RPORT_AIO5
#define		DDR_DIROUT		DDR_AIO5

// *** machine-specific constants ***

// 1/2 step, NSTEPPING=2, for 1/4 step, NSTEPPING = 4 etc
#define		NSTEPPING			16

// FULL steps per mm (calculate from 200 steps/rev)
#define		FULL_STEPS_PER_MM	5

// calculations
#define		PRESCALER			256
#define		STEPS_PER_MM	(FULL_STEPS_PER_MM * NSTEPPING)

// units
#define		MM						* STEPS_PER_MM
#define		MM_PER_SEC		* STEPS_PER_MM

#define		US						* F_CPU / 1000000 / PRESCALER
#define		MS						* F_CPU / 1000 / PRESCALER
#define		S							* F_CPU / 1 / PRESCALER

// *** tunables ***

#define		SPEED					(15 MM_PER_SEC)

// recalculations - don't touch!
#define		STEP_TIME			F_CPU / SPEED / PRESCALER
#define		MIN_STEP_TIME	F_CPU / 1000 / PRESCALER

// utilities
#define		MASK(a)			(1 << a)
#define		PORT_OUT_MASK	(0xF << PIN_LSB_OUT)
#define		abs(a)			(((a) >= 0)?(a):-(a))

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

volatile int32_t pos;
volatile int32_t npos;
volatile uint16_t speed;

// next step interrupt
ISR(TIMER1_COMPA_vect) {
	// toggle "L" led
	PINB = MASK(PB5);

	if (READ_STEPOUT & MASK(PIN_STEPOUT)) {
		if (npos > pos)
			PORT_DIROUT |= MASK(PIN_DIROUT);
		else if (npos < pos)
			PORT_DIROUT &= ~MASK(PIN_DIROUT);
		else
			TIMSK1 &= ~MASK(OCIE1A);

		PORT_STEPOUT &= ~MASK(PIN_STEPOUT);
	}
	else {
		PORT_STEPOUT |= MASK(PIN_STEPOUT);

		if (READ_DIROUT & MASK(PIN_DIROUT))
			pos++;
		else
			pos--;
	}

	// update speed
	OCR1A = speed;
}

void startstep(void) {
	if ((TIMSK1 & MASK(OCIE1A)) == 0)
	{
		OCR1A = speed;
		TCNT1 = 0;
	}

	// it's possible that the mask is enabled during the check above, but disabled by the time we get here - always set it to avoid a race condition
	TIMSK1 |= MASK(OCIE1A);
}

// main, where it all happens
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

	// variables
	pos = 0;
	uint8_t stepdebounce = 0;
	uint16_t spinner = 0;
	int r;
	int32_t	rv;

	// setup inputs
	DDR_STEP &= ~MASK(PIN_STEP);
	DDR_DIR &= ~MASK(PIN_DIR);

	// pull-ups
	PORT_STEP |= MASK(PIN_STEP);
	PORT_DIR |= MASK(PIN_DIR);

	// outputs to motor controller
	PORT_STEPOUT &= ~MASK(PIN_STEPOUT);
	PORT_DIROUT &= ~MASK(PIN_DIROUT);
	DDR_STEPOUT |= MASK(PIN_STEPOUT);
	DDR_DIROUT |= MASK(PIN_DIROUT);

	// setup timer 1 (step timer)
	TCCR1A = 0;
	TCCR1B = MASK(WGM12);
	#if PRESCALER == 1
		TCCR1B |= MASK(CS10);
	#elif PRESCALER == 8
		TCCR1B |= MASK(CS11);
	#elif PRESCALER == 64
		TCCR1B |= MASK(CS11) | MASK(CS10);
	#elif PRESCALER == 256
		TCCR1B |= MASK(CS12);
	#elif	PRESCALER == 1024
		TCCR1B |= MASK(CS12) | MASK(CS10);
	#else
		#error Invalid PRESCALER value: must be one of 1, 8, 64, 256 or 1024
	#endif

	// disable interrupt
	TIMSK1 = 0;

	// set speed: divide by 2 because we toggle each interrupt rather than full pulse
	speed = STEP_TIME / 2;
	OCR1A = speed;

	// enable interrupts
	sei();

	// main loop start
	lcd_gotoxy(0, 0);
	fprintf(&lcdo, "Stepper OK ");

	// main loop
	for (;;)
	{
		// check logic inputs
		if ((READ_STEP & MASK(PIN_STEP)) == 0) {
			if (stepdebounce >= 128) {
				if (stepdebounce == 128) {
					if (READ_DIR & MASK(PIN_DIR))
						npos++;
					else
						npos--;
					stepdebounce++;
				}
			}
			else
				stepdebounce++;
		}
		else
			stepdebounce = 0;

		// check serial input
		if (serial_rxchars()) {
			uint8_t c = getchar();
			switch (c) {
				case '>':
					npos++;
					break;
				case '<':
					npos--;
					break;
				case '?':
					fprintf(&serio, "pos:%li\n", pos);
					break;
				case '+':
					r = scanf("%li", &rv);
					if (r == 0)
						npos++;
					else
						npos += rv;
					break;
				case '-':
					r = scanf("%li", &rv);
					if (r == 0)
						npos--;
					else
						npos -= rv;
					break;
				case 'g':
					if (scanf("%li", &rv))
						npos = rv;
					break;
				case 's':
					if (scanf("%li", &rv))
						speed = rv;
					break;
				case 'h':
					npos = 0;
					break;
				case 'R':
					npos = pos = 0;
					break;
			}
		}

		if ((npos != pos) && ((TIMSK1 & MASK(OCIE1A)) == 0))
			startstep();

		if (((spinner++) & 0x0FFF) == 0) {
			lcd_clrscr();
			printf("p:%7li", pos);
			lcd_gotoxy(8, 0);
			printf("s:%i", speed);
			lcd_gotoxy(0, 1);
			printf("t:%7li", npos);
			lcd_gotoxy(8, 1);
			// printf("p:%i", PORT_STEP);
			// printf("%i %i", step1, step2);
			printf("%02X", READ_STEP);
		}
	}
}
