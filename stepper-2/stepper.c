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

// outputs (PWM) - dir lines only below, en lines MUST connect to OCR0A/B (DIO 5/6)
#define		PIN_DIR1			PIN_AIO0
#define		PORT_DIR1			WPORT_AIO0
#define		DDR_DIR1			DDR_AIO0

#define		PIN_DIR2			PIN_AIO1
#define		PORT_DIR2			WPORT_AIO1
#define		DDR_DIR2			DDR_AIO1


// *** machine-specific constants ***

// 1/2 step, NSTEPPING=2, for 1/4 step, NSTEPPING = 4 etc
#define		NSTEPPING			8
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

// *** step table ***
// sinstepi MUST satisfy 2^n for integer values of n where n = 2 for 1/2 step, 4 for 1/4 step etc
// generate with:
// perl -e 'my $n = 4; my @st; for (0..$n) { push @st, sprintf "%i", sin($_ * 90 * 3.1415926535897932384626433832795029 * 2 / 360 / $n) * 255 }; print "#define sinstepi $n\nstatic uint8_t sintable[sinstepi + 1] = { "; print join ", ", @st; print " };\n";';

#if   NSTEPPING == 1
#define sinstepi 1
static const uint8_t sintable[sinstepi + 1] = { 0, 255 };

#elif NSTEPPING == 2
#define sinstepi 2
static const uint8_t sintable[sinstepi + 1] = { 0, 180, 255 };

#elif NSTEPPING == 4
#define sinstepi 4
static uint8_t sintable[sinstepi + 1] = { 0, 97, 180, 235, 255 };

#elif NSTEPPING == 8
#define sinstepi 8
static uint8_t sintable[sinstepi + 1] = { 0, 49, 97, 141, 180, 212, 235, 250, 255 };

#elif NSTEPPING == 16
#define sinstepi 16
static uint8_t sintable[sinstepi + 1] = { 0, 24, 49, 74, 97, 120, 141, 161, 180, 197, 212, 224, 235, 244, 250, 253, 255 };

#elif NSTEPPING == 32
#define sinstepi 32
static uint8_t sintable[sinstepi + 1] = { 0, 12, 24, 37, 49, 61, 74, 85, 97, 109, 120, 131, 141, 151, 161, 171, 180, 188, 197, 204, 212, 218, 224, 230, 235, 240, 244, 247, 250, 252, 253, 254, 255 };

#else
#error Invalid NSTEPPING value

#endif

// recalculations - don't touch!
#define		STEP_TIME			F_CPU / SPEED / PRESCALER
#define		MIN_STEP_TIME	F_CPU / 1000 / PRESCALER
#define sinstepi2   (sinstepi * 2)
#define sinstepi3   (sinstepi * 3)
#define sinstepi4   (sinstepi * 4)
#define	sinsteplast (sinstepi4 - 1)

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
volatile uint16_t speed_sync;
volatile uint8_t superstep; // for disabling microstep during high speed runs
volatile uint8_t superstep_sync;
volatile int step1;
volatile int step2;
// uint8_t power0;
// uint8_t power1;

// integer sine approximation
int sinstep(uint8_t sequence) {
	while (sequence >= sinstepi4)
		sequence -= sinstepi4;
	if (sequence < (sinstepi + 1))
		return sintable[sequence];
	if ((sequence >= (sinstepi + 1)) && (sequence < (sinstepi2 + 1)))
		return sintable[sinstepi2 - sequence];
	if ((sequence >= (sinstepi2 + 1)) && (sequence < (sinstepi3 + 1)))
		return -sintable[sequence - sinstepi2];
	//if ((sequence >= (sinstepi3 + 1)) && (sequence < (sinstepi4 + 1)))
		return -sintable[sinstepi4 - sequence];
}

// generate appropriate stepper signals for a sequence number
void stepperseq(uint8_t sequence) {
	step1 = sinstep(sequence);
	step2 = sinstep(sequence + sinstepi);

	// set directions
	if (step1 >= 0)
		PORT_DIR1 |= MASK(PIN_DIR1);
	else {
		PORT_DIR1 &= ~MASK(PIN_DIR1);
	}

// 	PORT_DIR1 = (PORT_DIR1 & ~MASK(PIN_DIR1)) | ((((step1 >= 0)?255:0) ^ wx) & MASK(PIN_DIR1));

	if (step2 >= 0)
		PORT_DIR2 |= MASK(PIN_DIR2);
	else {
		PORT_DIR2 &= ~MASK(PIN_DIR2);
	}

// 	PORT_DIR2 = (PORT_DIR2 & ~MASK(PIN_DIR2)) | ((((step2 >= 0)?255:0) ^ wx) & MASK(PIN_DIR2));

	// set power
	TCNT0 = 0xFD;
	OCR0A = ((uint8_t) abs(step1));
	OCR0B = ((uint8_t) abs(step2));
}

// // PWM reset interrupt
// ISR(TIMER0_OVF_vect) {
// 	// now that our counter is at zero, load new power levels
// 	OCR0A = power0;
// 	OCR0B = power1;
// }

// next step interrupt
ISR(TIMER1_COMPA_vect) {
	uint8_t i;

	// toggle "L" led
	PINB = MASK(PB5);

	// update position
	if (npos > pos)
		pos += MASK(superstep_sync);
	else if (npos < pos)
		pos -= MASK(superstep_sync);

	// write new position
	i = pos & sinsteplast;
	stepperseq(i);
	// if we're at a sync point and we're changing microstep rate
	if ((i & (sinstepi2 - 1)) == 0)
		// do the change now
		superstep_sync = superstep;
	// update speed
	OCR1A = speed << superstep_sync;
}

void startstep(void) {
	if ((TIMSK1 & MASK(OCIE1A)) == 0)
	{
		OCR1A = speed;
// 		while ((OCR1A < MIN_STEP_TIME) && (superstep < sinstepi)) {
// 			OCR1A <<= 1;
// 			superstep <<= 1;
// 		}
// 		while (((OCR1A > (MIN_STEP_TIME * 2)) && (superstep > 1)) || (abs(npos - pos) < superstep)) {
// 			OCR1A >>= 1;
// 			superstep >>= 1;
// 		}
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
	int rs;

	// setup inputs
	DDR_STEP &= ~MASK(PIN_STEP);
	DDR_DIR &= ~MASK(PIN_DIR);

	// pull-ups
	PORT_STEP |= MASK(PIN_STEP);
	PORT_DIR |= MASK(PIN_DIR);

	// direction pins to h-bridge
	DDR_DIR1 |= MASK(PIN_DIR1);
	DDR_DIR2 |= MASK(PIN_DIR2);
	// enable pins to h-bridge - must be DIO5/6 for PWM operation
	DDR_DIO5 |= MASK(PIN_DIO5);
	DDR_DIO6 |= MASK(PIN_DIO6);

	// setup timer 0 (PWM timer)
	TCCR0A = MASK(COM0A1) | MASK(COM0B1) | MASK(WGM01) | MASK(WGM00); // enable PWM output pins (DIO5/6), fast PWM
	TCCR0B = MASK(CS00);	// prescaler = 1 (max speed)

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
	// set speed
	speed = STEP_TIME;
	OCR1A = speed << superstep;

	// initialize stepper drive
	//PORT_OUT = (PORT_OUT & ~PORT_OUT_MASK) | ((steps[0] ^ wmod) << PIN_LSB_OUT);
	stepperseq(0);

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
			if (stepdebounce >= 32) {
				if (stepdebounce == 32) {
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
					r = scanf("%li", &rv);
					if (r != 0)
						npos = rv;
					break;
				case 's':
					r = scanf("%li", &rv);
					if (r != 0)
						speed = rv;
					break;
				case 'h':
					npos = 0;
					break;
				case 'x':
					r = scanf("%i", &rs);
					if (r != 0)
						superstep = rs;
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
			printf("%i %i", step1, step2);
			// printf("%02X", READ_STEP);
		}
	}
}
