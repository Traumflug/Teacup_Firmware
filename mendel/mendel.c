#include	<stddef.h>
// #include	<stdio.h>
#include	<stdint.h>

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"serial.h"

/*
	machine variables
*/
#define	AXIS_COUNT 5

//														steps per rev * mm per tooth * teeth per rev
#define	STEPS_PER_MM					(3200 * 4.77 * 16)

#define	MAX_MM_PER_SEC				40
#define	MAX_IMMED_MM_PER_SEC	20
#define	ACCEL									10


/*
	state machine variables
*/

uint8_t linebuffer[64];

uint8_t state = 0;
#define STATE_WAIT_FOR_COMMAND				0
#define STATE_WAIT_FOR_COMMAND_DIGIT	1
#define STATE_WAIT_FOR_DATA						2
#define	STATE_WAIT_FOR_DATA_DIGIT			3
uint8_t command;
uint8_t command_digit;
uint8_t axis;

// volatile uint32_t xpos = 0;
// volatile uint32_t ypos = 0;
// volatile uint32_t zpos = 0;
// volatile uint32_t edelta = 0;
//
// uint32_t xtarget = 0;
// uint32_t ytarget = 0;
// uint32_t ztarget = 0;
//
// uint16_t xspeed = 0;
// uint16_t yspeed = 0;
// uint16_t zspeed = 0;

struct axis {
	volatile uint32_t	pos;
	int32_t						target;
	uint32_t					newtarget_mantissa;
	uint8_t						newtarget_exp;
	uint8_t						newtarget_sign;
	uint16_t					speed;
} axes[AXIS_COUNT];

uint8_t axis_char_to_id(uint8_t c) {
	if (c >= 'X')
		return c - 'X';
	if (c == 'E')
		return 3;
	if (c == 'T')
		return 4;
	return 255;
}

int main (void)
{
	// set up STDIN/OUT/ERR
// 	stdin = &serio;
// 	stdout = &serio;
// 	stderr = &serio;

	// set up serial
	serial_init();

	// enable interrupts
	sei();

	for (;;)
	{
		for (;serial_rxchars() == 0;);
		uint8_t c = serial_popchar();
		if (c >= 'a' && c <= 'z')
			c &= ~32;
		// preprocess
		switch (state) {
			case STATE_WAIT_FOR_COMMAND_DIGIT:
				if (c == 'G' || c == 'M')
					state = STATE_WAIT_FOR_COMMAND;
				break;
			case STATE_WAIT_FOR_DATA_DIGIT:
				if (axis_char_to_id(c) < 255)
					state = STATE_WAIT_FOR_DATA;
				break;
		}
		// actuate
		switch (state) {
			case STATE_WAIT_FOR_COMMAND:
				if (c == 'G' || c == 'M') {
					command = c;
					command_digit = 0;
					state = STATE_WAIT_FOR_COMMAND_DIGIT;
				}
				break;
			case STATE_WAIT_FOR_COMMAND_DIGIT:
				if (c >= '0' && c <= '9') {
					command_digit = (command_digit * 10) + (c - '0');
				}
				else {
					state = STATE_WAIT_FOR_DATA;
				}
				break;
			case STATE_WAIT_FOR_DATA:
				if (axis_char_to_id(c) < 255) {
					axis = axis_char_to_id(c);
					axes[axis].newtarget_mantissa = 0;
					axes[axis].newtarget_exp = 0;
					state = STATE_WAIT_FOR_DATA_DIGIT;
				}
				break;
			case STATE_WAIT_FOR_DATA_DIGIT:
				if (c == '-') {
					axes[axis].newtarget_sign = 1;
				}
				if (c >= '0' && c <= '9') {
					axes[axis].newtarget_mantissa = (axes[axis].newtarget_mantissa * 10) + (c - '0');
					if (axes[axis].newtarget_exp)
						axes[axis].newtarget_exp++;
				}
				if (c == '.') {
					axes[axis].newtarget_exp = 1;
				}
				break;
		}
		if (c == 13) {
			uint8_t i;
			for (i = 0; i < AXIS_COUNT; i++) {
				uint32_t n = axes[i].newtarget_mantissa;
			}
		}
	}
}
