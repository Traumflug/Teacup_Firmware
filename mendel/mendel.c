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
#define	AXIS_HOMING_COUNT	3

//														steps per rev * mm per tooth * teeth per rev
#define	STEPS_PER_MM					(3200 * 4.77 * 16)
#define	STEPS_PER_IN					(STEPS_PER_MM * 25.4)

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

uint8_t	option_bitfield;
#define	OPTION_RELATIVE						1
#define	OPTION_SYNCHRONISE				2
#define	OPTION_HOME_WHEN_COMPLETE	4
#define	OPTION_UNIT_INCHES				8

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
	uint8_t						newtarget_opt;
#define	newtarget_opt_sign	0x80
#define	newtarget_opt_set		0x40
#define	newtarget_opt_exp		0x3F
	uint16_t					speed;
} axes[AXIS_COUNT];

uint8_t axis_char_to_id(uint8_t c) {
	if (c >= 'X' && c <= 'Z')
		return c - 'X';
	if (c == 'E')
		return 3;
	if (c == 'F')
		return 4;
	return 255;
}

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
					axes[axis].newtarget_opt = 0;
					state = STATE_WAIT_FOR_DATA_DIGIT;
				}
				break;
			case STATE_WAIT_FOR_DATA_DIGIT:
				if (c == '-') {
					axes[axis].newtarget_opt |= newtarget_opt_sign;
				}
				if (c >= '0' && c <= '9') {
					axes[axis].newtarget_mantissa = (axes[axis].newtarget_mantissa * 10) + (c - '0');
					if (axes[axis].newtarget_opt & newtarget_opt_exp)
						axes[axis].newtarget_opt++;
				}
				if (c == '.') {
					axes[axis].newtarget_opt |= 1;
				}
				break;
		}
		if (c == 13) {
			if (command == 'G') {
				uint8_t i;
				switch (command_digit) {
					//	G30 - go home via point
					case 30:
						option_bitfield |= OPTION_HOME_WHEN_COMPLETE;
					// 	G0 - rapid, unsynchronised motion
					case 0:
						option_bitfield &= ~OPTION_SYNCHRONISE;
						break;
					//	G1 - synchronised motion
					case 1:
						option_bitfield |= OPTION_SYNCHRONISE;
						break;
					//	G2 - Arc Clockwise
					//	G3 - Arc Counter-clockwise
					//	G4 - Dwell
					//	G20 - inches as units
					//	G21 - mm as units
					//	G28 - go home
					case 28:
						for (i = 0; i < AXIS_HOMING_COUNT; i++) {
							option_bitfield &= ~OPTION_SYNCHRONISE;
							axes[i].target = 0;
						}
						break;
					//	G90 - absolute positioning
					case 90:
						option_bitfield &= ~OPTION_RELATIVE;
						break;
					//	G91 - relative positioning
					case 91:
						option_bitfield |= OPTION_RELATIVE;
						break;
					//	G92 - set home
					case 92:
						for (i = 0; i < AXIS_HOMING_COUNT; i++) {
							axes[i].pos = axes[i].target = 0;
						}
						break;
					// TODO: spit an error
				}
			}
			else if (command == 'M') {
				switch (command_digit) {
					// TODO: spit an error
				}
			}
			// update axes;
			uint8_t i;
			for (i = 0; i < AXIS_COUNT; i++) {
				if (axes[i].newtarget_opt & newtarget_opt_set) {
					float n = axes[i].newtarget_mantissa;
					uint8_t exp = axes[i].newtarget_opt & newtarget_opt_exp;
					if (axes[i].newtarget_opt & newtarget_opt_sign)
						n = -n;
					if (exp == 1)
						n /= 10;
					if (exp == 2)
						n /= 100;
					if (exp == 3)
						n /= 1000;
					if (exp == 4)
						n /= 10000;
					if (exp == 5)
						n /= 100000;
					if (option_bitfield & OPTION_UNIT_INCHES)
						axes[i].target = n * STEPS_PER_IN;
					else
						axes[i].target = n * STEPS_PER_MM;
				}
			}
		}
	}
}
