#include	"gcode.h"

#include	<string.h>

#include	"machine.h"
#include	"dda.h"
#include	"serial.h"

extern uint8_t	option_bitfield;

decfloat read_digit;

#define	PI	3.1415926535

/*
	utility functions
*/

uint8_t indexof(uint8_t c, char *string) {
	uint8_t i;
	for (i = 0;string[i];i++) {
		if (c == string[i])
			return i;
	}
	return 255;
}

int32_t	decfloat_to_int(decfloat *df, int32_t multiplicand, int32_t denominator) {
	int32_t	r = df->mantissa;
	uint8_t	e = df->exponent - 1;

	if (multiplicand != 1)
		r *= multiplicand;
	if (denominator != 1)
		r /= denominator;

	while (e >= 5) {
		r /= 100000;
		e -= 5;
	}

	if (e == 1)
		r /= 10;
	else if (e == 2)
		r /= 100;
	else if (e == 3)
		r /= 1000;
	else if (e == 4)
		r /= 10000;

	return r;
}

/*
	public functions
*/

void scan_char(uint8_t c) {
	static uint8_t last_field = 0;
// 	static uint32_t mantissa = 0;
// 	static uint8_t exp = 0;
	static GCODE_COMMAND next_target = { 0, 0, 0, 0, { 0, 0, 0, 0, 0 } };

	// uppercase
	if (c >= 'a' && c <= 'z')
		c &= ~32;

	// process field
	if (indexof(c, "GMXYZEF\n") != 255) {
		if (last_field) {
			switch (last_field) {
				case 'G':
					next_target.G = read_digit.mantissa;
					break;
				case 'M':
					next_target.M = read_digit.mantissa;
					break;
				case 'X':
					next_target.target.X = decfloat_to_int(&read_digit, STEPS_PER_MM_X, 1);
					break;
				case 'Y':
					next_target.target.Y = decfloat_to_int(&read_digit, STEPS_PER_MM_Y, 1);
					break;
				case 'Z':
					next_target.target.Z = decfloat_to_int(&read_digit, STEPS_PER_MM_Z, 1);
					break;
				case 'E':
					next_target.target.E = decfloat_to_int(&read_digit, STEPS_PER_MM_E, 1);
					break;
				case 'F':
					// just save an integer value for F, we need move distance and n_steps to convert it to a useful value, so wait until we have those to convert it
					next_target.target.F = read_digit.mantissa;
					break;
			}
			read_digit.sign = 0;
			read_digit.mantissa = 0;
			read_digit.exponent = 0;
		}
		last_field = c;
		switch (c) {
			case 'G':
				next_target.seen |= SEEN_G;
				break;
			case 'M':
				next_target.seen |= SEEN_M;
				break;
			case 'X':
				next_target.seen |= SEEN_X;
				break;
			case 'Y':
				next_target.seen |= SEEN_Y;
				break;
			case 'Z':
				next_target.seen |= SEEN_Z;
				break;
			case 'E':
				next_target.seen |= SEEN_E;
				break;
			case 'F':
				next_target.seen |= SEEN_F;
				break;
			case '\n':
				// process
				process_gcode_command(&next_target);

				// save options
				option_bitfield = next_target.option;

				// reset variables
				last_field = 0;
				memset(&next_target, 0, sizeof(GCODE_COMMAND));
				next_target.option = option_bitfield;

				serial_writeblock((uint8_t *) "OK\n", 3);
				break;
		}
	}

	// process digits
	else if (c == '-')
		read_digit.sign = 1;
	else if ((c == '.') && (read_digit.exponent == 0))
		read_digit.exponent = 1;
	else if (c >= '0' && c <= '9') {
		read_digit.mantissa = (read_digit.mantissa << 3) + (read_digit.mantissa << 1) + (c - '0');
		if (read_digit.exponent)
			read_digit.exponent++;
	}
}

void process_gcode_command(GCODE_COMMAND *gcmd) {
	uint8_t do_move;
	if (gcmd->seen & SEEN_G) {
		switch (gcmd->G) {
			// 	G0 - rapid, unsynchronised motion
			case 0:
				gcmd->option &= ~OPTION_SYNCHRONISE;
				do_move = 1;
				break;
			//	G30 - go home via point
			case 30:
				gcmd->option |= OPTION_HOME_WHEN_COMPLETE;
			//	G1 - synchronised motion
			case 1:
				gcmd->option |= OPTION_SYNCHRONISE;
				do_move = 1;
				break;
			//	G2 - Arc Clockwise
			//	G3 - Arc Counter-clockwise
			//	G4 - Dwell
			//	G20 - inches as units
			//	G21 - mm as units
			//	G28 - go home
			case 28:
				gcmd->option &= ~OPTION_SYNCHRONISE;
				do_move = 1;
				break;
			//	G90 - absolute positioning
			case 90:
				gcmd->option &= ~OPTION_RELATIVE;
				break;
			//	G91 - relative positioning
			case 91:
				gcmd->option |= OPTION_RELATIVE;
				break;
			//	G92 - set home
			case 92:
				break;
			// TODO: spit an error
		}
	}
	if (gcmd->seen & SEEN_M) {
		switch (gcmd->M) {
			// TODO: spit an error
		}
	}

	if (do_move) {
		dda_create(&gcmd->target, movebuffer);
	}
}
