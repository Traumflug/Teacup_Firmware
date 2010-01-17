#include	"gcode.h"

#include	<string.h>

#include	"machine.h"
#include	"dda.h"
#include	"serial.h"

extern uint8_t	option_bitfield;

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

float manexp_to_float(uint32_t mantissa, uint8_t exp) {
	float v = mantissa;

	if (exp == 2)
		v /= 10;
	else if (exp == 3)
		v /= 100;
	else if (exp == 4)
		v /= 1000;
	else if (exp == 5)
		v /= 10000;
	else if (exp == 6)
		v /= 100000;

	return v;
}

/*
	public functions
*/

void scan_char(uint8_t c) {
	static uint8_t last_field = 0;
	static uint32_t mantissa = 0;
	static uint8_t exp = 0;
	static GCODE_COMMAND next_target = { 0, 0, 0, 0, { 0, 0, 0, 0, 0 } };

	// uppercase
	if (c >= 'a' && c <= 'z')
		c &= ~32;

	// process field
	if (indexof(c, "GMXYZEF\n") != 255) {
		if (last_field) {
			switch (last_field) {
				case 'G':
					next_target.G = mantissa;
					break;
				case 'M':
					next_target.M = mantissa;
					break;
				case 'X':
					next_target.target.X = manexp_to_float(mantissa, exp) * STEPS_PER_MM_X;
					break;
				case 'Y':
					next_target.target.Y = manexp_to_float(mantissa, exp) * STEPS_PER_MM_Y;
					break;
				case 'Z':
					next_target.target.Z = manexp_to_float(mantissa, exp) * STEPS_PER_MM_Z;
					break;
				case 'E':
					next_target.target.E = manexp_to_float(mantissa, exp) * STEPS_PER_MM_E;
					break;
				case 'F':
					// just save an integer value for F, we need move distance and n_steps to convert it to a useful value, so wait until we have those to convert it
					next_target.target.F = mantissa;
					break;
			}
			mantissa = 0;
			exp = 0;
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
		exp |= 0x80;
	else if ((c == '.') && ((exp & 0x7F) == 0))
		exp |= 1;
	else if (c >= '0' && c <= '9') {
		mantissa = ((mantissa << 3) + (mantissa << 1)) + (c - '0');
		if (exp & 0x7F)
			exp++;
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
