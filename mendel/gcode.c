#include	"gcode.h"

#include	<string.h>

#include	"machine.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"temp.h"
#include	"timer.h"

uint8_t	option_bitfield;

decfloat read_digit;

#define	PI	3.1415926535

const char alphabet[] = "GMXYZEFSP";

/*
	utility functions
*/

int8_t indexof(uint8_t c, const char *string) {
	int8_t i;
	for (i = 0; string[i]; i++) {
		if (c == string[i])
			return i;
	}
	return -1;
}

int32_t	decfloat_to_int(decfloat *df, int32_t multiplicand, int32_t denominator) {
	int32_t	r = df->mantissa;
	uint8_t	e = df->exponent;

	// e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero
	if (e)
		e--;

	// scale factors
	if (multiplicand != 1)
		r *= multiplicand;
	if (denominator != 1)
		r /= denominator;

	// sign
	if (df->sign)
		r = -r;

	// exponent- try to keep divides to a minimum at expense of slightly more code
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

void SpecialMoveXY(int32_t x, int32_t y, uint32_t f) {
	TARGET t = startpoint;
	t.X = x;
	t.Y = y;
	t.F = f;
	enqueue(&t);
}

void SpecialMoveZ(int32_t z, uint32_t f) {
	TARGET t = startpoint;
	t.Z = z;
	t.F = f;
	enqueue(&t);
}

void SpecialMoveE(int32_t e, uint32_t f) {
	TARGET t = startpoint;
	t.E = e;
	t.F = f;
	enqueue(&t);
}

/*
	public functions
*/

void scan_char(uint8_t c) {
	static uint8_t last_field = 0;
	static GCODE_COMMAND next_target = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, { 0, 0, 0, 0, 0 } };

	// uppercase
	if (c >= 'a' && c <= 'z')
		c &= ~32;

	// process field
	if (last_field) {
		if ((indexof(c, alphabet) >= 0) || (c == 10)) {
			switch (last_field) {
				case 'G':
					next_target.G = read_digit.mantissa;
					serwrite_uint8(next_target.G);
					break;
				case 'M':
					next_target.M = read_digit.mantissa;
					serwrite_uint8(next_target.M);
					break;
				case 'X':
					next_target.target.X = decfloat_to_int(&read_digit, STEPS_PER_MM_X, 1);
					serwrite_int32(next_target.target.X);
					break;
				case 'Y':
					next_target.target.Y = decfloat_to_int(&read_digit, STEPS_PER_MM_Y, 1);
					serwrite_int32(next_target.target.Y);
					break;
				case 'Z':
					next_target.target.Z = decfloat_to_int(&read_digit, STEPS_PER_MM_Z, 1);
					serwrite_int32(next_target.target.Z);
					break;
				case 'E':
					next_target.target.E = decfloat_to_int(&read_digit, STEPS_PER_MM_E, 1);
					serwrite_uint32(next_target.target.E);
					break;
				case 'F':
					// just save an integer value for F, we need move distance and n_steps to convert it to a useful value, so wait until we have those to convert it
					next_target.target.F = read_digit.mantissa;
					serwrite_uint32(next_target.target.F);
					break;
				case 'S':
					// if this is temperature, multiply by 4 to convert to quarter-degree units
					// cosmetically this should be done in the temperature section,
					// but it takes less code, less memory and loses no precision if we do it here instead
					if (next_target.M == 104)
						next_target.S = decfloat_to_int(&read_digit, 4, 1);
					else
						next_target.S = decfloat_to_int(&read_digit, 1, 1);
					serwrite_uint16(next_target.S);
					break;
				case 'P':
					// if this is dwell, multiply by 1 million to convert seconds to milliseconds
					if (next_target.G == 4)
						next_target.P = decfloat_to_int(&read_digit, 1000, 1);
					else
						next_target.P = decfloat_to_int(&read_digit, 1, 1);
					serwrite_uint16(next_target.P);
					break;
			}
			last_field = 0;
			read_digit.sign = 0;
			read_digit.mantissa = 0;
			read_digit.exponent = 0;
		}
	}

	// not in a comment?
	if ((option_bitfield & OPTION_COMMENT) == 0) {
		if (indexof(c, alphabet) >= 0) {
			last_field = c;
			serial_writechar(c);
		}

		// process character
		switch (c) {
			// each command is either G or M, so preserve previous G/M unless a new one has appeared
			case 'G':
				next_target.seen_G = 1;
				next_target.seen_M = 0;
				next_target.M = 0;
				break;
			case 'M':
				next_target.seen_M = 1;
				next_target.seen_G = 0;
				next_target.G = 0;
				break;
			case 'X':
				next_target.seen_X = 1;
				break;
			case 'Y':
				next_target.seen_Y = 1;
				break;
			case 'Z':
				next_target.seen_Z = 1;
				break;
			case 'E':
				next_target.seen_E = 1;
				break;
			case 'F':
				next_target.seen_F = 1;
				break;
			case 'S':
				next_target.seen_S = 1;
				break;
			case 'P':
				next_target.seen_P = 1;
				break;

			// comments
			case ';':
				option_bitfield |= OPTION_COMMENT;
				break;

			// now for some numeracy
			case '-':
				read_digit.sign = 1;
				// force sign to be at start of number
				read_digit.exponent = 0;
				read_digit.mantissa = 0;
				break;
			case '.':
				if (read_digit.exponent == 0)
					read_digit.exponent = 1;
				break;

			default:
				// can't do ranges in switch..case, so process actual digits here
				if (c >= '0' && c <= '9') {
					read_digit.mantissa = (read_digit.mantissa << 3) + (read_digit.mantissa << 1) + (c - '0');
					if (read_digit.exponent)
						read_digit.exponent++;
				}
		}
	}

	// got a command
	if (c == 10) {
		serial_writechar(c);
		// process
		process_gcode_command(&next_target);

		// save options
		option_bitfield = next_target.option;
// 		option_bitfield &= ~OPTION_COMMENT;

		// reset variables
		next_target.seen_X = next_target.seen_Y = next_target.seen_Z = next_target.seen_E = next_target.seen_F = next_target.seen_S = next_target.seen_P = 0;
		last_field = 0;
		read_digit.sign = 0;
		read_digit.mantissa = 0;
		read_digit.exponent = 0;

		serial_writestr_P(PSTR("OK\n"));
	}
}

void process_gcode_command(GCODE_COMMAND *gcmd) {
	if (gcmd->option & OPTION_RELATIVE) {
		gcmd->target.X += startpoint.X;
		gcmd->target.Y += startpoint.Y;
		gcmd->target.Z += startpoint.Z;
		gcmd->target.E += startpoint.E;
	}

	if (gcmd->seen_G) {
		switch (gcmd->G) {
			// 	G0 - rapid, unsynchronised motion
			// since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
			case 0:
				gcmd->target.F = FEEDRATE_FAST_XY;
				enqueue(&gcmd->target);
				break;

			//	G1 - synchronised motion
			case 1:
				enqueue(&gcmd->target);
				break;

			//	G2 - Arc Clockwise

			//	G3 - Arc Counter-clockwise

			//	G4 - Dwell
			case 4:
				delay_ms(gcmd->P);
				break;

			//	G20 - inches as units
			case 20:
				gcmd->option |= OPTION_UNIT_INCHES;
				break;

			//	G21 - mm as units
			case 21:
				gcmd->option &= ~OPTION_UNIT_INCHES;
				break;

			//	G30 - go home via point
			case 30:
				enqueue(&gcmd->target);
				// no break here, G30 is move and then go home
			//	G28 - go home
			case 28:
				/*
					Home XY first
				*/
				// hit endstops
				SpecialMoveXY(-250L * STEPS_PER_MM_X, -250L * STEPS_PER_MM_Y, FEEDRATE_FAST_XY);
				startpoint.X = startpoint.Y = 0;

				SpecialMoveXY(5 * STEPS_PER_MM_X, 5 * STEPS_PER_MM_Y, FEEDRATE_SLOW_XY);

				// move back in to endstops slowly
				SpecialMoveXY(-20 * STEPS_PER_MM_X, -20 * STEPS_PER_MM_Y, FEEDRATE_SLOW_XY);

				// wait for queue to complete
				for (;!queue_empty(););

				// this is our home point
				startpoint.X = startpoint.Y = current_position.X = current_position.Y = 0;

				/*
					Home Z
				*/
				// hit endstop
				SpecialMoveZ(-250L * STEPS_PER_MM_Z, FEEDRATE_FAST_Z);
				startpoint.Z = 0;

				// move out a bit
				SpecialMoveZ(5 * STEPS_PER_MM_Z, FEEDRATE_SLOW_Z);

				// move back into endstop slowly
				SpecialMoveZ(-20L * STEPS_PER_MM_Z, FEEDRATE_SLOW_Z);

				// wait for queue to complete
				for (;queue_empty(););

				// this is our home point
				startpoint.Z = current_position.Z = 0;

				/*
					Home E
				*/
				// extruder only runs one way anyway
				startpoint.E = current_position.E = 0;

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
				startpoint.X = startpoint.Y = startpoint.Z = 0;
				break;

			// TODO: spit an error
			default:
				serial_writestr_P(PSTR("E: Bad G-code "));
				serwrite_uint8(gcmd->G);
				serial_writechar('\n');
		}
	}
	if (gcmd->seen_M) {
		switch (gcmd->M) {
			// M101- extruder on
			case 101:
				SpecialMoveE(E_STARTSTOP_STEPS, FEEDRATE_FAST_E);
				break;

			// M102- extruder reverse

			// M103- extruder off
			case 103:
				SpecialMoveE(-E_STARTSTOP_STEPS, FEEDRATE_FAST_E);
				break;

			// M104- set temperature
			case 104:
				temp_set(gcmd->S);
				break;

			// M105- get temperature
			case 105:
				// do .. while block here to provide local scope for temp
				do {
					uint16_t t = temp_get();
					serial_writestr_P(PSTR("T: "));
					serwrite_uint16(t >> 2);
					serial_writechar('.');
					if (t & 3) {
						if ((t & 3) == 1)
							serial_writechar('2');
						else if ((t & 3) == 3)
							serial_writechar('7');
						serial_writechar('5');
					}
					else {
						serial_writechar('0');
					}
					serial_writechar('\n');
				} while (0);
				break;

			// M106- fan on
			#ifdef	FAN_PIN
			case 106:
				WRITE(FAN_PIN, 1);
				break;

			// M107- fan off
			case 107:
				WRITE(FAN_PIN, 0);
				break;
			#endif

			// TODO: spit an error
			default:
				serial_writestr_P(PSTR("E: Bad M-code "));
				serwrite_uint8(gcmd->M);
				serial_writechar('\n');
		}
	}
}
