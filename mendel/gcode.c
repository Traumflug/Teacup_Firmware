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

/*
	utility functions
*/

int8_t indexof(uint8_t c, char *string) {
	int8_t i;
	for (i = 0;string[i];i++) {
		if (c == string[i])
			return i;
	}
	return -1;
}

int32_t	decfloat_to_int(decfloat *df, int32_t multiplicand, int32_t denominator) {
	int32_t	r = df->mantissa;
	uint8_t	e = df->exponent - 1;

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
	static GCODE_COMMAND next_target = { 0, 0, 0, 0, { 0, 0, 0, 0, 0 } };

	// uppercase
	if (c >= 'a' && c <= 'z')
		c &= ~32;

	// process field
	if (indexof(c, "GMXYZEFSP\n") >= 0) {
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
				case 'S':
					// if this is temperature, multiply by 4 to convert to quarter-degree units
					// cosmetically this should be done in the temperature section,
					// but it takes less code, less memory and loses no precision if we do it here instead
					if (next_target.M == 104)
						next_target.S = decfloat_to_int(&read_digit, 4, 1);
					else
						next_target.S = decfloat_to_int(&read_digit, 1, 1);
					break;
				case 'P':
					// if this is dwell, multiply by 1 million to convert seconds to milliseconds
					if (next_target.G == 4)
						next_target.P = decfloat_to_int(&read_digit, 1000, 1);
					else
						next_target.P = decfloat_to_int(&read_digit, 1, 1);
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
			case 'S':
				next_target.seen |= SEEN_S;
				break;
			case 'P':
				next_target.seen |= SEEN_P;
				break;
			case '\n':
				// process
				process_gcode_command(&next_target);

				// save options
				option_bitfield = next_target.option;

				// reset variables
				last_field = 0;
				next_target.seen = 0;

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
	if (gcmd->option & OPTION_RELATIVE) {
		gcmd->target.X += startpoint.X;
		gcmd->target.Y += startpoint.Y;
		gcmd->target.Z += startpoint.Z;
		gcmd->target.E += startpoint.E;
	}

	if (gcmd->seen & SEEN_G) {
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
				serial_writeblock((uint8_t *) "E: Bad G-code ", 14);
				serwrite_uint8(gcmd->G);
				serial_writechar('\n');
		}
	}
	if (gcmd->seen & SEEN_M) {
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
					serial_writeblock((uint8_t *) "T: ", 3);
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
				serial_writeblock((uint8_t *) "E: Bad M-code ", 14);
				serwrite_uint8(gcmd->M);
				serial_writechar('\n');
		}
	}
}
