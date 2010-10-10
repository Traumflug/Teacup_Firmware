#include	"gcode.h"

#include	<string.h>

#include	"config.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"temp.h"
#include	"timer.h"
#include	"dda_queue.h"
#include	"dda.h"
#include	"clock.h"
#include	"watchdog.h"
#include	"debug.h"
#include	"heater.h"
#include	"sersendf.h"
#include	"delay.h"

/*
	Switch user friendly values to coding friendly values

	This also affects the possible build volume. We have +-2^31 numbers available and as we internally measure position in steps and use a precision factor of 1000, this translates into a possible range of

		2^31 mm / STEPS_PER_MM_x / 1000

	for each axis. For a M6 threaded rod driven machine and 1/16 microstepping this evaluates to

		2^31 mm / 200 / 1 / 16 / 1000 = 671 mm,

	which is about the worst case we have. All other machines have a bigger build volume.
*/

#define	STEPS_PER_M_X			((uint32_t) (STEPS_PER_MM_X * 1000.0))
#define	STEPS_PER_M_Y			((uint32_t) (STEPS_PER_MM_Y * 1000.0))
#define	STEPS_PER_M_Z			((uint32_t) (STEPS_PER_MM_Z * 1000.0))
#define	STEPS_PER_M_E			((uint32_t) (STEPS_PER_MM_E * 1000.0))

/*
	mm -> inch conversion
*/

#define	STEPS_PER_IN_X		((uint32_t) ((25.4 * STEPS_PER_MM_X) + 0.5))
#define	STEPS_PER_IN_Y		((uint32_t) ((25.4 * STEPS_PER_MM_Y) + 0.5))
#define	STEPS_PER_IN_Z		((uint32_t) ((25.4 * STEPS_PER_MM_Z) + 0.5))
#define	STEPS_PER_IN_E		((uint32_t) ((25.4 * STEPS_PER_MM_E) + 0.5))

uint8_t last_field = 0;

#define crc(a, b)		(a ^ b)

decfloat read_digit					__attribute__ ((__section__ (".bss")));

GCODE_COMMAND next_target		__attribute__ ((__section__ (".bss")));

/*
	utility functions
*/

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

	// exponent- try to keep divides to a minimum for common (small) values at expense of slightly more code
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

/****************************************************************************
*                                                                           *
* Character Received - add it to our command                                *
*                                                                           *
****************************************************************************/

void scan_char(uint8_t c) {
	#ifdef ASTERISK_IN_CHECKSUM_INCLUDED
	if (next_target.seen_checksum == 0)
		next_target.checksum_calculated = crc(next_target.checksum_calculated, c);
	#endif

	// uppercase
	if (c >= 'a' && c <= 'z')
		c &= ~32;

	// process previous field
	if (last_field) {
		// check if we're seeing a new field or end of line
		// any character will start a new field, even invalid/unknown ones
		if ((c >= 'A' && c <= 'Z') || c == '*' || (c == 10) || (c == 13)) {
			switch (last_field) {
				case 'G':
					next_target.G = read_digit.mantissa;
					if (debug_flags & DEBUG_ECHO)
						serwrite_uint8(next_target.G);
					break;
				case 'M':
					next_target.M = read_digit.mantissa;
					if (debug_flags & DEBUG_ECHO)
						serwrite_uint8(next_target.M);
					break;
				case 'X':
					if (next_target.option_inches)
						next_target.target.X = decfloat_to_int(&read_digit, STEPS_PER_IN_X, 1);
					else
						next_target.target.X = decfloat_to_int(&read_digit, STEPS_PER_M_X, 1000);
					if (debug_flags & DEBUG_ECHO)
						serwrite_int32(next_target.target.X);
					break;
				case 'Y':
					if (next_target.option_inches)
						next_target.target.Y = decfloat_to_int(&read_digit, STEPS_PER_IN_Y, 1);
					else
						next_target.target.Y = decfloat_to_int(&read_digit, STEPS_PER_M_Y, 1000);
					if (debug_flags & DEBUG_ECHO)
						serwrite_int32(next_target.target.Y);
					break;
				case 'Z':
					if (next_target.option_inches)
						next_target.target.Z = decfloat_to_int(&read_digit, STEPS_PER_IN_Z, 1);
					else
						next_target.target.Z = decfloat_to_int(&read_digit, STEPS_PER_M_Z, 1000);
					if (debug_flags & DEBUG_ECHO)
						serwrite_int32(next_target.target.Z);
					break;
				case 'E':
					if (next_target.option_inches)
						next_target.target.E = decfloat_to_int(&read_digit, STEPS_PER_IN_E, 1);
					else
						next_target.target.E = decfloat_to_int(&read_digit, STEPS_PER_M_E, 1000);
					if (debug_flags & DEBUG_ECHO)
						serwrite_uint32(next_target.target.E);
					break;
				case 'F':
					// just use raw integer, we need move distance and n_steps to convert it to a useful value, so wait until we have those to convert it
					if (next_target.option_inches)
						next_target.target.F = decfloat_to_int(&read_digit, 254, 10);
					else
						next_target.target.F = decfloat_to_int(&read_digit, 1, 1);
					if (debug_flags & DEBUG_ECHO)
						serwrite_uint32(next_target.target.F);
					break;
				case 'S':
					// if this is temperature, multiply by 4 to convert to quarter-degree units
					// cosmetically this should be done in the temperature section,
					// but it takes less code, less memory and loses no precision if we do it here instead
					if ((next_target.M == 104) || (next_target.M == 109))
						next_target.S = decfloat_to_int(&read_digit, 4, 1);
					#ifdef	HEATER_PIN
					// if this is heater PID stuff, multiply by PID_SCALE because we divide by PID_SCALE later on
					else if ((next_target.M >= 130) && (next_target.M <= 132))
						next_target.S = decfloat_to_int(&read_digit, PID_SCALE, 1);
					#endif
					else
						next_target.S = decfloat_to_int(&read_digit, 1, 1);
					if (debug_flags & DEBUG_ECHO)
						serwrite_uint16(next_target.S);
					break;
				case 'P':
					// if this is dwell, multiply by 1000 to convert seconds to milliseconds
					if (next_target.G == 4)
						next_target.P = decfloat_to_int(&read_digit, 1000, 1);
					else
						next_target.P = decfloat_to_int(&read_digit, 1, 1);
					if (debug_flags & DEBUG_ECHO)
						serwrite_uint16(next_target.P);
					break;
				case 'N':
					next_target.N = decfloat_to_int(&read_digit, 1, 1);
					if (debug_flags & DEBUG_ECHO)
						serwrite_uint32(next_target.N);
					break;
				case '*':
					next_target.checksum_read = decfloat_to_int(&read_digit, 1, 1);
					if (debug_flags & DEBUG_ECHO)
						serwrite_uint8(next_target.checksum_read);
					break;
			}
			// reset for next field
			last_field = 0;
			read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;
		}
	}

	// skip comments
	if (next_target.seen_semi_comment == 0 && next_target.seen_parens_comment == 0) {
		// new field?
		if ((c >= 'A' && c <= 'Z') || c == '*') {
			last_field = c;
			if (debug_flags & DEBUG_ECHO)
				serial_writechar(c);
		}

		// process character
		switch (c) {
			// each currently known command is either G or M, so preserve previous G/M unless a new one has appeared
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
			case 'N':
				next_target.seen_N = 1;
				break;
			case '*':
				next_target.seen_checksum = 1;
				break;

			// comments
			case ';':
				next_target.seen_semi_comment = 1;
				break;
			case '(':
				next_target.seen_parens_comment = 1;
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
			#ifdef	DEBUG
			case ' ':
			case '\t':
			case 10:
			case 13:
				// ignore
				break;
			#endif

			default:
				// can't do ranges in switch..case, so process actual digits here
				if (c >= '0' && c <= '9') {
					// this is simply mantissa = (mantissa * 10) + atoi(c) in different clothes
					read_digit.mantissa = (read_digit.mantissa << 3) + (read_digit.mantissa << 1) + (c - '0');
					if (read_digit.exponent)
						read_digit.exponent++;
				}
				#ifdef	DEBUG
				else {
					// invalid
					serial_writechar('?');
					serial_writechar(c);
					serial_writechar('?');
				}
				#endif
		}
	} else if ( next_target.seen_parens_comment == 1 && c == ')')
		next_target.seen_parens_comment = 0; // recognize stuff after a (comment)


	#ifndef ASTERISK_IN_CHECKSUM_INCLUDED
	if (next_target.seen_checksum == 0)
		next_target.checksum_calculated = crc(next_target.checksum_calculated, c);
	#endif

	// end of line
	if ((c == 10) || (c == 13)) {
		if (debug_flags & DEBUG_ECHO)
			serial_writechar(c);

		if (
			#ifdef	REQUIRE_LINENUMBER
			((next_target.N >= next_target.N_expected) && (next_target.seen_N == 1))
			#else
			1
			#endif
			) {
			if (
				#ifdef	REQUIRE_CHECKSUM
				((next_target.checksum_calculated == next_target.checksum_read) && (next_target.seen_checksum == 1))
				#else
				((next_target.checksum_calculated == next_target.checksum_read) || (next_target.seen_checksum == 0))
				#endif
				) {
				// process
				process_gcode_command(&next_target);
				serial_writestr_P(PSTR("ok\n"));

				// expect next line number
				if (next_target.seen_N == 1)
					next_target.N_expected = next_target.N + 1;
			}
			else {
				serial_writestr_P(PSTR("Expected checksum "));
				serwrite_uint8(next_target.checksum_calculated);
				serial_writechar('\n');
				request_resend();
			}
		}
		else {
			serial_writestr_P(PSTR("Expected line number "));
			serwrite_uint32(next_target.N_expected);
			serial_writechar('\n');
			request_resend();
		}

		// reset variables
		next_target.seen_X = next_target.seen_Y = next_target.seen_Z = \
			next_target.seen_E = next_target.seen_F = next_target.seen_S = \
			next_target.seen_P = next_target.seen_N = next_target.seen_M = \
			next_target.seen_checksum = next_target.seen_semi_comment = \
			next_target.seen_parens_comment = next_target.checksum_read = \
			next_target.checksum_calculated = 0;
		last_field = 0;
		read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;
		// assume a G1 by default
		next_target.seen_G = 1;
		next_target.G = 1;
		if (next_target.option_relative) {
			next_target.target.X = next_target.target.Y = next_target.target.Z = 0;
			next_target.target.E = 0;
		}
	}
}

/****************************************************************************
*                                                                           *
* Command Received - process it                                             *
*                                                                           *
****************************************************************************/

void process_gcode_command(GCODE_COMMAND *gcmd) {
	uint32_t	backup_f;

	// convert relative to absolute
	if (gcmd->option_relative) {
		gcmd->target.X += startpoint.X;
		gcmd->target.Y += startpoint.Y;
		gcmd->target.Z += startpoint.Z;
	}
	// E ALWAYS relative, otherwise we overflow our registers after only a few layers
// 	gcmd->target.E += startpoint.E;
	// easier way to do this
// 	startpoint.E = 0;
	// moved to dda.c, end of dda_create() and dda_queue.c, next_move()

	if (gcmd->seen_G) {
		switch (gcmd->G) {
			// 	G0 - rapid, unsynchronised motion
			// since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
			case 0:
				backup_f = gcmd->target.F;
				gcmd->target.F = MAXIMUM_FEEDRATE_X * 2;
				enqueue(&gcmd->target);
				gcmd->target.F = backup_f;
				break;

			//	G1 - synchronised motion
			case 1:
				enqueue(&gcmd->target);
				break;

			//	G2 - Arc Clockwise
			// unimplemented

			//	G3 - Arc Counter-clockwise
			// unimplemented

			//	G4 - Dwell
			case 4:
				// wait for all moves to complete
				for (;queue_empty() == 0;)
					wd_reset();
				// delay
				delay_ms(gcmd->P);
				break;

			//	G20 - inches as units
			case 20:
				gcmd->option_inches = 1;
				break;

			//	G21 - mm as units
			case 21:
				gcmd->option_inches = 0;
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
				// hit endstops, no acceleration- we don't care about skipped steps
				startpoint.F = MAXIMUM_FEEDRATE_X;
				SpecialMoveXY(-250L * STEPS_PER_MM_X, -250L * STEPS_PER_MM_Y, MAXIMUM_FEEDRATE_X);
				startpoint.X = startpoint.Y = 0;

				// move forward a bit
				SpecialMoveXY(5 * STEPS_PER_MM_X, 5 * STEPS_PER_MM_Y, SEARCH_FEEDRATE_X);

				// move back in to endstops slowly
				SpecialMoveXY(-20 * STEPS_PER_MM_X, -20 * STEPS_PER_MM_Y, SEARCH_FEEDRATE_X);

				// wait for queue to complete
				for (;queue_empty() == 0;)
					wd_reset();

				// this is our home point
				startpoint.X = startpoint.Y = current_position.X = current_position.Y = 0;

				/*
					Home Z
				*/
				// hit endstop, no acceleration- we don't care about skipped steps
				startpoint.F = MAXIMUM_FEEDRATE_Z;
				SpecialMoveZ(-250L * STEPS_PER_MM_Z, MAXIMUM_FEEDRATE_Z);
				startpoint.Z = 0;

				// move forward a bit
				SpecialMoveZ(5 * STEPS_PER_MM_Z, SEARCH_FEEDRATE_Z);

				// move back into endstop slowly
				SpecialMoveZ(-20L * STEPS_PER_MM_Z, SEARCH_FEEDRATE_Z);

				// wait for queue to complete
				for (;queue_empty() == 0;)
					wd_reset();

				// this is our home point
				startpoint.Z = current_position.Z = 0;

				/*
					Home E
				*/
				// extruder only runs one way and we have no "endstop", just set this point as home
				startpoint.E = current_position.E = 0;

				/*
					Home F
				*/

				// F has been left at SEARCH_FEEDRATE_Z by the last move, this is a usable "home"
				// uncomment the following or substitute if you prefer a different default feedrate
				// startpoint.F = SEARCH_FEEDRATE_Z

				break;

			//	G90 - absolute positioning
			case 90:
				gcmd->option_relative = 0;
				break;

			//	G91 - relative positioning
			case 91:
				gcmd->option_relative = 1;
				break;

			//	G92 - set home
			case 92:
				startpoint.X = startpoint.Y = startpoint.Z = startpoint.E =
					current_position.X = current_position.Y = current_position.Z = current_position.E = 0;
				startpoint.F =
					current_position.F = SEARCH_FEEDRATE_Z;
				break;

			// unknown gcode: spit an error
			default:
				serial_writestr_P(PSTR("E: Bad G-code "));
				serwrite_uint8(gcmd->G);
				serial_writechar('\n');
		}
	}
	else if (gcmd->seen_M) {
		switch (gcmd->M) {
			// M101- extruder on
			case 101:
				if (temp_achieved() == 0) {
					enqueue(NULL);
				}
				do {
					// backup feedrate, move E very quickly then restore feedrate
					backup_f = startpoint.F;
					startpoint.F = MAXIMUM_FEEDRATE_E;
					SpecialMoveE(E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
					startpoint.F = backup_f;
				} while (0);
				break;

			// M102- extruder reverse

			// M103- extruder off
			case 103:
				do {
					// backup feedrate, move E very quickly then restore feedrate
					backup_f = startpoint.F;
					startpoint.F = MAXIMUM_FEEDRATE_E;
					SpecialMoveE(-E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
					startpoint.F = backup_f;
				} while (0);
				break;

			// M104- set temperature
			case 104:
				temp_set(gcmd->S);
				if (gcmd->S) {
					enable_heater();
					power_on();
				}
				else {
					disable_heater();
				}
				break;

			// M105- get temperature
			case 105:
				temp_print();
				break;

			// M106- fan on
			#ifdef	FAN_PIN
			case 106:
				enable_fan();
				break;
			// M107- fan off
			case 107:
				disable_fan();
				break;
			#endif

			// M109- set temp and wait
			case 109:
				temp_set(gcmd->S);
				if (gcmd->S) {
					enable_heater();
					power_on();
				}
				else {
					disable_heater();
				}
				enqueue(NULL);
				break;

			// M110- set line number
			case 110:
				gcmd->N_expected = gcmd->S - 1;
				break;
			// M111- set debug level
			#ifdef	DEBUG
			case 111:
				debug_flags = gcmd->S;
				break;
			#endif
			// M112- immediate stop
			case 112:
				disableTimerInterrupt();
				power_off();
				break;
			// M113- extruder PWM
			// M114- report XYZEF to host
			case 114:
				sersendf_P("X:%ld,Y:%ld,Z:%ld,E:%ld,F:%ld\n", current_position.X, current_position.Y, current_position.Z, current_position.E, current_position.F);
			 	break;

			#ifdef	HEATER_PIN
			// M130- heater P factor
			case 130:
				if (gcmd->seen_S)
					p_factor = gcmd->S;
				break;
			// M131- heater I factor
			case 131:
				if (gcmd->seen_S)
					i_factor = gcmd->S;
				break;
			// M132- heater D factor
			case 132:
				if (gcmd->seen_S)
					d_factor = gcmd->S;
				break;
			// M133- heater I limit
			case 133:
				if (gcmd->seen_S)
					i_limit = gcmd->S;
				break;
			// M134- save PID settings to eeprom
			case 134:
				heater_save_settings();
				break;
			#endif	/* HEATER_PIN */

			// M190- power on
			case 190:
				power_on();
				#ifdef	X_ENABLE_PIN
					WRITE(X_ENABLE_PIN, 0);
				#endif
				#ifdef	Y_ENABLE_PIN
					WRITE(Y_ENABLE_PIN, 0);
				#endif
				#ifdef	Z_ENABLE_PIN
					WRITE(Z_ENABLE_PIN, 0);
				#endif
				steptimeout = 0;
				break;
			// M191- power off
			case 191:
				#ifdef	X_ENABLE_PIN
					WRITE(X_ENABLE_PIN, 1);
				#endif
				#ifdef	Y_ENABLE_PIN
					WRITE(Y_ENABLE_PIN, 1);
				#endif
				#ifdef	Z_ENABLE_PIN
					WRITE(Z_ENABLE_PIN, 1);
				#endif
				power_off();
				break;

			#ifdef	DEBUG
			// M140- echo off
			case 140:
				debug_flags &= ~DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo off\n"));
				break;
			// M141- echo on
			case 141:
				debug_flags |= DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo on\n"));
				break;

			// DEBUG: return current position
			case 250:
				serial_writestr_P(PSTR("{X:"));
				serwrite_int32(current_position.X);
				serial_writestr_P(PSTR(",Y:"));
				serwrite_int32(current_position.Y);
				serial_writestr_P(PSTR(",Z:"));
				serwrite_int32(current_position.Z);
				serial_writestr_P(PSTR(",E:"));
				serwrite_int32(current_position.E);
				serial_writestr_P(PSTR(",F:"));
				serwrite_int32(current_position.F);
				serial_writestr_P(PSTR(",c:"));
				serwrite_uint32(movebuffer[mb_tail].c);
				serial_writestr_P(PSTR("}\n"));

				serial_writestr_P(PSTR("{X:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.X);
				serial_writestr_P(PSTR(",Y:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.Y);
				serial_writestr_P(PSTR(",Z:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.Z);
				serial_writestr_P(PSTR(",E:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.E);
				serial_writestr_P(PSTR(",F:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.F);
				serial_writestr_P(PSTR(",c:"));
				#ifdef ACCELERATION_REPRAP
				serwrite_uint32(movebuffer[mb_tail].end_c);
				#else
				serwrite_uint32(movebuffer[mb_tail].c);
				#endif
				serial_writestr_P(PSTR("}\n"));

				print_queue();
				break;

			// DEBUG: read arbitrary memory location
			case 253:
				if (gcmd->seen_P == 0)
					gcmd->P = 1;
				for (; gcmd->P; gcmd->P--) {
					serwrite_hex8(*(volatile uint8_t *)(gcmd->S));
					gcmd->S++;
				}
				serial_writechar('\n');
				break;

			// DEBUG: write arbitrary memory locatiom
			case 254:
				serwrite_hex8(gcmd->S);
				serial_writechar(':');
				serwrite_hex8(*(volatile uint8_t *)(gcmd->S));
				serial_writestr_P(PSTR("->"));
				serwrite_hex8(gcmd->P);
				serial_writechar('\n');
				(*(volatile uint8_t *)(gcmd->S)) = gcmd->P;
				break;
			#endif /* DEBUG */
			// unknown mcode: spit an error
			default:
				serial_writestr_P(PSTR("E: Bad M-code "));
				serwrite_uint8(gcmd->M);
				serial_writechar('\n');
		}
	}
}

/****************************************************************************
*                                                                           *
* Request a resend of the current line - used from various places.          *
*                                                                           *
* Relies on the global variable next_target.N being valid.                  *
*                                                                           *
****************************************************************************/

void request_resend(void) {
	serial_writestr_P(PSTR("rs "));
	serwrite_uint8(next_target.N);
	serial_writechar('\n');
}

