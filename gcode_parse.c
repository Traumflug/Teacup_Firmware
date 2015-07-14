#include	"gcode_parse.h"

/** \file
	\brief Parse received G-Codes
*/

#include	<string.h>

#include	"serial.h"
#include	"sermsg.h"
#include	"dda_queue.h"
#include	"debug.h"
#include	"heater.h"
#include	"sersendf.h"

#include	"gcode_process.h"
#ifdef SIMULATOR
  #include "simulator.h"
#endif


/** Bitfield for available sources of G-code.

  A typical source is the SD card or canned G-code. Serial is currently never
  turned off.
*/
enum gcode_source gcode_sources = GCODE_SOURCE_SERIAL;

/// current or previous gcode word
/// for working out what to do with data just received
uint8_t last_field[Parser_ListSize];

/// crude crc macro
#define crc(a, b)		(a ^ b)

/// crude floating point data storage
decfloat BSS read_digit[Parser_ListSize];

/// this is where we build the next command for each source (UART, SD Card, etc)
GCODE_COMMAND BSS command[Parser_ListSize];

#ifdef SD
  #define STR_BUF_LEN 13
  char gcode_str_buf[STR_BUF_LEN];
  static uint8_t str_buf_ptr = 0;
#endif

/*
	decfloat_to_int() is the weakest subject to variable overflow. For evaluation, we assume a build room of +-1000 mm and STEPS_PER_MM_x between 1.000 and 4096. Accordingly for metric units:

		df->mantissa:  +-0..1048075    (20 bit - 500 for rounding)
		df->exponent:  0, 2, 3, 4 or 5 (10 bit)
		multiplicand:  1000            (10 bit)

	imperial units:

		df->mantissa:  +-0..32267      (15 bit - 500 for rounding)
		df->exponent:  0, 2, 3, 4 or 5 (10 bit)
		multiplicand:  25400           (15 bit)
*/
// decfloat_to_int() can handle a bit more:
#define	DECFLOAT_EXP_MAX 3 // more is pointless, as 1 um is our presision
// (2^^32 - 1) / multiplicand - powers[DECFLOAT_EXP_MAX] / 2 =
// 4294967295 / 1000 - 5000 =
#define	DECFLOAT_MANT_MM_MAX 4289967  // = 4290 mm
// 4294967295 / 25400 - 5000 =
#define	DECFLOAT_MANT_IN_MAX 164093   // = 164 inches = 4160 mm

/*
	utility functions
*/
extern const uint32_t powers[];  // defined in sermsg.c

/// convert a floating point input value into an integer with appropriate scaling.
/// \param *df pointer to floating point structure that holds fp value to convert
/// \param multiplicand multiply by this amount during conversion to integer
///
/// Tested for up to 42'000 mm (accurate), 420'000 mm (precision 10 um) and
/// 4'200'000 mm (precision 100 um).
static int32_t decfloat_to_int(decfloat *df, uint16_t multiplicand) {
	uint32_t	r = df->mantissa;
	uint8_t	e = df->exponent;

	// e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero
	if (e)
		e--;

	// This raises range for mm by factor 1000 and for inches by factor 100.
	// It's a bit expensive, but we should have the time while parsing.
	while (e && multiplicand % 10 == 0) {
		multiplicand /= 10;
		e--;
	}

	r *= multiplicand;
	if (e)
		r = (r + powers[e] / 2) / powers[e];

	return df->sign ? -(int32_t)r : (int32_t)r;
}

void gcode_init(void) {
	// gcc guarantees us all variables are initialised to 0.

  int i;
  for (i=0;i<Parser_ListSize;i++) {
    command[i].target.F = SEARCH_FEEDRATE_Z;
	#ifndef E_ABSOLUTE
      command[i].option_e_relative = 1;
	#endif
  }
}

/** Character received - add it to our command.

  \param c The next character to process.

  \return Whether end of line was reached.

  This parser operates character by character, so there's no need for a
  buffer holding the entire line of G-code.
*/
uint8_t gcode_parse_char(uint8_t c, ParserType ctx) {
	uint8_t checksum_char = c;

  GCODE_COMMAND *cmd=&command[ctx];

	// uppercase
	if (c >= 'a' && c <= 'z')
		c &= ~32;
#ifdef SIMULATOR
  sim_gcode_ch(c,ctx);
#endif

  // An asterisk is a quasi-EOL and always ends all fields.
  if (c == '*') {
    cmd->seen_semi_comment = cmd->seen_parens_comment =
    cmd->read_string = 0;
  }

  // Skip comments and strings.
  if (cmd->seen_semi_comment == 0 &&
      cmd->seen_parens_comment == 0 &&
      cmd->read_string == 0
     ) {
    // Check if the field has ended. Either by a new field, space or EOL.
    if (last_field[ctx] && (c < '0' || c > '9') && c != '.') {
			switch (last_field[ctx]) {
				case 'G':
					cmd->G = read_digit[ctx].mantissa;
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint8(cmd->G);
					break;
				case 'M':
					cmd->M = read_digit[ctx].mantissa;
          #ifdef SD
            if (cmd->M == 23) {
              // SD card command with a filename.
              cmd->read_string = 1;  // Reset by string handler or EOL.
              str_buf_ptr = 0;
              last_field[ctx] = 0;
            }
          #endif
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint8(cmd->M);
					break;
				case 'X':
					if (cmd->option_inches)
            cmd->target.axis[X] = decfloat_to_int(&read_digit[ctx], 25400);
					else
            cmd->target.axis[X] = decfloat_to_int(&read_digit[ctx], 1000);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_int32(cmd->target.axis[X]);
					break;
				case 'Y':
					if (cmd->option_inches)
            cmd->target.axis[Y] = decfloat_to_int(&read_digit[ctx], 25400);
					else
            cmd->target.axis[Y] = decfloat_to_int(&read_digit[ctx], 1000);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_int32(cmd->target.axis[Y]);
					break;
				case 'Z':
					if (cmd->option_inches)
            cmd->target.axis[Z] = decfloat_to_int(&read_digit[ctx], 25400);
					else
            cmd->target.axis[Z] = decfloat_to_int(&read_digit[ctx], 1000);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_int32(cmd->target.axis[Z]);
					break;
				case 'E':
					if (cmd->option_inches)
            cmd->target.axis[E] = decfloat_to_int(&read_digit[ctx], 25400);
					else
            cmd->target.axis[E] = decfloat_to_int(&read_digit[ctx], 1000);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
            serwrite_int32(cmd->target.axis[E]);
					break;
				case 'F':
					// just use raw integer, we need move distance and n_steps to convert it to a useful value, so wait until we have those to convert it
					if (cmd->option_inches)
						cmd->target.F = decfloat_to_int(&read_digit[ctx], 25400);
					else
						cmd->target.F = decfloat_to_int(&read_digit[ctx], 1);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint32(cmd->target.F);
					break;
				case 'S':
					// if this is temperature, multiply by 4 to convert to quarter-degree units
					// cosmetically this should be done in the temperature section,
					// but it takes less code, less memory and loses no precision if we do it here instead
					if ((cmd->M == 104) || (cmd->M == 109) || (cmd->M == 140))
						cmd->S = decfloat_to_int(&read_digit[ctx], 4);
					// if this is heater PID stuff, multiply by PID_SCALE because we divide by PID_SCALE later on
					else if ((cmd->M >= 130) && (cmd->M <= 132))
						cmd->S = decfloat_to_int(&read_digit[ctx], PID_SCALE);
					else
						cmd->S = decfloat_to_int(&read_digit[ctx], 1);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_int32(cmd->S);
					break;
				case 'P':
					cmd->P = decfloat_to_int(&read_digit[ctx], 1);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint16(cmd->P);
					break;
				case 'T':
					cmd->T = read_digit[ctx].mantissa;
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint8(cmd->T);
					break;
				case 'N':
					cmd->N = decfloat_to_int(&read_digit[ctx], 1);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint32(cmd->N);
					break;
				case '*':
					cmd->checksum_read = decfloat_to_int(&read_digit[ctx], 1);
					if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
						serwrite_uint8(cmd->checksum_read);
					break;
			}
		}

		// new field?
		if ((c >= 'A' && c <= 'Z') || c == '*') {
			last_field[ctx] = c;
      read_digit[ctx].sign = read_digit[ctx].mantissa = read_digit[ctx].exponent = 0;
			if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
				serial_writechar(c);
		}

		// process character
    // Can't do ranges in switch..case, so process actual digits here.
    // Do it early, as there are many more digits than characters expected.
    if (c >= '0' && c <= '9') {
      if (read_digit[ctx].exponent < DECFLOAT_EXP_MAX + 1 &&
          ((cmd->option_inches == 0 &&
          read_digit[ctx].mantissa < DECFLOAT_MANT_MM_MAX) ||
          (cmd->option_inches &&
          read_digit[ctx].mantissa < DECFLOAT_MANT_IN_MAX))) {
        // this is simply mantissa = (mantissa * 10) + atoi(c) in different clothes
        read_digit[ctx].mantissa = (read_digit[ctx].mantissa << 3) +
                              (read_digit[ctx].mantissa << 1) + (c - '0');
        if (read_digit[ctx].exponent)
          read_digit[ctx].exponent++;
      }
    }
    else {
      switch (c) {
        // Each currently known command is either G or M, so preserve
        // previous G/M unless a new one has appeared.
        // FIXME: same for T command
        case 'G':
          cmd->seen_G = 1;
          cmd->seen_M = 0;
          cmd->M = 0;
          break;
        case 'M':
          cmd->seen_M = 1;
          cmd->seen_G = 0;
          cmd->G = 0;
          break;
        case 'X':
          cmd->seen_X = 1;
          break;
        case 'Y':
          cmd->seen_Y = 1;
          break;
        case 'Z':
          cmd->seen_Z = 1;
          break;
        case 'E':
          cmd->seen_E = 1;
          break;
        case 'F':
          cmd->seen_F = 1;
          break;
        case 'S':
          cmd->seen_S = 1;
          break;
        case 'P':
          cmd->seen_P = 1;
          break;
        case 'T':
          cmd->seen_T = 1;
          break;
        case 'N':
          cmd->seen_N = 1;
          break;
        case '*':
          cmd->seen_checksum = 1;
          break;

        // comments
        case ';':
          cmd->seen_semi_comment = 1;    // Reset by EOL.
          break;
        case '(':
          cmd->seen_parens_comment = 1;  // Reset by ')' or EOL.
          break;

        // now for some numeracy
        case '-':
          read_digit[ctx].sign = 1;
          // force sign to be at start of number, so 1-2 = -2 instead of -12
          read_digit[ctx].exponent = 0;
          read_digit[ctx].mantissa = 0;
          break;
        case '.':
          if (read_digit[ctx].exponent == 0)
            read_digit[ctx].exponent = 1;
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
          #ifdef	DEBUG
            // invalid
            serial_writechar('?');
            serial_writechar(c);
            serial_writechar('?');
          #endif
          break;
      }
		}
	} else if ( cmd->seen_parens_comment == 1 && c == ')')
		cmd->seen_parens_comment = 0; // recognize stuff after a (comment)

	if (cmd->seen_checksum == 0)
		cmd->checksum_calculated =
			crc(cmd->checksum_calculated, checksum_char);

	// end of line
	if ((c == 10) || (c == 13)) {
		if (DEBUG_ECHO && (debug_flags & DEBUG_ECHO))
			serial_writechar(c);

    // Assume G1 for unspecified movements.
    if ( ! cmd->seen_G && ! cmd->seen_M && ! cmd->seen_T &&
        (cmd->seen_X || cmd->seen_Y || cmd->seen_Z ||
         cmd->seen_E || cmd->seen_F)) {
      cmd->seen_G = 1;
      cmd->G = 1;
    }

		if (
		#ifdef	REQUIRE_LINENUMBER
			((cmd->N >= cmd->N_expected) && (cmd->seen_N == 1)) ||
			(cmd->seen_M && (cmd->M == 110))
		#else
			1
		#endif
			) {
			if (
				#ifdef	REQUIRE_CHECKSUM
				((cmd->checksum_calculated == cmd->checksum_read) && (cmd->seen_checksum == 1))
				#else
				((cmd->checksum_calculated == cmd->checksum_read) || (cmd->seen_checksum == 0))
				#endif
				) {
				// process
        if (ctx == Parser_Uart )
          serial_writestr_P(PSTR("ok "));
        process_gcode_command(cmd);
        if (ctx == Parser_Uart )
          serial_writechar('\n');
				// expect next line number
				if (cmd->seen_N == 1)
					cmd->N_expected = cmd->N + 1;
			}
			else {
        if (ctx == Parser_Uart ) {
          sersendf_P(PSTR("rs N%ld Expected checksum %d\n"), cmd->N_expected, cmd->checksum_calculated);
// 				request_resend();
        }
      }
		}
		else {
      if (ctx == Parser_Uart ) {
        sersendf_P(PSTR("rs N%ld Expected line number %ld\n"), cmd->N_expected, cmd->N_expected);
// 			request_resend();
      }
		}

		// reset variables
		cmd->seen_X = cmd->seen_Y = cmd->seen_Z = \
			cmd->seen_E = cmd->seen_F = cmd->seen_S = \
			cmd->seen_P = cmd->seen_T = cmd->seen_N = \
      cmd->seen_G = cmd->seen_M = cmd->seen_checksum = \
      cmd->seen_semi_comment = cmd->seen_parens_comment = \
      cmd->read_string = cmd->checksum_read = \
      cmd->checksum_calculated = 0;
      last_field[ctx] = 0;
      read_digit[ctx].sign = read_digit[ctx].mantissa = read_digit[ctx].exponent = 0;

		if (cmd->option_all_relative) {
      cmd->target.axis[X] = cmd->target.axis[Y] = cmd->target.axis[Z] = 0;
		}
		if (cmd->option_all_relative || cmd->option_e_relative) {
      cmd->target.axis[E] = 0;
		}

    return 1;
	}

  #ifdef SD
  // Handle string reading. After checking for EOL.
  if (cmd->read_string) {
    if (c == ' ') {
      if (str_buf_ptr)
        cmd->read_string = 0;
    }
    else if (str_buf_ptr < STR_BUF_LEN) {
      gcode_str_buf[str_buf_ptr] = c;
      str_buf_ptr++;
      gcode_str_buf[str_buf_ptr] = '\0';
    }
  }
  #endif /* SD */

  return 0;
}

/***************************************************************************\
*                                                                           *
* Request a resend of the current line - used from various places.          *
*                                                                           *
* FIXME: Assumes only the Parser_Uart is interested, but this is a mistake  *
*                                                                           *
\***************************************************************************/

void request_resend(void) {
	serial_writestr_P(PSTR("rs "));
	serwrite_uint8(command[Parser_Uart].N);
	serial_writechar('\n');
}
