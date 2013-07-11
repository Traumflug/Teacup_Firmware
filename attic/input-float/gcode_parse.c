#include	"gcode_parse.h"

/** \file
	\brief Parse received G-Codes
*/

#include	<stdlib.h>

#include	"config.h"
#include	"sersendf.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"gcode_process.h"

/// current or previous gcode word
/// for working out what to do with data just received
uint8_t last_field = 0;

/// crude crc macro
#define crc(a, b)		(a ^ b)

#define	GCODE_LINE_BUFFER_LEN	64

#define iA 0
#define iB 1
#define iC 2
#define iD 3
#define iE 4
#define iF 5
#define iG 6
#define iH 7
#define iI 8
#define iJ 9
#define iK 10
#define iL 11
#define iM 12
#define iN 13
#define iO 14
#define iP 15
#define iQ 16
#define iR 17
#define iS 18
#define iT 19
#define iU 20
#define iV 21
#define iW 22
#define iX 23
#define iY 24
#define iZ 25
#define	iAsterisk	26

GCODE_COMMAND next_target;

uint8_t		gcode_line[GCODE_LINE_BUFFER_LEN];
uint8_t		gcode_line_pointer = 0;

float			words[32];
uint32_t	seen_mask = 0;

#define	SEEN(c)	(seen_mask & (1L << (c)))

uint32_t	line_number = 0;

const uint8_t char2index(uint8_t c) __attribute__ ((pure));
const uint8_t char2index(uint8_t c) {
	if (c >= 'a' && c <= 'z')
		return c - 'a';
	if (c >= 'A' && c <= 'Z')
		return c - 'A';
	if (c == '*')
		return 26;
	return 255;
}

void gcode_parse_char(uint8_t c) {
	if (gcode_line_pointer < (GCODE_LINE_BUFFER_LEN - 1))
		gcode_line[gcode_line_pointer++] = c;
	if ((c == 13) || (c == 10)) {
		uint8_t i;
		for (i = gcode_line_pointer; i < GCODE_LINE_BUFFER_LEN; i++)
			gcode_line[i] = 0;
		if (gcode_line_pointer > 2)
			gcode_parse_line(gcode_line);
		gcode_line_pointer = 0;
	}
}

void gcode_parse_line(uint8_t *c) {
	enum {
		STATE_FIND_WORD,
		STATE_FIND_VALUE,
		STATE_SEMICOLON_COMMENT,
		STATE_BRACKET_COMMENT,
	} state = STATE_FIND_WORD;
	
	uint8_t i;	// string index
	uint8_t w = 0;	// current gcode word
	uint8_t checksum = 0;
	
	seen_mask = 0;
	
	// calculate checksum
	for(i = 0; c[i] != '*' && c[i] != 0; i++)
		checksum = checksum ^ c[i];
	
	// extract gcode words from line
	for (i = 0; c[i] != 0 && c[i] != 13 && c[i] != 10; i++) {
		switch (state) {
			case STATE_FIND_WORD:
				// start of word
				if (char2index(c[i]) < 255) {
					w = char2index(c[i]);
					state = STATE_FIND_VALUE;
				}
				// comment until end of line
				if (c[i] == ';')
					state = STATE_SEMICOLON_COMMENT;
				// comment until close bracket
				if (c[i] == '(')
					state = STATE_BRACKET_COMMENT;
				break;
			case STATE_FIND_VALUE:
				if ((c[i] >= '0' && c[i] <= '9') || c[i] == '-') {
					uint8_t	*ep;
					float v = strtod((const char *) &c[i], (char **) &ep);
					state = STATE_FIND_WORD;
					if (ep > &c[i]) {
// 						sersendf_P(PSTR("[seen %c: %lx->"), w + 'A', seen_mask);
						seen_mask |= (1L << w);
// 						sersendf_P(PSTR("%lx]"), seen_mask);
						words[w] = v;
						i = ep - c - 1;
					}
				}
				break;
			case STATE_BRACKET_COMMENT:
				if (c[i] == ')')
					state = STATE_FIND_WORD;
				break;
			case STATE_SEMICOLON_COMMENT:
				// dummy entry to suppress compiler warning
				break;
		} // switch (state)
	} // for i=0 .. newline
	
	// TODO: process line just read
	
	if (SEEN(iAsterisk)) {
		if (checksum != words[iAsterisk]) {
			if (seen_mask & iAsterisk)
				sersendf_P(PSTR("rs %d "), ((uint8_t) words[iAsterisk]));
			sersendf_P(PSTR("Bad checksum, received %d, expected %d\n"), ((uint8_t) words[iAsterisk]), checksum);
			seen_mask = 0;
			return;
		}
	}
	
	if (SEEN(iN)) {
		if (((uint32_t) words[iN]) != line_number) {
			sersendf_P(PSTR("Bad line number, received %ld, expected %ld\n"), ((uint32_t) words[iN]), line_number);
			seen_mask = 0;
			return;
		}
		line_number++;
	}
	
	serial_writestr_P(PSTR("ok "));

	// patch words into next_target struct
	// TODO: eliminate next_target, update gcode_process to use words[] directly

	next_target.flags = 0;
	if (SEEN(iG)) {
		next_target.seen_G = 1;
		next_target.G = words[iG];
// 		sersendf_P(PSTR("G:%d/"), next_target.G);
	}
	if (SEEN(iM)) {
		next_target.seen_M = 1;
		next_target.M = words[iM];
// 		sersendf_P(PSTR("M:%d/"), next_target.M);
	}
	if (SEEN(iX)) {
		next_target.seen_X = 1;
		next_target.target.X = words[iX] * STEPS_PER_MM_X;
// 		sersendf_P(PSTR("X:%ld/"), next_target.target.X);
	}
	if (SEEN(iY)) {
		next_target.seen_Y = 1;
		next_target.target.Y = words[iY] * STEPS_PER_MM_Y;
// 		sersendf_P(PSTR("Y:%ld/"), next_target.target.Y);
	}
	if (SEEN(iZ)) {
		next_target.seen_Z = 1;
		next_target.target.Z = words[iZ] * STEPS_PER_MM_Z;
// 		sersendf_P(PSTR("Z:%ld/"), next_target.target.Z);
	}
	if (SEEN(iE)) {
		next_target.seen_E = 1;
		next_target.target.E = words[iE] * STEPS_PER_MM_E;
// 		sersendf_P(PSTR("E:%ld/"), next_target.target.E);
	}
	if (SEEN(iF)) {
		next_target.seen_F = 1;
		next_target.target.F = words[iF];
// 		sersendf_P(PSTR("F:%ld/"), next_target.target.F);
	}
	if (SEEN(iS)) {
		next_target.seen_S = 1;
		// if this is temperature, multiply by 4 to convert to quarter-degree units
		// cosmetically this should be done in the temperature section,
		// but it takes less code, less memory and loses no precision if we do it here instead
		if ((next_target.M == 104) || (next_target.M == 109))
			next_target.S = words[iS] * 4.0;
		// if this is heater PID stuff, multiply by PID_SCALE because we divide by PID_SCALE later on
		else if ((next_target.M >= 130) && (next_target.M <= 132))
			next_target.S = words[iS] * PID_SCALE;
		else
			next_target.S = words[iS];
// 		sersendf_P(PSTR("S:%d/"), next_target.S);
	}
	if (SEEN(iP)) {
		next_target.seen_P = 1;
		next_target.P = words[iP];
// 		sersendf_P(PSTR("P:%u/"), next_target.P);
	}
	if (SEEN(iT)) {
		next_target.seen_T = 1;
		next_target.T = words[iT];
// 		sersendf_P(PSTR("T:%d/"), next_target.T);
	}
	if (SEEN(iN)) {
		next_target.seen_N = 1;
		next_target.N = words[iN];
// 		sersendf_P(PSTR("N:%lu/"), next_target.N);
	}
	next_target.N_expected = line_number;
	if (SEEN(iAsterisk)) {
		next_target.seen_checksum = 1;
		next_target.checksum_read = words[iAsterisk];
	}
	next_target.checksum_calculated = checksum;

	process_gcode_command();
	serial_writechar('\n');

	seen_mask = 0;
}

void gcode_init(void) {
	// gcc guarantees us all variables are initialised to 0.

	// assume a G1 by default
	next_target.seen_G = 1;
	next_target.G = 1;

	#ifndef E_ABSOLUTE
		next_target.option_e_relative = 1;
	#endif
}

/***************************************************************************\
*                                                                           *
* Request a resend of the current line - used from various places.          *
*                                                                           *
* Relies on the global variable next_target.N being valid.                  *
*                                                                           *
\***************************************************************************/

void request_resend(void) {
	serial_writestr_P(PSTR("rs "));
	serwrite_uint8(next_target.N);
	serial_writechar('\n');
}
