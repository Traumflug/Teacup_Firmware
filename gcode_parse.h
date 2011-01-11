#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include	<stdint.h>

#include	"dda.h"

// wether the asterisk (checksum-command) is included for checksum calculation
// undefined for RepRap host software
//#define ASTERISK_IN_CHECKSUM_INCLUDED

// wether to insist on N line numbers
// if not defined, N's are completely ignored
//#define	REQUIRE_LINENUMBER

// wether to insist on a checksum
//#define	REQUIRE_CHECKSUM

// this is a very crude decimal-based floating point structure.
// a real floating point would at least have signed exponent.
// max (DECFLOAT_EXP_MAX - 1) digits after decimal point, because point is
// counted as well
#define DECFLOAT_EXP_WIDTH 3					
#define DECFLOAT_EXP_MAX  ((1 << DECFLOAT_EXP_WIDTH) - 1)
#define DECFLOAT_MANT_WIDTH (32 - 1 - DECFLOAT_EXP_WIDTH)
#define DECFLOAT_MANT_MAX (((uint32_t)1 << DECFLOAT_MANT_WIDTH) - 1)
typedef struct {
	uint32_t	sign			:1;
	uint32_t	mantissa	:DECFLOAT_MANT_WIDTH;
	uint32_t	exponent	:DECFLOAT_EXP_WIDTH;
} decfloat;

// this holds all the possible data from a received command
typedef struct {
	union {
		struct {
			uint8_t					seen_G	:1;
			uint8_t					seen_M	:1;
			uint8_t					seen_X	:1;
			uint8_t					seen_Y	:1;
			uint8_t					seen_Z	:1;
			uint8_t					seen_E	:1;
			uint8_t					seen_F	:1;
			uint8_t					seen_S	:1;

			uint8_t					seen_P	:1;
			uint8_t					seen_T	:1;
			uint8_t					seen_N	:1;
			uint8_t					seen_checksum				:1;
			uint8_t					seen_semi_comment		:1;
			uint8_t					seen_parens_comment	:1;
			uint8_t					option_relative			:1;
			uint8_t					option_inches				:1;
		};
		uint16_t				flags;
	};

	uint8_t						G;
	uint8_t						M;
	TARGET						target;

	int16_t						S;
	uint16_t					P;

	uint8_t						T;

	uint32_t					N;
	uint32_t					N_expected;

	uint8_t						checksum_read;
	uint8_t						checksum_calculated;
} GCODE_COMMAND;

// the command being processed
extern GCODE_COMMAND next_target;

// accept the next character and process it
void gcode_parse_char(uint8_t c);

// uses the global variable next_target.N
void request_resend(void);

#endif	/* _GCODE_PARSE_H */
