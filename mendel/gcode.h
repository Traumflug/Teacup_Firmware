#ifndef	_GCODE_H
#define	_GCODE_H

#include	<stdint.h>

#include	"dda.h"

typedef struct {
	uint32_t	sign			:1;
	uint32_t	mantissa	:24;
	uint32_t	exponent	:7;
} decfloat;

typedef struct {
	uint8_t					seen_G	:1;
	uint8_t					seen_M	:1;
	uint8_t					seen_X	:1;
	uint8_t					seen_Y	:1;
	uint8_t					seen_Z	:1;
	uint8_t					seen_E	:1;
	uint8_t					seen_F	:1;
	uint8_t					seen_S	:1;
	uint8_t					seen_P	:1;

	uint8_t					option_relative			:1;
	uint8_t					option_inches				:1;
// 	uint8_t					option_synchronise	:1;

	uint8_t						G;
	uint8_t						M;
	TARGET						target;

	uint16_t					S;
	uint16_t					P;
} GCODE_COMMAND;

extern GCODE_COMMAND next_target;

int8_t indexof(uint8_t c, const char *string);
int32_t	decfloat_to_int(decfloat *df, int32_t multiplicand, int32_t denominator);

void scan_char(uint8_t c);
void process_gcode_command(GCODE_COMMAND *gcmd);

#endif	/* _GCODE_H */
