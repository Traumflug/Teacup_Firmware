#ifndef	_GCODE_H
#define	_GCODE_H

#include	<stdint.h>

typedef struct {
	int32_t						X;
	int32_t						Y;
	int32_t						Z;
	uint32_t					E;
	uint32_t					F;
} TARGET;

typedef struct {
	uint16_t	sign			:1;
	uint16_t	mantissa	:11;
	uint16_t	exponent	:4;
} decfloat;

typedef struct {
	uint16_t					seen;
#define							SEEN_G	1
#define							SEEN_M	2
#define							SEEN_X	4
#define							SEEN_Y	8
#define							SEEN_Z	16
#define							SEEN_E	32
#define							SEEN_F	64
#define							SEEN_S	128
#define							SEEN_P	256

	uint8_t						option;
#define	OPTION_RELATIVE						1
#define	OPTION_SYNCHRONISE				2
#define	OPTION_UNIT_INCHES				4

	uint8_t						G;
	uint8_t						M;
	TARGET						target;

	uint16_t					S;
	uint16_t					P;
} GCODE_COMMAND;

int8_t indexof(uint8_t c, char *string);
int32_t	decfloat_to_int(decfloat *df, int32_t multiplicand, int32_t denominator);

void scan_char(uint8_t c);
void process_gcode_command(GCODE_COMMAND *gcmd);

#endif	/* _GCODE_H */
