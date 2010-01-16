#ifndef	_GCODE_H
#define	_GCODE_H

#include	"target.h"

typedef struct {
	uint16_t					seen;
#define							SEEN_G	1
#define							SEEN_M	2
#define							SEEN_X	4
#define							SEEN_Y	8
#define							SEEN_Z	16
#define							SEEN_E	32
#define							SEEN_F	64

	uint8_t						option;
#define	OPTION_RELATIVE						1
#define	OPTION_SYNCHRONISE				2
#define	OPTION_HOME_WHEN_COMPLETE	4
#define	OPTION_UNIT_INCHES				8

	uint8_t						G;
	uint8_t						M;
	TARGET						target;
} GCODE_COMMAND;

void scan_char(uint8_t c);
void process_gcode_command(GCODE_COMMAND *gcmd);

#endif	/* _GCODE_H */