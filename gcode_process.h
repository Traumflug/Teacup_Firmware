#ifndef	_GCODE_PROCESS_H
#define	_GCODE_PROCESS_H

#include	"gcode_parse.h"


void zero_x(void);
void zero_y(void);
void zero_z(void);
void zero_e(void);

// this is where we construct a move without a gcode command, useful for gcodes which require multiple moves eg; homing
void SpecialMoveE(int32_t e, uint32_t f);

// when we have a whole line, feed it to this
void process_gcode_command(void);

#endif	/* _GCODE_PROCESS_H */
