#ifndef	_GCODE_PROCESS_H
#define	_GCODE_PROCESS_H

#include	"gcode_parse.h"

// this is where we construct a move without a gcode command, useful for gcodes which require multiple moves eg; homing
void SpecialMoveXY(int32_t x, int32_t y, uint32_t f);
void SpecialMoveZ(int32_t z, uint32_t f);
void SpecialMoveE(int32_t e, uint32_t f);

// when we have a whole line, feed it to this
void process_gcode_command(void);

#endif	/* _GCODE_PROCESS_H */
