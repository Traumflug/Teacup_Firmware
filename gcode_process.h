#ifndef	_GCODE_PROCESS_H
#define	_GCODE_PROCESS_H

#include	"gcode_parse.h"

enum gcode_source {
  GCODE_SOURCE_SERIAL  = 0b00000001,
  GCODE_SOURCE_SD      = 0b00000010,
};


extern enum gcode_source gcode_sources;

// the current tool
extern uint8_t tool;
// the tool to be changed when we get an M6
extern uint8_t next_tool;

// when we have a whole line, feed it to this
void process_gcode_command(void);

#endif	/* _GCODE_PROCESS_H */
