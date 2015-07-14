#ifndef	_HOME_H
#define _HOME_H

void home(GCODE_COMMAND * const cmd);

void home_x_negative(GCODE_COMMAND * const cmd);
void home_x_positive(GCODE_COMMAND * const cmd);
void home_y_negative(GCODE_COMMAND * const cmd);
void home_y_positive(GCODE_COMMAND * const cmd);
void home_z_negative(GCODE_COMMAND * const cmd);
void home_z_positive(GCODE_COMMAND * const cmd);

#endif	/* _HOME_H */
