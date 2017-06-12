#ifndef	_HOME_H
#define _HOME_H
#include	<avr/eeprom.h>
extern int32_t EEMEM EE_real_zmax;
void home(void);

void home_x_negative(void);
void home_x_positive(void);
void home_y_negative(void);
void home_y_positive(void);
void home_z_negative(void);
void home_z_positive(void);
void home_set_zmax(uint32_t z,int relative);

#endif	/* _HOME_H */
