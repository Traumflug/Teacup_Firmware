#ifndef	_HOME_H
#define _HOME_H

// 20110814 modmaker - Parked these definitions here, don't want to put them
//                     in the configuration files because they're rather static.
//
// beware to use only partial axis speed when approaching the switch because
// we've got no acceleration implemented here!
#define HOME_FEED			600		/*[mm/min]*/
// use fixed feed of 120 [mm/min] to run from the switch (assume any
// axis can run at this speed)
#define RELEASE_FEED		120		/*[mm/min]*/
#define RELEASE_DISTANCE	5.0 	/*[mm]*/

void home(void);

void home_x_negative(void);
void home_x_positive(void);
void home_y_negative(void);
void home_y_positive(void);
void home_z_negative(void);
void home_z_positive(void);

#endif	/* _HOME_H */
