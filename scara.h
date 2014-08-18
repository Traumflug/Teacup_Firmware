/*
	This file contains all defines, that are uniaue to Scara-style printers. 
	It will only be included, if SCARA_PRINTER is defines in config.h
*/

/*
	Set the exact length of the inner and outer Scara-arm in micrometers (default: 150000 µm for RepRap Morgan)
	The length is measured from the center of both axes of the arm, not the outer length!
*/
#define INNER_ARM_LENGTH 150000UL
#define OUTER_ARM_LENGTH 150000UL

/*
	The relative position of the central axis of a Scara-type printer from the bed's zero position
	(x=0, y=0; top right corner of the print space) should be accurate, as it defines the position of the
	print space in the "Scara-world".
*/
#define SCARA_TOWER_OFFSET_X	100000L
#define SCARA_TOWER_OFFSET_Y	-62000L

/*
	We need the approximate home-positions for the homing procedure.
	Home-positon shoud be measured relatively to the print aera's zero coordinates (x=0, y=0),
	which for scara are at to top-right of the bed (looking towards the pole/top-platform).
	The positon doesn't need to be very accurate because it only gives a rough startpoint for the homing
	procedure.
*/
#define SCARA_HOME_X	25000L
#define SCARA_HOME_Y	-60000L



