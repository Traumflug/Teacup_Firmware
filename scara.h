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


