#ifndef _RAMPS_V1_3_H
#define _RAMPS_V1_3_H

/*
	Machine Pin Definitions
	- make sure to avoid duplicate usage of a pin
	- comment out pins not in use, as this drops the corresponding code and makes operations faster
*/

#include	"arduino.h"


/*
	This is for the RAMPS v1.3 shield
*/
// TODO: 20110813 SJL - the following two are not yet used&verified for RAMPS1.3
//#define TX_ENABLE_PIN					DIO12
//#define	RX_ENABLE_PIN					DIO13

#define	X_STEP_PIN  					AIO0
#define	X_DIR_PIN   					AIO1
#define	X_MIN_PIN   					DIO3
//#define	X_MAX_PIN   					DIO2
//#define	X_ENABLE_PIN					DIO38
//#define	X_INVERT_DIR
//#define	X_INVERT_MIN
//#define	X_INVERT_MAX
//#define	X_INVERT_ENABLE

#define	Y_STEP_PIN  					AIO6
#define	Y_DIR_PIN   					AIO7
#define	Y_MIN_PIN   					DIO14
//#define	Y_MAX_PIN   					DIO15
//#define	Y_ENABLE_PIN					AIO2
#define	Y_INVERT_DIR
//#define	Y_INVERT_MIN
//#define	Y_INVERT_MAX
//#define	Y_INVERT_ENABLE

#define	Z_STEP_PIN  					DIO46
#define	Z_DIR_PIN   					DIO48
#define	Z_INVERT_DIR
#define	Z_MIN_PIN   					DIO18
#define	Z_MAX_PIN   					DIO19
//#define	Z_ENABLE_PIN					AIO8
//#define	Z_INVERT_MIN
//#define	Z_INVERT_MAX
//#define	Z_INVERT_ENABLE

#define	E_STEP_PIN  					DIO26
#define	E_DIR_PIN   					DIO28
//#define	E_ENABLE_PIN					DIO24
//#define	E_INVERT_DIR

// TODO: 20110813 SJL - the following two are not yet used&verified for RAMPS1.3
//#define	SD_CARD_DETECT				DIO2
//#define	SD_WRITE_PROTECT			DIO

#endif

