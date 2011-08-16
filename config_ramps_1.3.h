#ifndef _CONFIG_RAMPS_1_3_H
#define _CONFIG_RAMPS_1_3_H

/*
	This is for the RAMPS v1.3 shield
*/

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
//#define	Z_MAX_PIN   					DIO19
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
//#define	SD_WRITE_PROTECT			DIO3

/*
 * TODO: 20110810 SJL - Fix this workaround for using the high inputs of a 16 input mux!
 *
 * NOTE: For the highest 8 inputs of a 16 channel mux, the correct 'pin' definition
 *       gives a wrong result because only the bit position in the (byte) port is
 *       defined and the port information isn't used. A workaround for this is to
 *       manually add 8 to the AIO pin setting starting with AIO8_PIN and up.
 *       E.g. AIO13_PIN should be written as (AIO13_PIN + 8).
 */
// Thermistor inputs (input 1 on THERM0, 2 on THERM1, etc)
#define	ANALOG_INPUT_1					(AIO13_PIN + 8)
#define	ANALOG_INPUT_2					(AIO14_PIN + 8)
#define	ANALOG_INPUT_3					(AIO15_PIN + 8)

// PWM power outputs (output 1 on Q1, 2 on Q2, etc)

// This is a bit clumsy, but needed because of the smart way the
// variable lists are constructed with the preprocessor.
// The configuration file should use PWM_OUTPUT_1 etc. for reference.
#if 0
#define PWM_OUTPUT_1					PB4
#define PWM_OUTPUT_2					PH5
#define PWM_OUTPUT_3					PH6
#else
#define PWM_OUTPUT_1_WPORT				PB4_WPORT
#define PWM_OUTPUT_1_PIN				PB4_PIN
#define PWM_OUTPUT_1_PWM				PB4_PWM
#define PWM_OUTPUT_1_DDR				PB4_DDR

#define PWM_OUTPUT_2_WPORT				PH5_WPORT
#define PWM_OUTPUT_2_PIN				PH5_PIN
#define PWM_OUTPUT_2_PWM				PH5_PWM
#define PWM_OUTPUT_2_DDR				PH5_DDR

#define PWM_OUTPUT_3_WPORT				PH6_WPORT
#define PWM_OUTPUT_3_PIN				PH6_PIN
#define PWM_OUTPUT_3_PWM				PH6_PWM
#define PWM_OUTPUT_3_DDR				PH6_DDR
#endif

#endif	// _CONFIG_RAMPS_1_3_H

