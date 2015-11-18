/* Test: Does the code still compile without endstops?
 *
 * This config file is defined to test this, at least on
 * one processor.
 */
#define CPU                      atmega644
#ifndef F_CPU
#define F_CPU                    20000000UL
#endif
#define MOTHERBOARD

//#define TX_ENABLE_PIN            xxxx
//#define RX_ENABLE_PIN            xxxx

#define X_STEP_PIN               DIO19
#define X_DIR_PIN                DIO18
//#define X_MIN_PIN                DIO7
//#define X_MAX_PIN                DIO6
//#define X_ENABLE_PIN             xxxx
//#define X_INVERT_DIR
//#define X_INVERT_MIN
//#define X_INVERT_MAX
//#define X_INVERT_ENABLE

#define Y_STEP_PIN               DIO23
#define Y_DIR_PIN                DIO22
//#define Y_MIN_PIN                DIO5
//#define Y_MAX_PIN                DIO2
//#define Y_ENABLE_PIN             xxxx
//#define Y_INVERT_DIR
//#define Y_INVERT_MIN
//#define Y_INVERT_MAX
//#define Y_INVERT_ENABLE

#define Z_STEP_PIN               DIO26
#define Z_DIR_PIN                DIO25
//#define Z_MIN_PIN                DIO1
//#define Z_MAX_PIN                DIO0
//#define Z_ENABLE_PIN             xxxx
//#define Z_INVERT_DIR
//#define Z_INVERT_MIN
//#define Z_INVERT_MAX
//#define Z_INVERT_ENABLE

#define E_STEP_PIN               DIO28
#define E_DIR_PIN                DIO27
//#define E_ENABLE_PIN             xxxx
//#define E_INVERT_DIR
//#define E_INVERT_ENABLE

#define PS_ON_PIN                DIO15
//#define PS_MOSFET_PIN            xxxx
#define STEPPER_ENABLE_PIN       DIO24
#define STEPPER_INVERT_ENABLE

#define SD_CARD_SELECT_PIN       DIO10

#ifndef DEFINE_TEMP_SENSOR
  #define DEFINE_TEMP_SENSOR(...)
#endif

#define TEMP_THERMISTOR
DEFINE_TEMP_SENSOR(extruder, TT_THERMISTOR, AIO1,  THERMISTOR_EXTRUDER)
DEFINE_TEMP_SENSOR(bed,      TT_THERMISTOR, AIO2,  THERMISTOR_BED)

#ifndef DEFINE_HEATER
  #define DEFINE_HEATER(...)
#endif

DEFINE_HEATER(extruder, DIO4,    0,      1)
DEFINE_HEATER(bed,      DIO3,    0,      1)

#define HEATER_EXTRUDER HEATER_extruder
#define HEATER_BED HEATER_bed
#define BAUD                     115200
