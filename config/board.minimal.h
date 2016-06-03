//#define CPU_TYPE atmega644
#define CPU                      atmega644p
//#define F_CPU_OPT f_cpu_opt
#ifndef F_CPU
#define F_CPU                    16000000UL
#endif
#define MOTHERBOARD
#define TX_ENABLE_PIN            DIO12
#define RX_ENABLE_PIN            DIO13
#define X_STEP_PIN               DIO15
#define X_DIR_PIN                DIO18
//#define X_MIN_PIN                xxxx
//#define X_MAX_PIN                xxxx
#define X_ENABLE_PIN             DIO19
//#define X_INVERT_DIR
//#define X_INVERT_MIN
//#define X_INVERT_MAX
//#define X_INVERT_ENABLE
#define Y_STEP_PIN               DIO23
#define Y_DIR_PIN                DIO22
//#define Y_MIN_PIN                xxxx
//#define Y_MAX_PIN                xxxx
#define Y_ENABLE_PIN             DIO24
//#define Y_INVERT_DIR
//#define Y_INVERT_MIN
//#define Y_INVERT_MAX
//#define Y_INVERT_ENABLE
#define Z_STEP_PIN               DIO27
#define Z_DIR_PIN                DIO28
//#define Z_MIN_PIN                xxxx
//#define Z_MAX_PIN                xxxx
#define Z_ENABLE_PIN             DIO29
//#define Z_INVERT_DIR
//#define Z_INVERT_MIN
//#define Z_INVERT_MAX
//#define Z_INVERT_ENABLE
#define E_STEP_PIN               DIO17
#define E_DIR_PIN                DIO16
//#define E_ENABLE_PIN             xxxx
//#define E_INVERT_DIR
//#define E_INVERT_ENABLE

#define PS_ON_PIN                DIO14
//#define PS_MOSFET_PIN            xxxx
//#define STEPPER_ENABLE_PIN       xxxx
//#define STEPPER_INVERT_ENABLE
//#define DEBUG_LED_PIN            xxxx
//#define SD_CARD_SELECT_PIN       xxxx
//#define MCP3008_SELECT_PIN       xxxx
#ifndef DEFINE_TEMP_SENSOR
  #define DEFINE_TEMP_SENSOR(...)
#endif
//#define TEMP_MAX6675
//#define TEMP_THERMISTOR
//#define TEMP_AD595
//#define TEMP_PT100
#define TEMP_INTERCOM
//#define TEMP_MCP3008
//#define TEMP_SENSOR_PIN xxxx
//#define TEMP_SENSOR_PIN xxxx
//#define TEMP_SENSOR_PIN xxxx
//DEFINE_TEMP_SENSORS_START
//                 name      type           pin    additional
DEFINE_TEMP_SENSOR(noheater, TT_INTERCOM,   AIO0,  0)

// Beta algorithm      r0      beta  r2    vadc
// Steinhart-Hart      rp      t0    r0      t1    r1      t2    r2
//DEFINE_TEMP_SENSORS_END
#ifndef DEFINE_HEATER
  #define DEFINE_HEATER(...)
#endif
//#define HEATER_PIN xxxx
//DEFINE_HEATERS_START
//            name      pin      invert  pwm
//DEFINE_HEATERS_END
#define BAUD                     115200
//#define XONXOFF
//#define USB_SERIAL
//#define DISPLAY_BUS_4BIT
//#define DISPLAY_BUS_8BIT
//#define DISPLAY_BUS_I2C
//#define DISPLAY_BUS_SPI
//#define DISPLAY_RS_PIN           xxxx
//#define DISPLAY_RW_PIN           xxxx
//#define DISPLAY_E_PIN            xxxx
//#define DISPLAY_D4_PIN           xxxx
//#define DISPLAY_D5_PIN           xxxx
//#define DISPLAY_D6_PIN           xxxx
//#define DISPLAY_D7_PIN           xxxx
//#define DISPLAY_TYPE_SSD1306
//#define DISPLAY_TYPE_HD44780
