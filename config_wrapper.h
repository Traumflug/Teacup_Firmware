
/**
  Some stuff common to all config.h files. Put it here to allow files like
  board.*.config.h or printer.*.h to be moved to about everywhere in the
  file system. Else we'd have to enforce the Configtool user to put these
  files where we need them.
*/
#include "arduino.h"

#ifndef DEFINE_HEATER_ACTUAL
  #define DEFINE_HEATER_ACTUAL(...)
#endif

// Heater definition helpers.  This collection of macros turns any DEFINE_HEATER(...) macro
// expansion into a DEFINE_HEATER_ACTUAL(a,b,c,d), i.e. a macro with four arguments.  Currently
// we expand 3 to 6 arguments, but we ignore the 6th arg in this code.  This allows
// us to use this "old" code with some "newer" configs which come at us from the future, so long
// as the newer configs add parameters at the end of the list rather than in the middle.
// From http://stackoverflow.com/questions/11761703/overloading-macro-on-number-of-arguments
#define GET_MACRO(_1, _2, _3, _4, _5, _6, NAME, ...) NAME
#define DEFINE_HEATER(...) GET_MACRO(__VA_ARGS__, DHTR_6, DHTR_5, DHTR_4, DHTR_3)(__VA_ARGS__)
#define DHTR_3(name, pin, pwm) DEFINE_HEATER_ACTUAL(name, pin, 0, pwm, 100)
#define DHTR_4(name, pin, invert, pwm) DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, 100)
#define DHTR_5(name, pin, invert, pwm, max_pwm)  DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, max_pwm)
#define DHTR_6(name, pin, invert, pwm, max_pwm, arg6)  DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, max_pwm)

#ifndef DEFINE_HOMING_ACTUAL
  #define DEFINE_HOMING_ACTUAL(...)
#endif

// Homing movements helpers.
#define GET_MACRO_HOME(_1, _2, _3, _4, NAME, ...) NAME
#define DEFINE_HOMING(...) GET_MACRO_HOME(__VA_ARGS__, DHM_4, DHM_3, DHM_2, DHM_1)(__VA_ARGS__)
#define DHM_1(STEP_1) DEFINE_HOMING_ACTUAL(STEP_1, none, none, none)
#define DHM_2(STEP_1, STEP_2) DEFINE_HOMING_ACTUAL(STEP_1, STEP_2, none, none)
#define DHM_3(STEP_1, STEP_2, STEP_3) DEFINE_HOMING_ACTUAL(STEP_1, STEP_2, STEP_3, none)
#define DHM_4(STEP_1, STEP_2, STEP_3, STEP_4) DEFINE_HOMING_ACTUAL(STEP_1, STEP_2, STEP_3, STEP_4)

/**
  This wrapper config header is used to allow makefiles and test scripts to
  replace or augment the user's 'config.h' file in a controlled manner. A
  makefile may add CFLAGS+=-DUSER_CONFIG=alternate_config.h to cause Teacup
  to build with a different config header.
*/
#ifndef USER_CONFIG
#define USER_CONFIG "config.h"
#endif

#include USER_CONFIG

#include "simulator.h"

/**
  Give users a hint in case they obviously forgot to read instructions.
*/
#ifndef STEPS_PER_M_X
  #error config.h missing. Please follow instructions at \
    reprap.org/wiki/Teacup_Firmware#Simple_Installation
#endif

/**
  Additional tests to protect against misconfiguration.
*/
#ifdef USB_SERIAL
  #undef BAUD
#endif

/**
  Check wether we need SPI.
*/
#if (defined SD_CARD_SELECT_PIN || defined TEMP_MAX6675 || \
     defined TEMP_MC3008) && ! defined SIMULATOR
  #define SPI
#endif

/**
  Check wether we need I2C.
*/
#ifdef DISPLAY_BUS_I2C
  #define I2C
#endif

/**
  ACCELERATION_TEMPORAL doesn't support lookahead, yet.
*/
#if defined ACCELERATION_TEMPORAL && defined LOOKAHEAD
  #warning Acceleration temporal doesnt support lookahead, yet. \
           Lookahead disabled.
  #undef LOOKAHEAD
#endif

/**
  For ACCELERATION_REPRAP or no acceleration at all, lookahead makes no sense.
  Explicitely disable it to deal with misconfigurations.
*/
#if ! defined ACCELERATION_RAMPING && ! defined ACCELERATION_TEMPORAL
  #undef LOOKAHEAD
#endif

/**
  LOOKAHEAD won't work if Z-jerk is zero and bed leveling is active
  because most moves will have Z-steps and lookahead will be skipped.
*/
#if defined BED_LEVELING && defined LOOKAHEAD && MAX_JERK_Z==0
  #warning When bed-leveling is activated, lookahead will be ineffective \
           because MAX_JERK_Z is zero.
#endif

/**
  Silently discard EECONFIG on ARM. Silently to not disturb regression tests.

  TODO:

   - EECONFIG is currently misplaced as a printer property. Move EECONFIG to
     the board configuration or drop it entirely in favour of PID settings in
     Configtool.

   - Remove this silent discard in favour of the #error in heater-arm.c.
*/
#if defined __ARMEL__ && defined EECONFIG
  #undef EECONFIG
#endif

/**
  Silently discard BANG_BANG on ARM. Silently to not disturb regression tests.

  TODO:

   - BANG_BANG is currently misplaced as a printer property. Move BANG_BANG to
     the board configuration.

   - Remove this silent discard in favour of the #error in heater-arm.c.
*/
#if defined __ARMEL__ && defined BANG_BANG
  #undef BANG_BANG
#endif

/**
 * TEMP_EWMA changed from float 0.0 - 1.0 to integer 1000
 * Some people could accidentally change the 1.0 to 1 which will have effect
 * in very slow temperature readings. Let's force any value <= 1.0 to 1000
 * and give them a warning.
*/
#if TEMP_EWMA < 2
#undef TEMP_EWMA
#define TEMP_EWMA 1000
#warning TEMP_EWMA scaling changed! Take a look in your printer.xxx.h.
#endif

/**
 * With GCC major >= 5 we can calculate constant sqrt expressions
 * lower versions, e.g. in the Arduino IDE, need for this a
 * __builtin_ function.
 * 
 * "warning: initializer element is not a constant expression"
 * 
 * Someone may extend this later also for sin/cos.
*/
#if (__GNUC__ <= 4)
#define sqrt __builtin_sqrt
#endif
