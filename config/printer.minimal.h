#define KINEMATICS_STRAIGHT
//#define KINEMATICS_COREXY
#define STEPS_PER_M_X            100
#define STEPS_PER_M_Y            200
#define STEPS_PER_M_Z            300
#define STEPS_PER_M_E            400
#define MAXIMUM_FEEDRATE_X       500
#define MAXIMUM_FEEDRATE_Y       600
#define MAXIMUM_FEEDRATE_Z       700
#define MAXIMUM_FEEDRATE_E       800
#define SEARCH_FEEDRATE_X        900
#define SEARCH_FEEDRATE_Y        1000
#define SEARCH_FEEDRATE_Z        1100
#define ENDSTOP_CLEARANCE_X      0
#define ENDSTOP_CLEARANCE_Y      0
#define ENDSTOP_CLEARANCE_Z      0
//#define X_MIN                    0
//#define X_MAX                    0
//#define Y_MIN                    0
//#define Y_MAX                    0
//#define Z_MIN                    0
//#define Z_MAX                    0
//#define E_ABSOLUTE

/**
  FIXME: It has become clear no one is using ACCELERATION_REPRAP since it
  no longer compiles.  If we define this acceleration, this project no longer
  compiles.  Should we fix the project or remove the unneeded acceleration mode?
*/

//#define ACCELERATION_REPRAP
//#define ACCELERATION_RAMPING
//#define ACCELERATION_TEMPORAL
//#define ACCELERATION             xxxx
//#define LOOKAHEAD
//#define MAX_JERK_X               0
//#define MAX_JERK_Y               0
//#define MAX_JERK_Z               0
//#define MAX_JERK_E               0
//#define USE_INTERNAL_PULLUPS
//#define Z_AUTODISABLE
#define TEMP_HYSTERESIS          6
#define TEMP_RESIDENCY_TIME      12
//#define TEMP_EWMA                1.0
//#define REPORT_TARGET_TEMPS
//#define HEATER_SANITY_CHECK
//#define EECONFIG
//#define BANG_BANG
//#define BANG_BANG_ON             xxxx
//#define BANG_BANG_OFF            xxxx
#define MOVEBUFFER_SIZE          3
//#define DC_EXTRUDER              xxxx
//#define DC_EXTRUDER_PWM          xxxx
//#define USE_WATCHDOG
#define TH_COUNT                 8
//#define FAST_PWM
#define PID_SCALE                1024L
#define ENDSTOP_STEPS            4
//#defined CANNED_CYCLE xxxx
