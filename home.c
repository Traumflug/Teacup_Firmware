#include	"home.h"

/** \file
	\brief Homing routines
*/

#include <math.h>
#include	"dda.h"
#include	"dda_queue.h"
#include	"pinio.h"
#include	"gcode_parse.h"

// Check configuration.
#if defined X_MIN_PIN || defined X_MAX_PIN
  #ifndef SEARCH_FEEDRATE_X
    #error SEARCH_FEEDRATE_X undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_X
    #error ENDSTOP_CLEARANCE_X undefined. It should be defined in config.h.
  #endif
#endif
#if defined Y_MIN_PIN || defined Y_MAX_PIN
  #ifndef SEARCH_FEEDRATE_Y
    #error SEARCH_FEEDRATE_Y undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_Y
    #error ENDSTOP_CLEARANCE_Y undefined. It should be defined in config.h.
  #endif
#endif
#if defined Z_MIN_PIN || defined Z_MAX_PIN
  #ifndef SEARCH_FEEDRATE_Z
    #error SEARCH_FEEDRATE_Z undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_Z
    #error ENDSTOP_CLEARANCE_Z undefined. It should be defined in config.h.
  #endif
#endif

// Calculate feedrates according to clearance and deceleration.
// For a description, see #define ENDSTOP_CLEARANCE_{XYZ} in config.h.
//   s = 1/2 * a * t^2; t = v / a  <==> v = sqrt(2 * a * s))
//   units: / 1000 for um -> mm; * 60 for mm/s -> mm/min
#ifdef ENDSTOP_CLEARANCE_X
  #define SEARCH_FAST_X (uint32_t)((double)60. * \
            sqrt((double)2 * ACCELERATION * ENDSTOP_CLEARANCE_X / 1000.))
#endif
#ifdef ENDSTOP_CLEARANCE_Y
  #define SEARCH_FAST_Y (uint32_t)((double)60. * \
            sqrt((double)2 * ACCELERATION * ENDSTOP_CLEARANCE_Y / 1000.))
#endif
#ifdef ENDSTOP_CLEARANCE_Z
  #define SEARCH_FAST_Z (uint32_t)((double)60. * \
            sqrt((double)2 * ACCELERATION * ENDSTOP_CLEARANCE_Z / 1000.))
#endif


/// home all 3 axes
void home() {

  home_x_negative();
  home_x_positive();

  home_y_negative();
  home_y_positive();

  home_z_negative();
  home_z_positive();
}

#ifdef X_MIN
  #define X_MIN_VAL X_MIN
#else
  #define X_MIN_VAL 0
#endif
#ifdef Y_MIN
  #define Y_MIN_VAL Y_MIN
#else
  #define Y_MIN_VAL 0
#endif
#ifdef Z_MIN
  #define Z_MIN_VAL Z_MIN
#else
  #define Z_MIN_VAL 0
#endif

#ifdef X_MAX
  #define X_MAX_VAL X_MAX
#else
  #define X_MAX_VAL 0
#endif
#ifdef Y_MAX
  #define Y_MAX_VAL Y_MAX
#else
  #define Y_MAX_VAL 0
#endif
#ifdef Z_MAX
  #define Z_MAX_VAL Z_MAX
#else
  #define Z_MAX_VAL 0
#endif

/// \var search_feedrate_P
/// \brief desired feedrate for homing on each axis except E
static const axes_int32_t PROGMEM axis_min_P = {
  X_MIN_VAL * 1000.0,
  Y_MIN_VAL * 1000.0,
  Z_MIN_VAL * 1000.0
};
static const axes_int32_t PROGMEM axis_max_P = {
  X_MAX_VAL * 1000.0,
  Y_MAX_VAL * 1000.0,
  Z_MAX_VAL * 1000.0
};

#if defined X_MIN_PIN || defined Y_MIN_PIN || defined Z_MIN_PIN \
 || defined X_MAX_PIN || defined Y_MAX_PIN || defined Z_MAX_PIN
/// find endstop
/// @param axis axis to home
/// @param endstop_check check value to use for enqueue_home
/// @param direction +1 for positive; -1 for negative
static void home_axis(enum axis_e axis, uint8_t endstop_check,
                      int direction, int32_t end_position) {
  TARGET t = startpoint;

  t.axis[axis] = +1000000 * direction;
#warning This patch was written before adaptive homing feedrates were
#warning created, so ENDSTOP_CLEARANCE is not respected.
  #ifdef SLOW_HOMING
    // hit home soft
    t.F = pgm_read_dword(&search_feedrate_P[axis]);
  #else
    // hit home hard
    t.F = pgm_read_dword(&maximum_feedrate_P[axis]);
  #endif
  enqueue_home(&t, endstop_check, 1);

  #ifndef SLOW_HOMING
    // back off slowly
    t.axis[axis] = -1000000 * direction;
    t.F = pgm_read_dword(&search_feedrate_P[axis]);
    enqueue_home(&t, endstop_check, 0);
  #endif

  // set home
  queue_wait(); // we have to wait here, see G92
  startpoint.axis[axis] = next_target.target.axis[axis] = end_position;
  dda_new_startpoint();
}
#endif

#if defined X_MIN_PIN || defined Y_MIN_PIN || defined Z_MIN_PIN
/// Home an axis in the negative direction
static void home_negative(enum axis_e axis) {
  home_axis(axis, 1<<axis, -1, axis_min_P[axis]);
}
#endif

#if defined X_MAX_PIN || defined Y_MAX_PIN || defined Z_MAX_PIN
/// Home an axis in the positive direction
static void home_positive(enum axis_e axis) {
  home_axis(axis, 1<<axis, +1, axis_max_P[axis]);
}
#endif

/// find X MIN endstop
void home_x_negative() {
#if defined X_MIN_PIN
  home_negative(X);
#endif
}

/// find X MAX endstop
void home_x_positive() {
  #if defined X_MAX_PIN && ! defined X_MAX
    #warning X_MAX_PIN defined, but not X_MAX. home_x_positive() disabled.
  #endif
  #if defined X_MAX_PIN && defined X_MAX
    home_positive(X);
  #endif
}

/// find Y MIN endstop
void home_y_negative() {
#if defined Y_MIN_PIN
  home_negative(Y);
#endif
}

/// find Y MAX endstop
void home_y_positive() {
	#if defined Y_MAX_PIN && ! defined Y_MAX
		#warning Y_MAX_PIN defined, but not Y_MAX. home_y_positive() disabled.
	#endif
	#if defined Y_MAX_PIN && defined Y_MAX
    home_positive(Y);
	#endif
}

/// find Z MIN endstop
void home_z_negative() {
	#if defined Z_MIN_PIN
    home_negative(Z);
	#endif
}

/// find Z MAX endstop
void home_z_positive() {
	#if defined Z_MAX_PIN && ! defined Z_MAX
		#warning Z_MAX_PIN defined, but not Z_MAX. home_z_positive() disabled.
	#endif
	#if defined Z_MAX_PIN && defined Z_MAX
    home_positive(Z);
	#endif
}
