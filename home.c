#include "home.h"
#include "bed_leveling.h"
/** \file
  \brief Homing routines
*/

#include <math.h>
#include "dda.h"
#include "dda_queue.h"
#include "pinio.h"
#include "gcode_parse.h"

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

static const uint32_t PROGMEM fast_feedrate_P[3] = {
  (SEARCH_FAST_X > SEARCH_FEEDRATE_X) ? SEARCH_FAST_X : SEARCH_FEEDRATE_X,
  (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) ? SEARCH_FAST_Y : SEARCH_FEEDRATE_Y,
  (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) ? SEARCH_FAST_Z : SEARCH_FEEDRATE_Z,
};

static const uint32_t PROGMEM search_feedrate_P[3] = {
  (SEARCH_FAST_X > SEARCH_FEEDRATE_X) ? SEARCH_FEEDRATE_X : 0,
  (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) ? SEARCH_FEEDRATE_Y : 0,
  (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) ? SEARCH_FEEDRATE_Z : 0,
};

uint8_t get_endstop_check(enum axis_e n, int8_t dir);
void home_axis(enum axis_e n, int8_t dir, enum axis_endstop_e endstop_check);
void set_axis_home_position(enum axis_e n, int8_t dir);

/// home all 3 axes
void home() {
  #ifdef DEFINE_HOMING_ACTUAL
    #undef DEFINE_HOMING_ACTUAL
      #define DEFINE_HOMING_ACTUAL(first, second, third, fourth) \
        { \
          home_##first(); \
          home_##second(); \
          home_##third(); \
          home_##fourth(); \
        };
      #include "config_wrapper.h"
    #undef DEFINE_HOMING_ACTUAL
  #endif
}

void home_none() {
}

/// find X MIN endstop
void home_x_negative() {
  #if defined X_MIN_PIN
    home_axis(X, -1, X_MIN_ENDSTOP);
  #endif
}

/// find X_MAX endstop
void home_x_positive() {
  #if defined X_MAX_PIN && ! defined X_MAX
    #warning X_MAX_PIN defined, but not X_MAX. home_x_positive() disabled.
  #endif
  #if defined X_MAX_PIN && defined X_MAX
    home_axis(X, 1, X_MAX_ENDSTOP);
  #endif
}

/// fund Y MIN endstop
void home_y_negative() {
  #if defined Y_MIN_PIN
    home_axis(Y, -1, Y_MIN_ENDSTOP);
  #endif
}

/// find Y MAX endstop
void home_y_positive() {
  #if defined Y_MAX_PIN && ! defined Y_MAX
    #warning Y_MAX_PIN defined, but not Y_MAX. home_y_positive() disabled.
  #endif
  #if defined Y_MAX_PIN && defined Y_MAX
    home_axis(Y, 1, Y_MAX_ENDSTOP);
  #endif
}

/// find Z MIN endstop
void home_z_negative() {
  #if defined Z_MIN_PIN
    home_axis(Z, -1, Z_MIN_ENDSTOP);
  #endif
}

/// find Z MAX endstop
void home_z_positive() {
  #if defined Z_MAX_PIN && ! defined Z_MAX
    #warning Z_MAX_PIN defined, but not Z_MAX. home_z_positive() disabled.
  #endif
  #if defined Z_MAX_PIN && defined Z_MAX
    home_axis(Z, 1, Z_MAX_ENDSTOP);
  #endif
}

void home_axis(enum axis_e n, int8_t dir, enum axis_endstop_e endstop_check) {
  TARGET t = startpoint;
  startpoint.axis[n] = 0;

  t.axis[n] = dir * MAX_DELTA_UM;
  t.F = pgm_read_dword(&fast_feedrate_P[n]);
  enqueue_home(&t, endstop_check, 1);

  uint32_t search_feedrate;
  search_feedrate = pgm_read_dword(&search_feedrate_P[n]);
  if (search_feedrate) {
    // back off slowly
    t.axis[n] = 0;
    t.F = search_feedrate;
    enqueue_home(&t, endstop_check, 0);
  }

  queue_wait();
  set_axis_home_position(n, dir);
  dda_new_startpoint();

#ifdef BED_LEVELING
  // Move to calculated Z-plane offset
  if (n==Z)
    enqueue(&next_target.target);
#endif /* BED_LEVELING */
}

void set_axis_home_position(enum axis_e n, int8_t dir) {
  int32_t home_position = 0;
  if (dir < 0) {
    if (n == X) {
      #ifdef X_MIN
      home_position = (int32_t)(X_MIN * 1000);
      #endif
    }
    else if (n == Y) {
      #ifdef Y_MIN
      home_position = (int32_t)(Y_MIN * 1000);
      #endif
    }
    else if (n == Z) {
      #ifdef Z_MIN
      home_position = (int32_t)(Z_MIN * 1000);
      #endif
    }
  }
  else {
    if (n == X) {
      #ifdef X_MAX
      home_position = (int32_t)(X_MAX * 1000);
      #endif
    }
    else if (n == Y) {
      #ifdef Y_MAX
      home_position = (int32_t)(Y_MAX * 1000);
      #endif
    }
    else if (n == Z) {
      #ifdef Z_MAX
      home_position = (int32_t)(Z_MAX * 1000);
      #endif
    }
  }
  startpoint.axis[n] = next_target.target.axis[n] = home_position;
  if (n == Z) {
    // Compensate for z-offset that will be added in by next move
    startpoint.axis[n] -= bed_level_offset(startpoint.axis);
  }
}
