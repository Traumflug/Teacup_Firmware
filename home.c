
/** \file
	\brief Homing routines
*/

#include <math.h>
#include	"dda.h"
#include	"dda_queue.h"
#include	"pinio.h"
#include	"gcode_parse.h"
#include	"home.h"

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
void home(GCODE_COMMAND * const cmd) {

  home_x_negative(cmd);
  home_x_positive(cmd);

  home_y_negative(cmd);
  home_y_positive(cmd);

  home_z_negative(cmd);
  home_z_positive(cmd);
}

/// find X MIN endstop
void home_x_negative(GCODE_COMMAND * const cmd) {
	#if defined X_MIN_PIN
		TARGET t = startpoint;

    t.axis[X] = -1000000;
    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) // Preprocessor can't check this :-/
      t.F = SEARCH_FAST_X;
    else
      t.F = SEARCH_FEEDRATE_X;
    enqueue_home(&t, 0x01, 1);

    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
			// back off slowly
      t.axis[X] = +1000000;
			t.F = SEARCH_FEEDRATE_X;
      enqueue_home(&t, 0x01, 0);
    }

		// set X home
		queue_wait(); // we have to wait here, see G92
		#ifdef X_MIN
      startpoint.axis[X] = cmd->target.axis[X] = (int32_t)(X_MIN * 1000.0);
		#else
      startpoint.axis[X] = cmd->target.axis[X] = 0;
		#endif
		dda_new_startpoint();
	#endif
}

/// find X_MAX endstop
void home_x_positive(GCODE_COMMAND * const cmd) {
	#if defined X_MAX_PIN && ! defined X_MAX
		#warning X_MAX_PIN defined, but not X_MAX. home_x_positive() disabled.
	#endif
	#if defined X_MAX_PIN && defined X_MAX
		TARGET t = startpoint;

    t.axis[X] = +1000000;
    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X)
      t.F = SEARCH_FAST_X;
    else
      t.F = SEARCH_FEEDRATE_X;
    enqueue_home(&t, 0x02, 1);

    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
      t.axis[X] = -1000000;
			t.F = SEARCH_FEEDRATE_X;
      enqueue_home(&t, 0x02, 0);
    }

		// set X home
		queue_wait();
		// set position to MAX
    startpoint.axis[X] = cmd->target.axis[X] = (int32_t)(X_MAX * 1000.);
		dda_new_startpoint();
	#endif
}

/// fund Y MIN endstop
void home_y_negative(GCODE_COMMAND * const cmd) {
	#if defined Y_MIN_PIN
		TARGET t = startpoint;

    t.axis[Y] = -1000000;
    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y)
      t.F = SEARCH_FAST_Y;
    else
      t.F = SEARCH_FEEDRATE_Y;
    enqueue_home(&t, 0x04, 1);

    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
      t.axis[Y] = +1000000;
			t.F = SEARCH_FEEDRATE_Y;
      enqueue_home(&t, 0x04, 0);
    }

		// set Y home
		queue_wait();
		#ifdef	Y_MIN
      startpoint.axis[Y] = cmd->target.axis[Y] = (int32_t)(Y_MIN * 1000.);
		#else
      startpoint.axis[Y] = cmd->target.axis[Y] = 0;
		#endif
		dda_new_startpoint();
	#endif
}

/// find Y MAX endstop
void home_y_positive(GCODE_COMMAND * const cmd) {
	#if defined Y_MAX_PIN && ! defined Y_MAX
		#warning Y_MAX_PIN defined, but not Y_MAX. home_y_positive() disabled.
	#endif
	#if defined Y_MAX_PIN && defined Y_MAX
		TARGET t = startpoint;

    t.axis[Y] = +1000000;
    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y)
      t.F = SEARCH_FAST_Y;
    else
      t.F = SEARCH_FEEDRATE_Y;
    enqueue_home(&t, 0x08, 1);

    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
      t.axis[Y] = -1000000;
			t.F = SEARCH_FEEDRATE_Y;
      enqueue_home(&t, 0x08, 0);
    }

		// set Y home
		queue_wait();
		// set position to MAX
    startpoint.axis[Y] = cmd->target.axis[Y] = (int32_t)(Y_MAX * 1000.);
		dda_new_startpoint();
	#endif
}

/// find Z MIN endstop
void home_z_negative(GCODE_COMMAND * const cmd) {
	#if defined Z_MIN_PIN
		TARGET t = startpoint;

    t.axis[Z] = -1000000;
    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z)
      t.F = SEARCH_FAST_Z;
    else
      t.F = SEARCH_FEEDRATE_Z;
    enqueue_home(&t, 0x10, 1);

    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) {
      t.axis[Z] = +1000000;
			t.F = SEARCH_FEEDRATE_Z;
      enqueue_home(&t, 0x10, 0);
    }

		// set Z home
		queue_wait();
		#ifdef Z_MIN
      startpoint.axis[Z] = cmd->target.axis[Z] = (int32_t)(Z_MIN * 1000.);
		#else
      startpoint.axis[Z] = cmd->target.axis[Z] = 0;
		#endif
		dda_new_startpoint();
	#endif
}

/// find Z MAX endstop
void home_z_positive(GCODE_COMMAND * const cmd) {
	#if defined Z_MAX_PIN && ! defined Z_MAX
		#warning Z_MAX_PIN defined, but not Z_MAX. home_z_positive() disabled.
	#endif
	#if defined Z_MAX_PIN && defined Z_MAX
		TARGET t = startpoint;

    t.axis[Z] = +1000000;
    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z)
      t.F = SEARCH_FAST_Z;
    else
      t.F = SEARCH_FEEDRATE_Z;
    enqueue_home(&t, 0x20, 1);

    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) {
      t.axis[Z] = -1000000;
			t.F = SEARCH_FEEDRATE_Z;
      enqueue_home(&t, 0x20, 0);
    }

		// set Z home
		queue_wait();
		// set position to MAX
    startpoint.axis[Z] = cmd->target.axis[Z] = (int32_t)(Z_MAX * 1000.);
		dda_new_startpoint();
	#endif
}
