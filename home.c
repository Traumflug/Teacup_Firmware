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

//Select position of endstops
#ifdef X_MIN_ENDSTOP_POSITION
#define X_HOME_MIN	((int32_t)(X_MIN_ENDSTOP_POSITION * 1000.0))
#elif defined X_MIN
#define X_HOME_MIN	((int32_t)(X_MIN * 1000.0))
#else
#define X_HOME_MIN	(0)
#endif

#ifdef X_MAX_ENDSTOP_POSITION
#define X_HOME_MAX	((int32_t)(X_MAX_ENDSTOP_POSITION * 1000.0))
#elif defined X_MAX
#define X_HOME_MAX	((int32_t)(X_MAX * 1000.0))
#else
#define X_HOME_MAX	(0)
#endif

#ifdef Y_MIN_ENDSTOP_POSITION
#define Y_HOME_MIN	((int32_t)(Y_MIN_ENDSTOP_POSITION * 1000.0))
#elif defined Y_MIN
#define Y_HOME_MIN	((int32_t)(Y_MIN * 1000.0))
#else
#define Y_HOME_MIN	(0)
#endif

#ifdef Y_MAX_ENDSTOP_POSITION
#define Y_HOME_MAX	((int32_t)(Y_MAX_ENDSTOP_POSITION * 1000.0))
#elif defined Y_MAX
#define Y_HOME_MAX	((int32_t)(Y_MAX * 1000.0))
#else
#define Y_HOME_MAX	(0)
#endif

#ifdef Z_MIN_ENDSTOP_POSITION
#define Z_HOME_MIN	((int32_t)(Z_MIN_ENDSTOP_POSITION * 1000.0))
#elif defined Z_MIN
#define Z_HOME_MIN	((int32_t)(Z_MIN * 1000.0))
#else
#define Z_HOME_MIN	(0)
#endif

#ifdef Z_MAX_ENDSTOP_POSITION
#define Z_HOME_MAX	((int32_t)(Z_MAX_ENDSTOP_POSITION * 1000.0))
#elif defined Z_MAX
#define Z_HOME_MAX	((int32_t)(Z_MAX * 1000.0))
#else
#define Z_HOME_MAX	(0)
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

/// find X MIN endstop
void home_x_negative() {
	#if defined X_MIN_PIN
		TARGET t = startpoint;

    t.axis[X] = -1000000;
    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) // Preprocessor can't check this :-/
      t.F = SEARCH_FAST_X;
    else
      t.F = SEARCH_FEEDRATE_X;
		enqueue_home(&t, 0x1, 1);

    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
			// back off slowly
      t.axis[X] = +1000000;
			t.F = SEARCH_FEEDRATE_X;
			enqueue_home(&t, 0x1, 0);
    }

		// set X home
		queue_wait(); // we have to wait here, see G92
		
		//PK Correct value is selected at the top of this file
      startpoint.axis[X] = next_target.target.axis[X] = X_HOME_MIN;
		dda_new_startpoint();
	#endif
}

/// find X_MAX endstop
void home_x_positive() {
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
		enqueue_home(&t, 0x1, 1);

    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
      t.axis[X] = -1000000;
			t.F = SEARCH_FEEDRATE_X;
			enqueue_home(&t, 0x1, 0);
    }

		// set X home
		queue_wait();
		// set position to MAX
		//PK X_HOME_MAX instead of X_MAX
    startpoint.axis[X] = next_target.target.axis[X] = X_HOME_MAX;
		dda_new_startpoint();
	#endif
	
	//PK Added going to real home position
	#ifdef X_HOME_POSITION
		t.axis[X] = (int32_t)(X_HOME_POSITION * 1000.);;
		t.F = MAXIMUM_FEEDRATE_X;
		enqueue(&t);
	#endif	
}

/// fund Y MIN endstop
void home_y_negative() {
	#if defined Y_MIN_PIN
		TARGET t = startpoint;

    t.axis[Y] = -1000000;
    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y)
      t.F = SEARCH_FAST_Y;
    else
      t.F = SEARCH_FEEDRATE_Y;
		enqueue_home(&t, 0x2, 1);

    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
      t.axis[Y] = +1000000;
			t.F = SEARCH_FEEDRATE_Y;
			enqueue_home(&t, 0x2, 0);
    }

		// set Y home
		queue_wait();
		//PK Correct value is selected at the top of this file
      startpoint.axis[Y] = next_target.target.axis[Y] = Y_HOME_MIN;
		dda_new_startpoint();
	#endif
}

/// find Y MAX endstop
void home_y_positive() {
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
		enqueue_home(&t, 0x2, 1);

    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
      t.axis[Y] = -1000000;
			t.F = SEARCH_FEEDRATE_Y;
			enqueue_home(&t, 0x2, 0);
    }

		// set Y home
		queue_wait();
		// set position to MAX
		//PK Y_HOME_MAX instead of Y_MAX
    startpoint.axis[Y] = next_target.target.axis[Y] = Y_HOME_MAX;
		dda_new_startpoint();
	#endif
	
	//PK Added going to real home position
	#ifdef Y_HOME_POSITION
		t.axis[Y] = (int32_t)(Y_HOME_POSITION * 1000.);;
		t.F = MAXIMUM_FEEDRATE_Y;
		enqueue(&t);
	#endif
}

/// find Z MIN endstop
void home_z_negative() {
	#if defined Z_MIN_PIN
		TARGET t = startpoint;

    t.axis[Z] = -1000000;
    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z)
      t.F = SEARCH_FAST_Z;
    else
      t.F = SEARCH_FEEDRATE_Z;
		enqueue_home(&t, 0x4, 1);

    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) {
      t.axis[Z] = +1000000;
			t.F = SEARCH_FEEDRATE_Z;
			enqueue_home(&t, 0x4, 0);
    }

		// set Z home
		queue_wait();
		//PK Correct value is selected at the top of this file
      startpoint.axis[Z] = next_target.target.axis[Z] = Z_HOME_MIN;
		dda_new_startpoint();
	#endif
}

/// find Z MAX endstop
void home_z_positive() {
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
		enqueue_home(&t, 0x4, 1);

    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) {
      t.axis[Z] = -1000000;
			t.F = SEARCH_FEEDRATE_Z;
			enqueue_home(&t, 0x4, 0);
    }

		// set Z home
		queue_wait();
		// set position to MAX
		//PK Z_HOME_MAX instead of Z_MAX
    startpoint.axis[Z] = next_target.target.axis[Z] = Z_HOME_MAX;
		dda_new_startpoint();
	#endif
	
	//PK Added going to real home position
	#ifdef Z_HOME_POSITION
		t.axis[Z] = (int32_t)(Z_HOME_POSITION * 1000.);;
		t.F = MAXIMUM_FEEDRATE_Z;
		enqueue(&t);
	#endif
}
