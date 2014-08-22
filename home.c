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

#ifdef SCARA_PRINTER
/*
	Homing a Scara-type printer like the RepRap Morgan requires a special
	approach. After moving both axes simultaneously to negative until one
	axis reaches the endstop, the axis that didn't reach its endstop, has
	to be moved positive until it reaches its endstop.
	The new function home_xy_negative() returns the axis that has to be moved
	positive. This result is passed directly to home_scara_positive(),
	that moves this axis in a positive direction until it reaches its endstop 
	too.
*/
void home_scara(void) {
#if defined X_MIN_PIN && defined Y_MIN_PIN
	TARGET t = startpoint;

	t.X = -1000000;
	t.Y = -1000000;
    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) // Preprocessor can't check this :-/
		t.F = SEARCH_FAST_X;
    else
		t.F = SEARCH_FEEDRATE_X;
	enqueue_home(&t, 0x3, 1);

	//We need to know which endtop was reached.
	queue_wait();

	t = startpoint;

	if (! x_min() || ! y_min()) {
		//If only one endstop was reached, only this axis should back off
		//Get the axis that reached its endstop and prepare to back off
		if (x_min()) {
			t.X = +1000000;
			if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
				t.F = SEARCH_FEEDRATE_X;
				enqueue_home(&t, 0x1, 0);
			}

			//home the other axis
			t = startpoint;

			t.Y = +1000000;
		    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y)
				t.F = SEARCH_FAST_Y;
			else
				t.F = SEARCH_FEEDRATE_Y;
			enqueue_home(&t, 0x2, 1);

			if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
				t.Y = -1000000;
				t.F = SEARCH_FEEDRATE_Y;
				enqueue_home(&t, 0x2, 0);
			}
		}
	
		if (y_min()) {
			t.Y = +1000000;
			if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
				t.F = SEARCH_FEEDRATE_Y;
				enqueue_home(&t, 0x2, 0);
			}

			//home the other axis
			t = startpoint;

			t.X = +1000000;
		    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X)
				t.F = SEARCH_FAST_X;
			else
				t.F = SEARCH_FEEDRATE_X;
			enqueue_home(&t, 0x1, 1);

			if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
				t.X = -1000000;
				t.F = SEARCH_FEEDRATE_X;
				enqueue_home(&t, 0x1, 0);
			}
		}
	}
	else {
		//If both endstops were reached, we can and should back off both axes (if applicable).
		t.X = +1000000;
		t.Y = +1000000;
		if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
			// back off slowly
			t.F = SEARCH_FEEDRATE_X; //For Scara x- and y-feedartes are always the same
			enqueue_home(&t, 0x3, 0);
		}
		//As both axes have reached their endstops we are finished here.
    }

	// set X home
	queue_wait(); 
	startpoint.X = next_target.target.X = SCARA_HOME_X;
	startpoint.Y = next_target.target.Y = SCARA_HOME_Y;
	dda_new_startpoint();
#else
	/*
		Scara requires x_min- and y_min-endstops.
	*/
	#warning You need to define both, X_MIN_PIN and Y_MIN_PIN, for scara-type printers to work!!
#endif
}
#endif

/// home all 3 axes
void home() {

#ifdef SCARA_PRINTER
	home_scara();
#else
	home_x_negative();
	home_x_positive();

	home_y_negative();
	home_y_positive();
#endif

	home_z_negative();
	home_z_positive();
}

/// find X MIN endstop
void home_x_negative() {
	#if defined X_MIN_PIN
		TARGET t = startpoint;

		t.X = -1000000;
    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) // Preprocessor can't check this :-/
      t.F = SEARCH_FAST_X;
    else
      t.F = SEARCH_FEEDRATE_X;
		enqueue_home(&t, 0x1, 1);

    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
			// back off slowly
			t.X = +1000000;
			t.F = SEARCH_FEEDRATE_X;
			enqueue_home(&t, 0x1, 0);
    }

		// set X home
		queue_wait(); // we have to wait here, see G92
		#ifdef X_MIN
			startpoint.X = next_target.target.X = (int32_t)(X_MIN * 1000.0);
		#else
			startpoint.X = next_target.target.X = 0;
		#endif
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

		t.X = +1000000;
    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X)
      t.F = SEARCH_FAST_X;
    else
      t.F = SEARCH_FEEDRATE_X;
		enqueue_home(&t, 0x1, 1);

    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
			t.X = -1000000;
			t.F = SEARCH_FEEDRATE_X;
			enqueue_home(&t, 0x1, 0);
    }

		// set X home
		queue_wait();
		// set position to MAX
		startpoint.X = next_target.target.X = (int32_t)(X_MAX * 1000.);
		dda_new_startpoint();
		// go to zero
		t.X = 0;
		t.F = MAXIMUM_FEEDRATE_X;
		enqueue(&t);
	#endif
}

/// fund Y MIN endstop
void home_y_negative() {
	#if defined Y_MIN_PIN
		TARGET t = startpoint;

		t.Y = -1000000;
    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y)
      t.F = SEARCH_FAST_Y;
    else
      t.F = SEARCH_FEEDRATE_Y;
		enqueue_home(&t, 0x2, 1);

    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
			t.Y = +1000000;
			t.F = SEARCH_FEEDRATE_Y;
			enqueue_home(&t, 0x2, 0);
    }

		// set Y home
		queue_wait();
		#ifdef	Y_MIN
			startpoint.Y = next_target.target.Y = (int32_t)(Y_MIN * 1000.);
		#else
			startpoint.Y = next_target.target.Y = 0;
		#endif
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

		t.Y = +1000000;
    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y)
      t.F = SEARCH_FAST_Y;
    else
      t.F = SEARCH_FEEDRATE_Y;
		enqueue_home(&t, 0x2, 1);

    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
			t.Y = -1000000;
			t.F = SEARCH_FEEDRATE_Y;
			enqueue_home(&t, 0x2, 0);
    }

		// set Y home
		queue_wait();
		// set position to MAX
		startpoint.Y = next_target.target.Y = (int32_t)(Y_MAX * 1000.);
		dda_new_startpoint();
		// go to zero
		t.Y = 0;
		t.F = MAXIMUM_FEEDRATE_Y;
		enqueue(&t);
	#endif
}

/// find Z MIN endstop
void home_z_negative() {
	#if defined Z_MIN_PIN
		TARGET t = startpoint;

		t.Z = -1000000;
    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z)
      t.F = SEARCH_FAST_Z;
    else
      t.F = SEARCH_FEEDRATE_Z;
		enqueue_home(&t, 0x4, 1);

    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) {
			t.Z = +1000000;
			t.F = SEARCH_FEEDRATE_Z;
			enqueue_home(&t, 0x4, 0);
    }

		// set Z home
		queue_wait();
		#ifdef Z_MIN
			startpoint.Z = next_target.target.Z = (int32_t)(Z_MIN * 1000.);
		#else
			startpoint.Z = next_target.target.Z = 0;
		#endif
		dda_new_startpoint();
		z_disable();
	#endif
}

/// find Z MAX endstop
void home_z_positive() {
	#if defined Z_MAX_PIN && ! defined Z_MAX
		#warning Z_MAX_PIN defined, but not Z_MAX. home_z_positive() disabled.
	#endif
	#if defined Z_MAX_PIN && defined Z_MAX
		TARGET t = startpoint;

		t.Z = +1000000;
    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z)
      t.F = SEARCH_FAST_Z;
    else
      t.F = SEARCH_FEEDRATE_Z;
		enqueue_home(&t, 0x4, 1);

    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) {
			t.Z = -1000000;
			t.F = SEARCH_FEEDRATE_Z;
			enqueue_home(&t, 0x4, 0);
    }

		// set Z home
		queue_wait();
		// set position to MAX
		startpoint.Z = next_target.target.Z = (int32_t)(Z_MAX * 1000.);
		dda_new_startpoint();
		// go to zero
		t.Z = 0;
		t.F = MAXIMUM_FEEDRATE_Z;
		enqueue(&t);
	#endif
}
