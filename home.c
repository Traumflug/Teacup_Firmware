#include	"home.h"

/** \file
	\brief Homing routines
*/

#include	"dda.h"
#include	"dda_queue.h"
#include	"pinio.h"
#include	"gcode_parse.h"

/// home all 3 axes
void home() {
	#if defined	X_MIN_PIN
		home_x_negative();
	#elif defined X_MAX_PIN
		home_x_positive();
	#endif

	#if defined	Y_MIN_PIN
		home_y_negative();
	#elif defined Y_MAX_PIN
		home_y_positive();
	#endif

	#if defined Z_MIN_PIN
		home_z_negative();
	#elif defined	Z_MAX_PIN
		home_z_positive();
	#endif
}

/// find X MIN endstop
void home_x_negative() {
	#if defined X_MIN_PIN
		TARGET t = startpoint;

		t.X = -1000000;
		#ifdef SLOW_HOMING
			// hit home soft
			t.F = SEARCH_FEEDRATE_X;
		#else
			// hit home hard
			t.F = MAXIMUM_FEEDRATE_X;
		#endif
		enqueue_home(&t, 0x1, 1);

		#ifndef SLOW_HOMING
			// back off slowly
			t.X = +1000000;
			t.F = SEARCH_FEEDRATE_X;
			enqueue_home(&t, 0x1, 0);
		#endif

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
		#ifdef SLOW_HOMING
			// hit home soft
			t.F = SEARCH_FEEDRATE_X;
		#else
			// hit home hard
			t.F = MAXIMUM_FEEDRATE_X;
		#endif
		enqueue_home(&t, 0x1, 1);

		#ifndef SLOW_HOMING
			// back off slowly
			t.X = -1000000;
			t.F = SEARCH_FEEDRATE_X;
			enqueue_home(&t, 0x1, 0);
		#endif

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
		#ifdef SLOW_HOMING
			// hit home soft
			t.F = SEARCH_FEEDRATE_Y;
		#else
			// hit home hard
			t.F = MAXIMUM_FEEDRATE_Y;
		#endif
		enqueue_home(&t, 0x2, 1);

		#ifndef SLOW_HOMING
			// back off slowly
			t.Y = +1000000;
			t.F = SEARCH_FEEDRATE_Y;
			enqueue_home(&t, 0x2, 0);
		#endif

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
		#ifdef SLOW_HOMING
			// hit home soft
			t.F = SEARCH_FEEDRATE_Y;
		#else
			// hit home hard
			t.F = MAXIMUM_FEEDRATE_Y;
		#endif
		enqueue_home(&t, 0x2, 1);

		#ifndef SLOW_HOMING
			// back off slowly
			t.Y = -1000000;
			t.F = SEARCH_FEEDRATE_Y;
			enqueue_home(&t, 0x2, 0);
		#endif

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
		#ifdef SLOW_HOMING
			// hit home soft
			t.F = SEARCH_FEEDRATE_Z;
		#else
			// hit home hard
			t.F = MAXIMUM_FEEDRATE_Z;
		#endif
		enqueue_home(&t, 0x4, 1);

		#ifndef SLOW_HOMING
			// back off slowly
			t.Z = +1000000;
			t.F = SEARCH_FEEDRATE_Z;
			enqueue_home(&t, 0x4, 0);
		#endif

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
		#ifdef SLOW_HOMING
			// hit home soft
			t.F = SEARCH_FEEDRATE_Z;
		#else
			// hit home hard
			t.F = MAXIMUM_FEEDRATE_Z;
		#endif
		enqueue_home(&t, 0x4, 1);

		#ifndef SLOW_HOMING
			// back off slowly
			t.Z = -1000000;
			t.F = SEARCH_FEEDRATE_Z;
			enqueue_home(&t, 0x4, 0);
		#endif

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
