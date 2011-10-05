#include	"home.h"

/** \file
	\brief Homing routines
*/

#include	"dda.h"
#include	"dda_queue.h"
#include	"delay.h"
#include	"pinio.h"

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

	#if defined Z_MAX_PIN
		home_z_positive();
	#elif defined	Z_MIN_PIN
		home_z_negative();
	#endif
}

/// find X MIN endstop
void home_x_negative() {
	#if defined X_MIN_PIN
		TARGET t = {0, current_position.Y, current_position.Z};

		// hit home hard
		t.X = -1000*STEPS_PER_MM_X;
		t.F = MAXIMUM_FEEDRATE_X;
		enqueue_home(&t, 0x1, 1);

		// back off slowly
		t.X = +1000*STEPS_PER_MM_X;
		t.F = SEARCH_FEEDRATE_X;
		enqueue_home(&t, 0x1, 0);

		// set X home
		#ifdef X_MIN
			startpoint.X = current_position.X = (int32_t) (X_MIN * STEPS_PER_MM_X);
		#else
			startpoint.X = current_position.X = 0;
		#endif
	#endif
}

/// find X_MAX endstop
void home_x_positive() {
	#if defined X_MAX_PIN
		TARGET t = {0, current_position.Y, current_position.Z};

		// hit home hard
		t.X = +1000*STEPS_PER_MM_X;
		t.F = MAXIMUM_FEEDRATE_X;
		enqueue_home(t, 0x1, 1);

		// back off slowly
		t.X = -1000*STEPS_PER_MM_X;
		t.F = SEARCH_FEEDRATE_X;
		enqueue_home(t, 0x1, 0);

		// set X home
		// set position to MAX
		startpoint.X = current_position.X = (int32_t) (X_MAX * STEPS_PER_MM_X);
		// go to zero
		t.X = 0;
		t.F = MAXIMUM_FEEDRATE_X;
		enqueue(&t);
	#endif
}

/// fund Y MIN endstop
void home_y_negative() {
	#if defined Y_MIN_PIN
		TARGET t = {0, current_position.Y, current_position.Z};

		// hit home hard
		t.Y = -1000*STEPS_PER_MM_Y;
		t.F = MAXIMUM_FEEDRATE_Y;
		enqueue_home(&t, 0x2, 1);

		// back off slowly
		t.Y = +1000*STEPS_PER_MM_Y;
		t.F = SEARCH_FEEDRATE_Y;
		enqueue_home(&t, 0x2, 0);

		// set Y home
		#ifdef	Y_MIN
			startpoint.Y = current_position.Y = (int32_t) (Y_MIN * STEPS_PER_MM_Y);
		#else
			startpoint.Y = current_position.Y = 0;
		#endif
	#endif
}

/// find Y MAX endstop
void home_y_positive() {
	#if defined Y_MAX_PIN
		TARGET t = {0, current_position.Y, current_position.Z};

		// hit home hard
		t.Y = +1000*STEPS_PER_MM_Y;
		t.F = MAXIMUM_FEEDRATE_Y;
		enqueue_home(&t, 0x2, 1);

		// back off slowly
		t.X = -1000*STEPS_PER_MM_Y;
		t.F = SEARCH_FEEDRATE_Y;
		enqueue_home(&t, 0x2, 0);

		// set Y home
		// set position to MAX
		startpoint.Y = current_position.Y = (int32_t) (Y_MAX * STEPS_PER_MM_Y);
		// go to zero
		t.Y = 0;
		t.F = MAXIMUM_FEEDRATE_Y;
		enqueue(&t);
	#endif
}

/// find Z MIN endstop
void home_z_negative() {
	#if defined Z_MIN_PIN
		TARGET t = {current_position.X, current_position.Y, 0};

		// hit home hard
		t.Z = -1000*STEPS_PER_MM_Z;
		t.F = MAXIMUM_FEEDRATE_Z;
		enqueue_home(&t, 0x4, 1);

		// back off slowly
		t.Z = +1000*STEPS_PER_MM_Z;
		t.F = SEARCH_FEEDRATE_Z;
		enqueue_home(&t, 0x4, 0);

		// set Z home
		#ifdef Z_MIN
			startpoint.Z = current_position.Z = (int32_t) (Z_MIN * STEPS_PER_MM_Z);
		#else
			startpoint.Z = current_position.Z = 0;
		#endif
		z_disable();
	#endif
}

/// find Z MAX endstop
void home_z_positive() {
	#if defined Z_MAX_PIN
		TARGET t = {current_position.X, current_position.Y, 0};

		// hit home hard
		t.Z = +1000*STEPS_PER_MM_Z;
		t.F = MAXIMUM_FEEDRATE_Z;
		enqueue_home(&t, 0x4, 1);

		// back off slowly
		t.Z = -1000*STEPS_PER_MM_Z;
		t.F = SEARCH_FEEDRATE_Z;
		enqueue_home(&t, 0x4, 0);

		// set Z home
		// set position to MAX
		startpoint.Z = current_position.Z = (int32_t) (Z_MAX * STEPS_PER_MM_Z);
		// go to zero
		t.Z = 0;
		t.F = MAXIMUM_FEEDRATE_Z;
		enqueue(&t);
	#endif
}
