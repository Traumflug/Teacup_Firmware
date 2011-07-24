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
	power_on();
	queue_wait();
	stepper_enable();
	x_enable();

	#if defined X_MIN_PIN
		uint8_t	debounce_count = 0;

		// hit home hard
		x_direction(0);
		while (debounce_count < 8) {
			// debounce
			if (x_min())
				debounce_count++;
			else
				debounce_count = 0;
			// step
			x_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_X / ((float) MAXIMUM_FEEDRATE_X)));
		}
		debounce_count = 0;

		// back off slowly
		x_direction(1);
		while (x_min() != 0) {
			// step
			x_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_X / ((float) SEARCH_FEEDRATE_X)));
		}

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
	power_on();
	queue_wait();
	stepper_enable();
	x_enable();

	#if defined X_MAX_PIN
		uint8_t	debounce_count = 0;

		// hit home hard
		x_direction(1);
		while (debounce_count < 8) {
			// debounce
			if (x_max())
				debounce_count++;
			else
				debounce_count = 0;
			// step
			x_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_X / ((float) MAXIMUM_FEEDRATE_X)));
		}
		debounce_count = 0;

		// back off slowly
		x_direction(0);
		while (x_max() != 0) {
			// step
			x_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_X / ((float) SEARCH_FEEDRATE_X)));
		}

		// set X home
		// set position to MAX
		startpoint.X = current_position.X = (int32_t) (X_MAX * STEPS_PER_MM_X);
		// go to zero
		TARGET t = {0, 0, 0, 0, MAXIMUM_FEEDRATE_X};
		enqueue(&t);
	#endif
}

/// fund Y MIN endstop
void home_y_negative() {
	power_on();
	queue_wait();
	stepper_enable();
	y_enable();

	#if defined Y_MIN_PIN
		uint8_t	debounce_count = 0;

		// hit home hard
		y_direction(0);
		while (debounce_count < 8) {
			// debounce
			if (y_min())
				debounce_count++;
			else
				debounce_count = 0;
			// step
			y_step();
			delay(5);
			unstep();
			// wait until neyt step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_Y / ((float) MAXIMUM_FEEDRATE_Y)));
		}
		debounce_count = 0;

		// back off slowly
		y_direction(1);
		while (y_min() != 0) {
			// step
			y_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_Y / ((float) SEARCH_FEEDRATE_Y)));
		}

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
	power_on();
	queue_wait();
	stepper_enable();
	y_enable();

	#if defined Y_MAX_PIN
		uint8_t	debounce_count = 0;

		// hit home hard
		y_direction(1);
		while (debounce_count < 8) {
			// debounce
			if (y_max())
				debounce_count++;
			else
				debounce_count = 0;
			// step
			y_step();
			delay(5);
			unstep();
			// wait until neyt step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_Y / ((float) MAXIMUM_FEEDRATE_Y)));
		}
		debounce_count = 0;

		// back off slowly
		y_direction(0);
		while (y_max() != 0) {
			// step
			y_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_Y / ((float) SEARCH_FEEDRATE_Y)));
		}

		// set Y home
		// set position to MAX
		startpoint.Y = current_position.Y = (int32_t) (Y_MAX * STEPS_PER_MM_Y);
		// go to zero
		TARGET t = {0, 0, 0, 0, MAXIMUM_FEEDRATE_Y};
		enqueue(&t);
	#endif
}

/// find Z MIN endstop
void home_z_negative() {
	power_on();
	queue_wait();
	stepper_enable();
	z_enable();

	#if defined Z_MIN_PIN
		uint8_t	debounce_count = 0;

		// hit home hard
		z_direction(0);
		while (debounce_count < 8) {
			// debounce
			if (z_min())
				debounce_count++;
			else
				debounce_count = 0;
			// step
			z_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_Z / ((float) MAXIMUM_FEEDRATE_Z)));
		}
		debounce_count = 0;

		// back off slowly
		z_direction(1);
		while (z_min() != 0) {
			// step
			z_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_Z / ((float) SEARCH_FEEDRATE_Z)));
		}

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
	power_on();
	queue_wait();
	stepper_enable();
	z_enable();

	#if defined Z_MAX_PIN
		uint8_t	debounce_count = 0;

		// hit home hard
		z_direction(1);
		while (debounce_count < 8) {
			// debounce
			if (z_max())
				debounce_count++;
			else
				debounce_count = 0;
			// step
			z_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_Z / ((float) MAXIMUM_FEEDRATE_Z)));
		}
		debounce_count = 0;

		// back off slowly
		z_direction(0);
		while (z_max() != 0) {
			// step
			z_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * 1000000.0 / STEPS_PER_MM_Z / ((float) SEARCH_FEEDRATE_Z)));
		}

		// set Z home:
		// set position to MAX
		startpoint.Z = current_position.Z = (int32_t) (Z_MAX * STEPS_PER_MM_Z);

		// go to zero
		// TARGET t = {0, 0, 0, 0, MAXIMUM_FEEDRATE_Z};
		// enqueue(&t);
	#endif
}
