#include	"home.h"

#include	"dda.h"
#include	"dda_queue.h"
#include	"delay.h"
#include	"pinio.h"

void home() {
	queue_wait();

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
	
	#if defined	Z_MIN_PIN
		home_z_negative();
	#elif defined Z_MAX_PIN
		home_z_positive();
	#endif
}

void home_x_negative() {
	#if defined X_MIN_PIN
		uint8_t	denoise_count = 0;

		// home X
		// hit home hard
		x_direction(0);
		while (denoise_count < 8) {
			// denoise
			if (x_min())
				denoise_count++;
			else
				denoise_count = 0;
			// step
			x_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_X / ((float) MAXIMUM_FEEDRATE_X)));
		}
		denoise_count = 0;

		// back off slowly
		x_direction(1);
		while (x_min() != 0) {
			// step
			x_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_X / ((float) SEARCH_FEEDRATE_X)));
		}

		// set X home
		startpoint.X = current_position.X = 0;
	#endif
}
	
void home_x_positive() {
	#if defined X_MAX_PIN
		uint8_t	denoise_count = 0;

		// home X
		// hit home hard
		x_direction(1);
		while (denoise_count < 8) {
			// denoise
			if (x_max())
				denoise_count++;
			else
				denoise_count = 0;
			// step
			x_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_X / ((float) MAXIMUM_FEEDRATE_X)));
		}
		denoise_count = 0;

		// back off slowly
		x_direction(0);
		while (x_max() != 0) {
			// step
			x_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_X / ((float) SEARCH_FEEDRATE_X)));
		}

		// set X home
		TARGET t = {0, 0, 0, 0, 0};
		// set position to MAX
		startpoint.X = current_position.X = (int32_t) (X_MAX * STEPS_PER_MM_X);
		// go to zero
		t.F = MAXIMUM_FEEDRATE_X;
		enqueue(&t);
	#endif
}
		
void home_y_negative() {
	#if defined Y_MIN_PIN
		uint8_t	denoise_count = 0;

		// home Y
		// hit home hard
		y_direction(0);
		while (denoise_count < 8) {
			// denoise
			if (y_min())
				denoise_count++;
			else
				denoise_count = 0;
			// step
			y_step();
			delay(5);
			unstep();
			// wait until neyt step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Y / ((float) MAXIMUM_FEEDRATE_Y)));
		}
		denoise_count = 0;

		// back off slowly
		y_direction(1);
		while (y_min() != 0) {
			// step
			y_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Y / ((float) SEARCH_FEEDRATE_Y)));
		}

		// set Y home
		startpoint.Y = current_position.Y = 0;
	#endif
}

void home_y_positive() {
	#if defined Y_MAX_PIN
		uint8_t	denoise_count = 0;

		// home Y
		// hit home hard
		y_direction(1);
		while (denoise_count < 8) {
			// denoise
			if (y_max())
				denoise_count++;
			else
				denoise_count = 0;
			// step
			y_step();
			delay(5);
			unstep();
			// wait until neyt step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Y / ((float) MAXIMUM_FEEDRATE_Y)));
		}
		denoise_count = 0;

		// back off slowly
		y_direction(0);
		while (y_max() != 0) {
			// step
			y_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Y / ((float) SEARCH_FEEDRATE_Y)));
		}

		// set Y home
		TARGET t = {0, 0, 0, 0, 0};
		// set position to MAX
		startpoint.Y = current_position.Y = (int32_t) (Y_MAX * STEPS_PER_MM_Y);
		// go to zero
		t.F = MAXIMUM_FEEDRATE_Y;
		enqueue(&t);
	#endif
}

void home_z_negative() {
	#if defined Z_MIN_PIN
		uint8_t	denoise_count = 0;

		// home Z
		// hit home hard
		z_direction(0);
		while (denoise_count < 8) {
			// denoise
			if (z_min())
				denoise_count++;
			else
				denoise_count = 0;
			// step
			z_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Z / ((float) MAXIMUM_FEEDRATE_Z)));
		}
		denoise_count = 0;

		// back off slowly
		z_direction(1);
		while (z_min() != 0) {
			// step
			z_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Z / ((float) SEARCH_FEEDRATE_Z)));
		}

		// set Z home
		startpoint.Z = current_position.Z = 0;
	#endif
}

void home_z_positive() {
	#if defined Z_MAX_PIN
		uint8_t	denoise_count = 0;

		// home Z
		// hit home hard
		z_direction(1);
		while (denoise_count < 8) {
			// denoise
			if (z_max())
				denoise_count++;
			else
				denoise_count = 0;
			// step
			z_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Z / ((float) MAXIMUM_FEEDRATE_Z)));
		}
		denoise_count = 0;

		// back off slowly
		z_direction(0);
		while (z_max() != 0) {
			// step
			z_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Z / ((float) SEARCH_FEEDRATE_Z)));
		}

		// set Z home
		TARGET t = {0, 0, 0, 0, 0};
		// set position to MAX
		startpoint.Z = current_position.Z = (int32_t) (Z_MAX * STEPS_PER_MM_Z);
		// go to zero
		t.F = MAXIMUM_FEEDRATE_Z;
		enqueue(&t);
	#endif
}
