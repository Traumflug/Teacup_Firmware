#include	"home.h"

#include	"dda.h"
#include	"dda_queue.h"
#include	"delay.h"
#include	"pinio.h"


// F is mm/min
// DELAY is clocks per step
// 1 clock = 1 second/F_CPU
// intermediaries are steps/mm and F_CPU per min
// F mm/min * steps_per_mm = steps/min
// clocks per min = 60 * F_CPU
// clocks/min / (steps/min) = clocks per step
// so delay = 60 * F_CPU / steps_per_mm / F
// #define	FEEDRATE_TO_DELAY(F)	60.0 * ((float) F_CPU) / STEPS_PER_MM_X / F

void home() {
	#if (! defined X_MIN_PIN) || (! defined Y_MIN_PIN) || (! defined Z_MIN_PIN)
	TARGET t = {0, 0, 0, 0, 0};
	#endif
	queue_wait();

	uint8_t	denoise_count = 0;

	// home X
	#if defined X_MIN_PIN || defined X_MAX_PIN
		// hit home hard
		#ifdef	X_MIN_PIN
		x_direction(0);
		#else
		x_direction(1);
		#endif
		while (denoise_count < 8) {
			// denoise
			#ifdef	X_MIN_PIN
			if (x_min())
			#else
			if (x_max())
			#endif
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
		#ifdef	X_MIN_PIN
		x_direction(1);
		while (x_min() != 0) {
		#else
		x_direction(0);
		while (x_max() != 0) {
		#endif
			// step
			x_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_X / ((float) SEARCH_FEEDRATE_X)));
		}

		// set X home
		#ifdef	X_MIN_PIN
			startpoint.X = current_position.X = 0;
		#else
			// set position to MAX
			startpoint.X = current_position.X = (int32_t) (X_MAX * STEPS_PER_MM_X);
			// go to zero
			t.F = MAXIMUM_FEEDRATE_X;
			enqueue(&t);
		#endif
	#endif

	// home Y
	#if defined Y_MIN_PIN || defined Y_MAX_PIN
		// hit home hard
		#ifdef	Y_MIN_PIN
		y_direction(0);
		#else
		y_direction(1);
		#endif
		while (denoise_count < 8) {
			// denoise
			#ifdef	Y_MIN_PIN
			if (y_min())
			#else
			if (y_max())
			#endif
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
		#ifdef	Y_MIN_PIN
		y_direction(1);
		while (y_min() != 0) {
		#else
		y_direction(0);
		while (y_max() != 0) {
			#endif
			// step
			y_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Y / ((float) SEARCH_FEEDRATE_Y)));
		}

		// set Y home
		#ifdef	Y_MIN_PIN
			startpoint.Y = current_position.Y = 0;
		#else
			// set position to MAX
			startpoint.Y = current_position.Y = (int32_t) (Y_MAX * STEPS_PER_MM_Y);
			// go to zero
			t.F = MAXIMUM_FEEDRATE_Y;
			enqueue(&t);
		#endif
	#endif

	// home Z
	#if defined Z_MIN_PIN || defined Z_MAX_PIN
		// hit home hard
		#ifdef	Z_MIN_PIN
		z_direction(0);
		#else
		z_direction(1);
		#endif
		while (denoise_count < 8) {
			// denoise
			#ifdef	Z_MIN_PIN
			if (z_min())
			#else
			if (z_max())
			#endif
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
		#ifdef	Z_MIN_PIN
		z_direction(1);
		while (z_min() != 0) {
		#else
		z_direction(0);
		while (z_max() != 0) {
			#endif
			// step
			z_step();
			delay(5);
			unstep();
			// wait until next step time
			delay((uint32_t) (60.0 * ((float) F_CPU) / STEPS_PER_MM_Z / ((float) SEARCH_FEEDRATE_Z)));
		}

		// set Z home
		#ifdef	Z_MIN_PIN
			startpoint.Z = current_position.Z = 0;
		#else
			// set position to MAX
			startpoint.Z = current_position.Z = (int32_t) (Z_MAX * STEPS_PER_MM_Z);
			// go to zero
			t.F = MAXIMUM_FEEDRATE_Z;
			enqueue(&t);
		#endif
	#endif
}
