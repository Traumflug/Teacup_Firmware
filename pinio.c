#include	"pinio.h"

void power_on() {

	#ifdef	STEPPER_ENABLE_PIN
		#ifdef	STEPPER_ENABLE_INVERT
			WRITE(STEPPER_ENABLE_PIN, 0);
		#else
			WRITE(STEPPER_ENABLE_PIN, 1);
		#endif
	#endif
	#ifdef	PS_ON_PIN
		WRITE(PS_ON_PIN, 0);
		SET_OUTPUT(PS_ON_PIN);
	#endif
}

void power_off() {

	x_disable();
	y_disable();
	z_disable();

	#ifdef	STEPPER_ENABLE_PIN
		#ifdef	STEPPER_ENABLE_INVERT
			WRITE(STEPPER_ENABLE_PIN, 1);
		#else
			WRITE(STEPPER_ENABLE_PIN, 0);
		#endif
	#endif
	#ifdef	PS_ON_PIN
		SET_INPUT(PS_ON_PIN);
	#endif
}
