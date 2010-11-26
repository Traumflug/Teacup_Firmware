#include	"pinio.h"

void power_off() {
	#ifdef	X_ENABLE_PIN
		x_disable();
	#endif
	#ifdef	Y_ENABLE_PIN
		y_disable();
	#endif
	#ifdef	Z_ENABLE_PIN
		z_disable();
	#endif

	#ifdef	STEPPER_ENABLE_PIN
	WRITE(STEPPER_ENABLE_PIN, STEPPER_ENABLE_INVERT ^ 1)
	#endif
	#ifdef	PS_ON_PIN
		SET_INPUT(PS_ON_PIN);
	#endif
}
