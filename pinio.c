#include	"pinio.h"

void power_on() {

	#ifdef	PS_ON_PIN
		WRITE(PS_ON_PIN, 0);
		SET_OUTPUT(PS_ON_PIN);
	#endif
}

void power_off() {

	stepper_disable();
	x_disable();
	y_disable();
	z_disable();
	e_disable();

	#ifdef	PS_ON_PIN
		SET_INPUT(PS_ON_PIN);
	#endif
}
