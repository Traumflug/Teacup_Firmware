#include	"pinio.h"
#include	"delay.h"

static char ps_is_on = 0;

/// step/psu timeout
volatile uint8_t	psu_timeout = 0;

void power_on() {

	if (ps_is_on == 0) {
		#ifdef	PS_ON_PIN
			WRITE(PS_ON_PIN, 0);
			SET_OUTPUT(PS_ON_PIN);
			_delay_ms(500);
		#endif
		ps_is_on = 1;
	}

	psu_timeout = 0;
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

	ps_is_on = 0;
}
