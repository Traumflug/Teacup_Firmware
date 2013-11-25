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
			delay_ms(500);
		#endif
    #ifdef PS_MOSFET_PIN
      WRITE(PS_MOSFET_PIN, 1);
      delay_ms(10);
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

  #ifdef PS_MOSFET_PIN
    WRITE(PS_MOSFET_PIN, 0);
  #endif

	ps_is_on = 0;
}

// step the 'n' axis
void do_step(enum axis_e n) {
  if (n == X)
    x_step();
  else if (n == Y)
    y_step();
  else if (n == Z)
    z_step();
  else if (n == E)
    e_step();
}

