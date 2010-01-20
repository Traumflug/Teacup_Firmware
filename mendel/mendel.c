#include	<stddef.h>
#include	<stdint.h>
#include	<string.h>

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"machine.h"

#include	"serial.h"
#include	"dda.h"
#include	"gcode.h"
#include	"timer.h"
#include	"clock.h"
#include	"temp.h"

int main (void)
{
	// set up serial
	serial_init();

	// set up inputs and outputs
	WRITE(X_STEP_PIN, 0);	SET_OUTPUT(X_STEP_PIN);
	WRITE(X_DIR_PIN,  0);	SET_OUTPUT(X_DIR_PIN);
	WRITE(X_MIN_PIN,  1);	SET_INPUT(X_MIN_PIN);

	WRITE(Y_STEP_PIN, 0);	SET_OUTPUT(Y_STEP_PIN);
	WRITE(Y_DIR_PIN,  0);	SET_OUTPUT(Y_DIR_PIN);
	WRITE(Y_MIN_PIN,  1);	SET_INPUT(Y_MIN_PIN);

	WRITE(Z_STEP_PIN, 0);	SET_OUTPUT(Z_STEP_PIN);
	WRITE(Z_DIR_PIN,  0);	SET_OUTPUT(Z_DIR_PIN);
	WRITE(Z_MIN_PIN,  1);	SET_INPUT(Z_MIN_PIN);

	WRITE(E_STEP_PIN, 0);	SET_OUTPUT(E_STEP_PIN);
	WRITE(E_DIR_PIN,  0);	SET_OUTPUT(E_DIR_PIN);

	WRITE(HEATER_PIN, 0);	SET_OUTPUT(HEATER_PIN);

	#ifdef	FAN_PIN
	WRITE(FAN_PIN, 0); SET_OUTPUT(FAN_PIN);
	#endif

	WRITE(SCK, 0);	SET_OUTPUT(SCK);
	WRITE(MISO, 1);	SET_INPUT(MISO);
	WRITE(SS, 0);		SET_OUTPUT(SS);


	// set up timers
	setupTimerInterrupt();

	// enable interrupts
	sei();

	serial_writeblock((uint8_t *) "Start\n", 6);

	// main loop
	for (;;)
	{
		if (serial_rxchars()) {
			uint8_t c = serial_popchar();
			scan_char(c);
		}

		if (clock_flag_250ms & CLOCK_FLAG_250MS_TEMPCHECK) {
			clock_flag_250ms &= ~CLOCK_FLAG_250MS_TEMPCHECK;
			temp_tick();
		}
	}
}
