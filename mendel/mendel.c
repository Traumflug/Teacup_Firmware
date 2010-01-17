#include	<stddef.h>
#include	<stdint.h>
#include	<string.h>

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"serial.h"

#include	"dda.h"
#include	"gcode.h"
#include	"timer.h"

#include	"machine.h"

uint8_t	option_bitfield;

struct {
	volatile int32_t	X;
	volatile int32_t	Y;
	volatile int32_t	Z;
	volatile int32_t	E;
	volatile int32_t	F;
} current_position = { 0, 0, 0, 0, 0 };

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

	// set up timers
	setupTimerInterrupt();

	// enable interrupts
	sei();

	for (;;)
	{
		for (;serial_rxchars() == 0;);
		uint8_t c = serial_popchar();

		scan_char(c);
	}
}
