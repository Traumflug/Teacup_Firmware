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
#include	"sermsg.h"

int main (void)
{
	uint8_t report;

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

	// set up clock
	clock_setup();

	// enable interrupts
	sei();

	// say hi to host
	serial_writestr_P(PSTR("Start\n"));

	// start queue
	//enableTimerInterrupt();

	// main loop
	for (;;)
	{
		if (serial_rxchars()) {
			uint8_t c = serial_popchar();
// 			TOGGLE(SCK);
			scan_char(c);
		}

// 		if (clock_flag_250ms & CLOCK_FLAG_250MS_TEMPCHECK) {
// 			clock_flag_250ms &= ~CLOCK_FLAG_250MS_TEMPCHECK;
// 			temp_tick();
// 		}

		if (clock_flag_250ms & CLOCK_FLAG_250MS_REPORT) {
			clock_flag_250ms &= ~CLOCK_FLAG_250MS_REPORT;
			report++;
			if (report == 4) {
				report = 0;

				// current move
				serial_writestr_P(PSTR("DDA: f#"));
				serwrite_int32(movebuffer[mb_head].f_counter);
				serial_writechar('/');
				serwrite_uint16(movebuffer[mb_head].f_scale);
				serial_writechar('/');
				serwrite_int16(movebuffer[mb_head].f_delta);
				serial_writechar('\n');

				// current position
				serial_writestr_P(PSTR("Pos: "));
				serwrite_int32(current_position.X);
				serial_writechar(',');
				serwrite_int32(current_position.Y);
				serial_writechar(',');
				serwrite_int32(current_position.Z);
				serial_writechar(',');
				serwrite_uint32(current_position.E);
				serial_writechar(',');
				serwrite_uint32(current_position.F);
				serial_writechar('\n');

				// target position
				serial_writestr_P(PSTR("Tar: "));
				serwrite_int32(movebuffer[mb_tail].endpoint.X);
				serial_writechar(',');
				serwrite_int32(movebuffer[mb_tail].endpoint.Y);
				serial_writechar(',');
				serwrite_int32(movebuffer[mb_tail].endpoint.Z);
				serial_writechar(',');
				serwrite_uint32(movebuffer[mb_tail].endpoint.E);
				serial_writechar(',');
				serwrite_uint32(movebuffer[mb_tail].endpoint.F);
				serial_writechar('\n');

				// Queue
				serial_writestr_P(PSTR("Q  : "));
// 				serwrite_uint8((mb_head - mb_tail) & (MOVEBUFFER_SIZE - 1));
				serwrite_uint8(mb_head);
				serial_writechar('/');
				serwrite_uint8(mb_tail);

				if (queue_full())
					serial_writechar('F');
				if (queue_empty())
					serial_writechar('E');
				serial_writechar('\n');
			}
		}
	}
}
