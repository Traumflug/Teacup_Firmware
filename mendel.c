/** \file
	\brief Main file - this is where it all starts, and ends
*/

/** \mainpage Teacup Reprap Firmware
	\section intro_sec Introduction
		Teacup Reprap Firmware (originally named FiveD on Arduino) is a firmware package for numerous reprap electronics sets.

		Please see README for a full introduction and long-winded waffle about this project
	\section install_sec	Installation
		\subsection step1 Step 1: Download
			\code git clone git://github.com/triffid/Teacup_Firmware \endcode
		\subsection step2 Step 2: configure
			\code cp config.[yourboardhere].h config.h \endcode
			Edit config.h to suit your machone
			Edit Makefile to select the correct chip and programming settings
		\subsection step3 Step 3: Compile
			\code make \endcode
			\code make program \endcode
		\subsection step4 Step 4: Test!
			\code ./func.sh mendel_reset
			./func.sh mendel_talk
			M115
			ctrl+d \endcode
*/

#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"config.h"
#include	"fuses.h"

#include	"serial.h"
#include	"dda_queue.h"
#include	"dda.h"
#include	"gcode_parse.h"
#include	"timer.h"
#include	"temp.h"
#include	"sermsg.h"
#include	"watchdog.h"
#include	"debug.h"
#include	"sersendf.h"
#include	"heater.h"
#include	"analog.h"
#include	"pinio.h"
#include	"arduino.h"
#include	"clock.h"
#include	"intercom.h"

/// initialise all I/O - set pins as input or output, turn off unused subsystems, etc
void io_init(void) {
	// disable modules we don't use
	#ifdef PRR
		PRR = MASK(PRTWI) | MASK(PRADC) | MASK(PRSPI);
	#elif defined PRR0
		PRR0 = MASK(PRTWI) | MASK(PRADC) | MASK(PRSPI);
		#if defined(PRUSART3)
			// don't use USART2 or USART3- leave USART1 for GEN3 and derivatives
			PRR1 |= MASK(PRUSART3) | MASK(PRUSART2);
		#endif
		#if defined(PRUSART2)
			// don't use USART2 or USART3- leave USART1 for GEN3 and derivatives
			PRR1 |= MASK(PRUSART2);
		#endif
	#endif
	ACSR = MASK(ACD);

	// setup I/O pins
	WRITE(X_STEP_PIN, 0);	SET_OUTPUT(X_STEP_PIN);
	WRITE(X_DIR_PIN,  0);	SET_OUTPUT(X_DIR_PIN);
	#ifdef X_MIN_PIN
		SET_INPUT(X_MIN_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(X_MIN_PIN, 1);
		#else
			WRITE(X_MIN_PIN, 0);
		#endif
	#endif
	#ifdef X_MAX_PIN
		SET_INPUT(X_MAX_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(X_MAX_PIN, 1);
		#else
			WRITE(X_MAX_PIN, 0);
		#endif
	#endif
	#ifdef X_ENABLE_PIN
		WRITE(X_ENABLE_PIN, 1); SET_OUTPUT(X_ENABLE_PIN);
	#endif

	WRITE(Y_STEP_PIN, 0);	SET_OUTPUT(Y_STEP_PIN);
	WRITE(Y_DIR_PIN,  0);	SET_OUTPUT(Y_DIR_PIN);
	#ifdef Y_MIN_PIN
		SET_INPUT(Y_MIN_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(Y_MIN_PIN, 1);
		#else
			WRITE(Y_MIN_PIN, 0);
		#endif
	#endif
	#ifdef Y_MAX_PIN
		SET_INPUT(Y_MAX_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(Y_MAX_PIN, 1);
		#else
			WRITE(Y_MAX_PIN, 0);
		#endif
	#endif
	#ifdef Y_ENABLE_PIN
		WRITE(Y_ENABLE_PIN, 1); SET_OUTPUT(Y_ENABLE_PIN);
	#endif

	#if defined Z_STEP_PIN && defined Z_DIR_PIN
		WRITE(Z_STEP_PIN, 0);	SET_OUTPUT(Z_STEP_PIN);
		WRITE(Z_DIR_PIN,  0);	SET_OUTPUT(Z_DIR_PIN);
	#endif
	#ifdef Z_MIN_PIN
		SET_INPUT(Z_MIN_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(Z_MIN_PIN, 1);
		#else
			WRITE(Z_MIN_PIN, 0);
		#endif
	#endif
	#ifdef Z_MAX_PIN
		SET_INPUT(Z_MAX_PIN);
		#ifdef USE_INTERNAL_PULLUPS
			WRITE(Z_MAX_PIN, 1);
		#else
			WRITE(Z_MAX_PIN, 0);
		#endif
	#endif
	#ifdef Z_ENABLE_PIN
		WRITE(Z_ENABLE_PIN, 1); SET_OUTPUT(Z_ENABLE_PIN);
	#endif

	#if defined E_STEP_PIN && defined E_DIR_PIN
		WRITE(E_STEP_PIN, 0);	SET_OUTPUT(E_STEP_PIN);
		WRITE(E_DIR_PIN,  0);	SET_OUTPUT(E_DIR_PIN);
	#endif
	#ifdef E_ENABLE_PIN
		WRITE(E_ENABLE_PIN, 1); SET_OUTPUT(E_ENABLE_PIN);
	#endif

	// setup PWM timers: fast PWM, no prescaler
	TCCR0A = MASK(WGM01) | MASK(WGM00);
	TCCR0B = MASK(CS00);
	TIMSK0 = 0;
	OCR0A = 0;
	OCR0B = 0;

	TCCR2A = MASK(WGM21) | MASK(WGM20);
	TCCR2B = MASK(CS20);
	TIMSK2 = 0;
	OCR2A = 0;
	OCR2B = 0;

	#ifdef	TCCR3A
		TCCR3A = MASK(WGM30);
		TCCR3B = MASK(WGM32) | MASK(CS30);
		TIMSK3 = 0;
		OCR3A = 0;
		OCR3B = 0;
	#endif

	#ifdef	TCCR4A
		TCCR4A = MASK(WGM40);
		TCCR4B = MASK(WGM42) | MASK(CS40);
		TIMSK4 = 0;
		OCR4A = 0;
		OCR4B = 0;
	#endif

	#ifdef	TCCR5A
		TCCR5A = MASK(WGM50);
		TCCR5B = MASK(WGM52) | MASK(CS50);
		TIMSK5 = 0;
		OCR5A = 0;
		OCR5B = 0;
	#endif

	#ifdef	STEPPER_ENABLE_PIN
		power_off();
	#endif

	// set all heater pins to output
	do {
		#undef	DEFINE_HEATER
		#define	DEFINE_HEATER(name, pin) WRITE(pin, 0); SET_OUTPUT(pin);
			#include "config.h"
		#undef DEFINE_HEATER
	} while (0);

	#ifdef	TEMP_MAX6675
		// setup SPI
		WRITE(SCK, 0);				SET_OUTPUT(SCK);
		WRITE(MOSI, 1);				SET_OUTPUT(MOSI);
		WRITE(MISO, 1);				SET_INPUT(MISO);
		WRITE(SS, 1);					SET_OUTPUT(SS);
	#endif

	#ifdef TEMP_INTERCOM
		// Enable the RS485 transceiver
		SET_OUTPUT(RX_ENABLE_PIN);
		SET_OUTPUT(TX_ENABLE_PIN);
		WRITE(RX_ENABLE_PIN,0);
		disable_transmit();
	#endif
}

/// Startup code, run when we come out of reset
void init(void) {
	// set up watchdog
	wd_init();

	// set up serial
	serial_init();

	// set up inputs and outputs
	io_init();

	// set up timers
	timer_init();

	// read PID settings from EEPROM
	heater_init();

	// set up default feedrate
	current_position.F = startpoint.F = next_target.target.F = SEARCH_FEEDRATE_Z;

	// start up analog read interrupt loop,
	// if any of the temp sensors in your config.h use analog interface
	analog_init();

	// set up temperature inputs
	temp_init();

	// enable interrupts
	sei();

	// reset watchdog
	wd_reset();

	// say hi to host
	serial_writestr_P(PSTR("start\nok\n"));

}

/// this is where it all starts, and ends
///
/// just run init(), then run an endless loop where we pass characters from the serial RX buffer to gcode_parse_char() and check the clocks
int main (void)
{
	init();

	// main loop
	for (;;)
	{
		// if queue is full, no point in reading chars- host will just have to wait
		if ((serial_rxchars() != 0) && (queue_full() == 0)) {
			uint8_t c = serial_popchar();
			gcode_parse_char(c);
		}

		ifclock(clock_flag_10ms) {
			clock_10ms();
		}
	}
}
