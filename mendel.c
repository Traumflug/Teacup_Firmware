/** \file
	\brief Main file - this is where it all starts, and ends
*/

/** \mainpage Teacup Reprap Firmware
	\section intro_sec Introduction
		Teacup Reprap Firmware (originally named FiveD on Arduino) is a firmware package for numerous reprap electronics sets.

		Please see README for a full introduction and long-winded waffle about this project
	\section install_sec	Installation
		\subsection step1 Step 1: Download
			\code git clone git://github.com/traumflug/Teacup_Firmware \endcode
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

#ifdef __AVR__
#include	<avr/interrupt.h>
#endif

#include	"config_wrapper.h"
#include "cpu.h"
#include	"serial.h"
#include	"dda_queue.h"
#include	"gcode_parse.h"
#include	"timer.h"
#include	"temp.h"
#include	"watchdog.h"
#include	"debug.h"
#include	"heater.h"
#include	"analog.h"
#include	"pinio.h"
#include	"clock.h"
#include	"intercom.h"
#include "spi.h"
#include "sd.h"
#include "simulator.h"

#ifdef SIMINFO
  #include "../simulavr/src/simulavr_info.h"
  SIMINFO_DEVICE(MCU_STR);
  SIMINFO_CPUFREQUENCY(F_CPU);
  #ifdef BAUD
    SIMINFO_SERIAL_IN("D0", "-", BAUD);
    SIMINFO_SERIAL_OUT("D1", "-", BAUD);
  #endif
#endif

#ifdef CANNED_CYCLE
  const char PROGMEM canned_gcode_P[] = CANNED_CYCLE;
#endif

/** Initialise all the subsystems.

  Note that order of appearance is critical here. For example, running
  spi_init() before io_init() makes SPI fail (for reasons not exactly
  investigated).
*/
void init(void) {

  cpu_init();

	// set up watchdog
	wd_init();

	// set up serial
	serial_init();

	// set up G-code parsing
	gcode_init();

	// set up inputs and outputs
  pinio_init();

  #ifdef SPI
    spi_init();
  #endif

	// set up timers
	timer_init();

	heater_init();

	// set up dda
	dda_init();

	// start up analog read interrupt loop,
	// if any of the temp sensors in your config.h use analog interface
	analog_init();

	// set up temperature inputs
	temp_init();

  #ifdef SD
    sd_init();
  #endif

	// enable interrupts
	sei();

	// reset watchdog
	wd_reset();

  // prepare the power supply
  power_init();

	// say hi to host
	serial_writestr_P(PSTR("start\nok\n"));

}

/// this is where it all starts, and ends
///
/// just run init(), then run an endless loop where we pass characters from the serial RX buffer to gcode_parse_char() and check the clocks
#ifdef SIMULATOR
int main (int argc, char** argv)
{
  sim_start(argc, argv);
#else
int main (void)
{
#endif
  uint8_t c, line_done, ack_waiting = 0;

	init();

	// main loop
	for (;;)
	{
		// if queue is full, no point in reading chars- host will just have to wait
    if (queue_full() == 0) {
      /**
        Postpone sending acknowledgement until there's a free slot in the
        movement queue. This way the host waits with sending the next line
        until it can be processed immediately. As a result, the serial receive
        queue is always almost empty; it exists only for receiving via XON/XOFF
        flow control. Another result is, the incoming line can be longer than
        the receiving buffer, see Github issue #52.

        At the time of the introduction of this strategy gcode_parse_char()
        parsed a single character in 100 to 400 CPU clocks, processing
        the line end took some 30'000 clocks. 115200 baud mean one character
        incoming every about 1250 CPU clocks on AVR 16 MHz.
      */
      if (ack_waiting) {
        serial_writestr_P(PSTR("ok\n"));
        ack_waiting = 0;
      }

      if (( ! gcode_active || gcode_active & GCODE_SOURCE_SERIAL) &&
          serial_rxchars() != 0) {
        gcode_active = GCODE_SOURCE_SERIAL;
        c = serial_popchar();
        line_done = gcode_parse_char(c);
        if (line_done) {
          gcode_active = 0;
          ack_waiting = 1;
        }
      }

      #ifdef SD
        if (( ! gcode_active || gcode_active & GCODE_SOURCE_SD) &&
            gcode_sources & GCODE_SOURCE_SD) {
          if (sd_read_gcode_line()) {
            serial_writestr_P(PSTR("\nSD file done.\n"));
            gcode_sources &= ! GCODE_SOURCE_SD;
            // There is no pf_close(), subsequent reads will stick at EOF
            // and return zeros.
          }
        }
      #endif

      #ifdef CANNED_CYCLE
        /**
          WARNING!

          This code works on a per-character basis.

          Unlike with SD reading code above and for historical reasons (was
          a quicky doing its job, before SD card was implemented), any data
          received over serial WILL be randomly distributed through the canned
          G-code, and you'll have a big mess!

          The solution is to join the strategy above and make canned G-code
          a third G-code source next to serial and SD.
        */
        static uint32_t canned_gcode_pos = 0;

        gcode_parse_char(pgm_read_byte(&(canned_gcode_P[canned_gcode_pos])));

        canned_gcode_pos++;
        if (pgm_read_byte(&(canned_gcode_P[canned_gcode_pos])) == 0)
          canned_gcode_pos = 0;

      #endif /* CANNED_CYCLE */
		}

		clock();
	}
}
