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
#include	<avr/io.h>
#include	<avr/interrupt.h>
#endif

#ifndef __ARMEL_NOTYET__
#include	"config_wrapper.h"
#include	"fuses.h"
#endif /* __ARMEL_NOTYET__ */

#include "cpu.h"
#include	"serial.h"
#ifndef __ARMEL_NOTYET__
#include	"dda_queue.h"
#include	"dda.h"
#include	"gcode_parse.h"
#include "gcode_process.h"
#include	"timer.h"
#include	"temp.h"
#include	"watchdog.h"
#include	"debug.h"
#include	"heater.h"
#include	"analog.h"
#endif /* __ARMEL_NOTYET__ */
#include	"pinio.h"
#ifndef __ARMEL_NOTYET__
#include	"arduino.h"
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
#endif /* __ARMEL_NOTYET__ */

/** Initialise all the subsystems.

  Note that order of appearance is critical here. For example, running
  spi_init() before io_init() makes SPI fail (for reasons not exactly
  investigated).
*/
void init(void) {

  cpu_init();

  #ifndef __ARMEL_NOTYET__
	// set up watchdog
	wd_init();
  #endif /* __ARMEL_NOTYET__ */

	// set up serial
	serial_init();

  #ifndef __ARMEL_NOTYET__
	// set up G-code parsing
	gcode_init();

	// set up inputs and outputs
  pinio_init();

  #if defined TEMP_MAX6675 || defined SD
    spi_init();
  #endif

	// set up timers
	timer_init();

	// read PID settings from EEPROM
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
  #endif /* __ARMEL_NOTYET__ */

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
	init();

	// main loop
	for (;;)
	{
    #ifndef __ARMEL_NOTYET__
		// if queue is full, no point in reading chars- host will just have to wait
    if (queue_full() == 0) {
      uint8_t c, line_done;

      if (( ! gcode_active || gcode_active & GCODE_SOURCE_SERIAL) &&
          serial_rxchars() != 0) {
        gcode_active = GCODE_SOURCE_SERIAL;
        c = serial_popchar();
        line_done = gcode_parse_char(c);
        if (line_done)
          gcode_active = 0;
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
    #endif /* __ARMEL_NOTYET__ */
	}
}
