#include	"clock.h"

/** \file
	\brief Do stuff periodically
*/

#include	"pinio.h"
#include	"sersendf.h"
#include	"dda_queue.h"
#include	"watchdog.h"
#include	"timer.h"
#include	"debug.h"
#include	"heater.h"
#include	"serial.h"
#ifdef	TEMP_INTERCOM
	#include	"intercom.h"
#endif
#include	"memory_barrier.h"

/*!	do stuff every 1/4 second

	called from clock_10ms(), do not call directly
*/
static void clock_250ms(void) {
  if (heaters_all_zero()) {
		if (psu_timeout > (30 * 4)) {
			power_off();
		}
		else {
      ATOMIC_START
        psu_timeout++;
      ATOMIC_END
		}
	}

	ifclock(clock_flag_1s) {
		if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
			// current position
			update_current_position();
			sersendf_P(PSTR("Pos: %lq,%lq,%lq,%lq,%lu\n"), current_position.X, current_position.Y, current_position.Z, current_position.E, current_position.F);

			// target position
			sersendf_P(PSTR("Dst: %lq,%lq,%lq,%lq,%lu\n"), movebuffer[mb_tail].endpoint.X, movebuffer[mb_tail].endpoint.Y, movebuffer[mb_tail].endpoint.Z, movebuffer[mb_tail].endpoint.E, movebuffer[mb_tail].endpoint.F);

			// Queue
			print_queue();

			// newline
			serial_writechar('\n');
		}
		// temperature
		/*		if (temp_get_target())
		temp_print();*/
	}
	#ifdef	TEMP_INTERCOM
	start_send();
	#endif
}

/*! do stuff every 10 milliseconds

	called from clock(), do not call directly
*/
static void clock_10ms(void) {
	// reset watchdog
	wd_reset();

	temp_tick();

	ifclock(clock_flag_250ms) {
		clock_250ms();
	}
}

/*! do reoccuring stuff

	call it occasionally in busy loops
*/
void clock() {
	ifclock(clock_flag_10ms) {
		clock_10ms();
	}
}


