#include	"clock.h"

/** \file
	\brief Do stuff periodically
*/

#include <stdint.h>
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

/**
  If the specific bit is set, execute the following block exactly once
  and then clear the flag.
*/
#define ifclock(F) for ( ; F; F = 0)


/**
  Every time our clock fires we increment this,
  so we know when 10ms/250ms/1s has elapsed.
*/
static uint8_t clock_counter_10ms = 0;
static uint8_t clock_counter_250ms = 0;
static uint8_t clock_counter_1s = 0;

/**
  Flags to tell clock() when above have elapsed.
*/
static volatile uint8_t clock_flag_10ms = 0;
static volatile uint8_t clock_flag_250ms = 0;
static volatile uint8_t clock_flag_1s = 0;


/** Advance our clock by a tick.

  Update clock counters accordingly. Should be called from the TICK_TIME
  Interrupt in timer.c.
*/
void clock_tick(void) {
  clock_counter_10ms += TICK_TIME_MS;
  if (clock_counter_10ms >= 10) {
    clock_counter_10ms -= 10;
    clock_flag_10ms = 1;

    clock_counter_250ms++;
    if (clock_counter_250ms >= 25) {
      clock_counter_250ms = 0;
      clock_flag_250ms = 1;

      clock_counter_1s++;
      if (clock_counter_1s >= 4) {
        clock_counter_1s = 0;
        clock_flag_1s = 1;
      }
    }
  }
}

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
      sersendf_P(PSTR("Pos: %lq,%lq,%lq,%lq,%lu\n"), current_position.axis[X], current_position.axis[Y], current_position.axis[Z], current_position.axis[E], current_position.F);

			// target position
      sersendf_P(PSTR("Dst: %lq,%lq,%lq,%lq,%lu\n"), movebuffer[mb_tail].endpoint.axis[X], movebuffer[mb_tail].endpoint.axis[Y], movebuffer[mb_tail].endpoint.axis[Z], movebuffer[mb_tail].endpoint.axis[E], movebuffer[mb_tail].endpoint.F);

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

	temp_sensor_tick();

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
#ifdef SIMULATOR
  sim_time_warp();
#endif
}


