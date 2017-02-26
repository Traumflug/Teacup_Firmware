#include	"dda_queue.h"

/** \file
	\brief DDA Queue - manage the move queue
*/

#include	<string.h>

#include	"config_wrapper.h"
#include	"timer.h"
#include	"serial.h"
#include	"temp.h"
#include	"delay.h"
#include	"sersendf.h"
#include	"clock.h"
#include "cpu.h"
#include	"memory_barrier.h"

/**
  Movebuffer head pointer. Points to the last move in the queue. This variable
  is used both in and out of interrupts, but is only written outside of
  interrupts.
*/
static uint_fast8_t mb_head = 0;

/// movebuffer tail pointer. Points to the currently executing move
/// this variable is read/written both in and out of interrupts.
uint_fast8_t	mb_tail = 0;

/// move buffer.
/// holds move queue
/// contents are read/written both in and out of interrupts, but
/// once writing starts in interrupts on a specific slot, the
/// slot will only be modified in interrupts until the slot is
/// is no longer live.
/// The size does not need to be a power of 2 anymore!
DDA BSS movebuffer[MOVEBUFFER_SIZE];

/**
  Pointer to the currently ongoing movement, or NULL, if there's no movement
  ongoing. Actually a cache of movebuffer[mb_tail].
*/
DDA *mb_tail_dda;

/// Find the next DDA index after 'x', where 0 <= x < MOVEBUFFER_SIZE
#define MB_NEXT(x) ((x) < MOVEBUFFER_SIZE - 1 ? (x) + 1 : 0)

/// check if the queue is completely full
uint_fast8_t queue_full() {
	MEMORY_BARRIER();
  return MB_NEXT(mb_head) == mb_tail;
}

// -------------------------------------------------------
// This is the one function called by the timer interrupt.
// It calls a few other functions, though.
// -------------------------------------------------------
/// Take a step or go to the next move.
void queue_step() {

  if (mb_tail_dda != NULL) {
    dda_step(mb_tail_dda);
	}

  /**
    Start the next move if this one is done and another one is available.

    This needs no atomic protection, because we're in an interrupt already.
  */
  if (mb_tail_dda == NULL || ! mb_tail_dda->live) {
    if (mb_tail != mb_head) {
      mb_tail = MB_NEXT(mb_tail);
      mb_tail_dda = &(movebuffer[mb_tail]);
      dda_start(mb_tail_dda);
    }
    else {
      mb_tail_dda = NULL;
    }
  }
}

/// add a move to the movebuffer
/// \note this function waits for space to be available if necessary, check queue_full() first if waiting is a problem
/// This is the only function that modifies mb_head and it always called from outside an interrupt.
void enqueue_home(TARGET *t, uint8_t endstop_check, uint8_t endstop_stop_cond) {
	// don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target
	while (queue_full())
		delay_us(100);

  uint_fast8_t h = MB_NEXT(mb_head);

	DDA* new_movebuffer = &(movebuffer[h]);

  // Initialise queue entry to a known state. This also clears flags like
  // dda->live, dda->done and dda->wait_for_temp.
  new_movebuffer->allflags = 0;

  new_movebuffer->endstop_check = endstop_check;
  new_movebuffer->endstop_stop_cond = endstop_stop_cond;
  dda_create(new_movebuffer, t);

  /**
    It's pointless to queue up movements which don't actually move the stepper,
    e.g. pure velocity changes or movements shorter than a single motor step.

    That said, accept movements which do move the steppers by forwarding
    mb_head. Also kick off movements if it's the first movement after a pause.
  */
  if ( ! new_movebuffer->nullmove) {
    // make certain all writes to global memory
    // are flushed before modifying mb_head.
    MEMORY_BARRIER();

    mb_head = h;

    if (mb_tail_dda == NULL) {
      /**
        Go to the next move.

        This is the version used from outside interrupts. The in-interrupt
        version is inlined (and simplified) in queue_step().
      */
      timer_reset();
      mb_tail = mb_head;  // Valid ONLY if the queue was empty before!
      mb_tail_dda = new_movebuffer; // Dito!
      dda_start(mb_tail_dda);
      // Compensate for the cli() in timer_set().
      sei();
    }
	}
}

/// DEBUG - print queue.
/// Qt/hs format, t is tail, h is head, s is F/full, E/empty or neither
void print_queue() {
  sersendf_P(PSTR("Queue: %d/%d%c\n"), mb_tail, mb_head,
             (queue_full() ? 'F' : (mb_tail_dda ? ' ' : 'E')));
}

/// dump queue for emergency stop.
/// Make sure to have all timers stopped with timer_stop() or
/// unexpected things might happen.
/// \todo effect on startpoint is undefined!
void queue_flush() {

  // if the timer were running, this would require
  // wrapping in ATOMIC_START ... ATOMIC_END.
  mb_tail = mb_head;
  mb_tail_dda = NULL;
}

/// wait for queue to empty
void queue_wait() {
  while (mb_tail_dda)
		clock();
}
