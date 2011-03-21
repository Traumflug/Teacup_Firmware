#include	"dda_queue.h"

/** \file
	\brief DDA Queue - manage the move queue
*/

#include	<string.h>
#include	<avr/interrupt.h>

#include	"config.h"
#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"temp.h"
#include	"delay.h"
#include	"sersendf.h"
#include	"clock.h"

/// movebuffer head pointer. Points to the last move in the queue.
uint8_t	mb_head = 0;

/// movebuffer tail pointer. Points to the currently executing move
uint8_t	mb_tail = 0;

/// move buffer.
/// holds move queue
DDA movebuffer[MOVEBUFFER_SIZE] __attribute__ ((__section__ (".bss")));

/// check if the queue is completely full
uint8_t queue_full() {
	return (((mb_tail - mb_head - 1) & (MOVEBUFFER_SIZE - 1)) == 0)?255:0;
}

/// check if the queue is completely empty
uint8_t queue_empty() {
	return ((mb_tail == mb_head) && (movebuffer[mb_tail].live == 0))?255:0;
}

// -------------------------------------------------------
// This is the one function called by the timer interrupt.
// It calls a few other functions, though.
// -------------------------------------------------------
/// Take a step or go to the next move.
void queue_step() {
	// do our next step
	if (movebuffer[mb_tail].live) {
		if (movebuffer[mb_tail].waitfor_temp) {
			setTimer(movebuffer[mb_tail].c >> 8);
			if (temp_achieved()) {
				movebuffer[mb_tail].live = movebuffer[mb_tail].waitfor_temp = 0;
				serial_writestr_P(PSTR("Temp achieved\n"));
			}

			#if STEP_INTERRUPT_INTERRUPTIBLE
				sei();
			#endif
		}
		else {
			// NOTE: dda_step makes this interrupt interruptible after steps have been sent but before new speed is calculated.
			dda_step(&(movebuffer[mb_tail]));
		}
	}

	// fall directly into dda_start instead of waiting for another step
	// the dda dies not directly after its last step, but when the timer fires and there's no steps to do
	if (movebuffer[mb_tail].live == 0)
		next_move();
}

/// add a move to the movebuffer
/// \note this function waits for space to be available if necessary, check queue_full() first if waiting is a problem
void enqueue(TARGET *t) {
	// don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target
	while (queue_full())
		delay(WAITING_DELAY);

	uint8_t h = mb_head + 1;
	h &= (MOVEBUFFER_SIZE - 1);

	if (t != NULL) {
		dda_create(&movebuffer[h], t);
	}
	else {
		// it's a wait for temp
		movebuffer[h].waitfor_temp = 1;
		movebuffer[h].nullmove = 0;
		#if (F_CPU & 0xFF000000) == 0
			// set "step" timeout to 1 second
			movebuffer[h].c = F_CPU << 8;
		#else
			// set "step" timeout to maximum
			movebuffer[h].c = 0xFFFFFF00;
		#endif
	}

	mb_head = h;

	// fire up in case we're not running yet
	if (movebuffer[mb_tail].live == 0)
		next_move();
}

/// go to the next move.
/// be aware that this is sometimes called from interrupt context, sometimes not.
void next_move() {
	if (queue_empty() == 0) {
		do {
			// next item
			uint8_t t = mb_tail + 1;
			t &= (MOVEBUFFER_SIZE - 1);
			if (movebuffer[t].waitfor_temp) {
				#ifndef	REPRAP_HOST_COMPATIBILITY
					serial_writestr_P(PSTR("Waiting for target temp\n"));
				#endif
				movebuffer[t].live = 1;
				setTimer(movebuffer[t].c >> 8);
			}
			else {
				dda_start(&movebuffer[t]);
			}
			mb_tail = t;
		} while ((queue_empty() == 0) && (movebuffer[mb_tail].live == 0));
	}
	else
		setTimer(0);
}

/// DEBUG - print queue.
/// Qt/hs format, t is tail, h is head, s is F/full, E/empty or neither
void print_queue() {
	sersendf_P(PSTR("Q%d/%d%c"), mb_tail, mb_head, (queue_full()?'F':(queue_empty()?'E':' ')));
}

/// dump queue for emergency stop.
/// \todo effect on startpoint/current_position is undefined!
void queue_flush() {
	// save interrupt flag
	uint8_t sreg = SREG;

	// disable interrupts
	cli();

	// flush queue
	mb_tail = mb_head;
	movebuffer[mb_head].live = 0;

	// restore interrupt flag
	SREG = sreg;
}

/// waits for a space in the queue to become available
void queue_wait() {
	for (;queue_empty() == 0;) {
		ifclock(CLOCK_FLAG_10MS) {
			clock_10ms();
		}
	}
}
