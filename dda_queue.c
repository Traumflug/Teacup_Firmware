#include	"dda_queue.h"

#include	<string.h>
#include	<avr/interrupt.h>

#include	"machine.h" // for XONXOFF
#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"temp.h"

uint8_t	mb_head = 0;
uint8_t	mb_tail = 0;
DDA movebuffer[MOVEBUFFER_SIZE] __attribute__ ((__section__ (".bss")));

uint8_t queue_full() {
	return (((mb_tail - mb_head - 1) & (MOVEBUFFER_SIZE - 1)) == 0)?255:0;
}

uint8_t queue_empty() {
	return ((mb_tail == mb_head) && (movebuffer[mb_tail].live == 0))?255:0;
}

// -------------------------------------------------------
// This is the one function called by the timer interrupt.
// It calls a few other functions, though.
// -------------------------------------------------------
void queue_step() {
	disableTimerInterrupt();

	// do our next step
	// NOTE: dda_step makes this interrupt interruptible after steps have been sent but before new speed is calculated.
	if (movebuffer[mb_tail].live) {
		if (movebuffer[mb_tail].waitfor_temp) {
			if (temp_achieved()) {
				movebuffer[mb_tail].live = movebuffer[mb_tail].waitfor_temp = 0;
				serial_writestr_P(PSTR("Temp achieved\n"));
			}

			#if STEP_INTERRUPT_INTERRUPTIBLE
				sei();
			#endif
		}
		else {
			dda_step(&(movebuffer[mb_tail]));
		}
	}

// 	serial_writechar('!');

	// fall directly into dda_start instead of waiting for another step
	if (movebuffer[mb_tail].live == 0)
		next_move();

	#if STEP_INTERRUPT_INTERRUPTIBLE
		cli();
	#endif
	// check queue, if empty we don't need to interrupt again until re-enabled in dda_create
	if (queue_empty() == 0)
		enableTimerInterrupt();
}

void enqueue(TARGET *t) {
	// don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target
	while (queue_full())
		delay(WAITING_DELAY);

	uint8_t h = mb_head;
	h++;
	if (h == MOVEBUFFER_SIZE)
		h = 0;

	dda_create(&movebuffer[h], t);

	mb_head = h;

	#ifdef	XONXOFF
	// If the queue has only two slots remaining, stop transmission. More
	// characters might come in until the stop takes effect.
	if (((mb_tail - mb_head - 1) & (MOVEBUFFER_SIZE - 1)) < (MOVEBUFFER_SIZE - 2))
		xoff();
	#endif

	// fire up in case we're not running yet
	enableTimerInterrupt();
}

void enqueue_temp_wait() {
	// don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target
	while (queue_full())
		delay(WAITING_DELAY);

	uint8_t h = mb_head + 1;
// 	h++;
// 	if (h == MOVEBUFFER_SIZE)
// 		h = 0;
	h &= (MOVEBUFFER_SIZE - 1);

	// wait for temp flag
	movebuffer[h].waitfor_temp = 1;
	movebuffer[h].nullmove = 0;
	#if (F_CPU & 0xFF000000) == 0
		// set "step" timeout to 1 second
		movebuffer[h].c = F_CPU << 8;
	#else
		// set "step" timeout to maximum
		movebuffer[h].c = 0xFFFFFF00;
	#endif

	mb_head = h;

	#ifdef	XONXOFF
	if (((mb_tail - mb_head - 1) & (MOVEBUFFER_SIZE - 1)) < (MOVEBUFFER_SIZE - 2))
		xoff();
	#endif

	// fire up in case we're not running yet
	enableTimerInterrupt();
}

void next_move() {
	if (queue_empty() == 0) {
		// next item
		uint8_t t = mb_tail + 1;
// 		t++;
// 		if (t == MOVEBUFFER_SIZE)
// 			t = 0;
		t &= (MOVEBUFFER_SIZE - 1);
		dda_start(&movebuffer[t]);
		mb_tail = t;
	}

	#ifdef	XONXOFF
	// restart transmission
	xon();
	#endif
}

void print_queue() {
	serial_writechar('Q');
	serwrite_uint8(mb_tail);
	serial_writechar('/');
	serwrite_uint8(mb_head);
	if (queue_full())
		serial_writechar('F');
	if (queue_empty())
		serial_writechar('E');
	serial_writechar('\n');
}
