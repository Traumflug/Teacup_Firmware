#include	"dda_queue.h"

#include	<string.h>

#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"

uint8_t	mb_head = 0;
uint8_t	mb_tail = 0;
DDA movebuffer[MOVEBUFFER_SIZE] __attribute__ ((__section__ (".bss")));

uint8_t queue_full() {
	return (((mb_tail - mb_head - 1) & (MOVEBUFFER_SIZE - 1)) == 0)?255:0;
}

uint8_t queue_empty() {
	return ((mb_tail == mb_head) && (movebuffer[mb_tail].live == 0))?255:0;
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
		// if queue is full, stop transmition
		if (queue_full())
			xoff();
	#endif

	// fire up in case we're not running yet
	enableTimerInterrupt();
}

void enqueue_temp_wait() {
	// don't call this function when the queue is full, but just in case, wait for a move to complete and free up the space for the passed target
	while (queue_full())
		delay(WAITING_DELAY);

	uint8_t h = mb_head;
	h++;
	if (h == MOVEBUFFER_SIZE)
		h = 0;

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
		// if queue is full, stop transmition
		if (queue_full())
			xoff();
	#endif

	// fire up in case we're not running yet
	enableTimerInterrupt();
}

void next_move() {
	if (queue_empty()) {
// 		memcpy(&startpoint, &current_position, sizeof(TARGET));
		startpoint.E = current_position.E = 0;
	}
	else {
		// next item
		uint8_t t = mb_tail;
		t++;
		if (t == MOVEBUFFER_SIZE)
			t = 0;
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
