#include	"dda_queue.h"

#include	"timer.h"
#include	"serial.h"
#include	"sermsg.h"

uint8_t	mb_head = 0;
uint8_t	mb_tail = 0;
DDA movebuffer[MOVEBUFFER_SIZE];

uint8_t queue_full() {
	if (mb_tail == 0)
		return mb_head == (MOVEBUFFER_SIZE - 1);
	else
		return mb_head == (mb_tail - 1);
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

void next_move() {
	if (queue_empty()) {
		disableTimerInterrupt();
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
