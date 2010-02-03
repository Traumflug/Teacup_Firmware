#ifndef	_DDA_QUEUE
#define	_DDA_QUEUE

#include	"dda.h"

/*
	variables
*/

extern uint8_t	mb_head;
extern uint8_t	mb_tail;
extern DDA movebuffer[MOVEBUFFER_SIZE];

/*
	methods
*/

uint8_t queue_full(void);
uint8_t queue_empty(void);
void enqueue(TARGET *t);
void next_move(void);
void print_queue(void);

#endif	/* _DDA_QUEUE */
