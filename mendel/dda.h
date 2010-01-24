#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"pinout.h"
#include	"machine.h"

typedef struct {
	int32_t						X;
	int32_t						Y;
	int32_t						Z;
	uint32_t					E;
	uint32_t					F;
} TARGET;

typedef struct {
// 	TARGET						currentpoint;
	TARGET						endpoint;

	uint8_t						x_direction		:1;
	uint8_t						y_direction		:1;
	uint8_t						z_direction		:1;
	uint8_t						e_direction		:1;
	uint8_t						f_direction		:1;
	uint8_t						nullmove			:1;
	uint8_t						live					:1;

	uint32_t					x_delta;
	uint32_t					y_delta;
	uint32_t					z_delta;
	uint32_t					e_delta;
	uint32_t					f_delta;

	int32_t						x_counter;
	int32_t						y_counter;
	int32_t						z_counter;
	int32_t						e_counter;
	int32_t						f_counter;

	uint32_t					total_steps;

	uint16_t					f_scale;
	uint32_t					move_duration;
} DDA;

extern uint8_t	mb_head;
extern uint8_t	mb_tail;
extern DDA movebuffer[MOVEBUFFER_SIZE];
extern TARGET startpoint;
extern TARGET current_position;

uint8_t queue_full(void);
uint8_t queue_empty(void);
void enqueue(TARGET *t);
void next_move(void);

uint32_t approx_distance( uint32_t dx, uint32_t dy );
uint32_t approx_distance_3( uint32_t dx, uint32_t dy, uint32_t dz );

void dda_create(TARGET *target, DDA *dda);
void dda_start(DDA *dda);
void dda_step(DDA *dda);

#endif	/* _DDA_H */
