#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"target.h"
#include	"pinout.h"
#include	"gcode.h"
#include	"machine.h"

typedef struct {
	TARGET						currentpoint;
	TARGET						endpoint;

	uint8_t						x_direction		:1;
	uint8_t						y_direction		:1;
	uint8_t						z_direction		:1;
	uint8_t						e_direction		:1;
	uint8_t						f_direction		:1;
	uint8_t						nullmove			:1;
	uint8_t						live					:1;

	int16_t						x_delta;
	int16_t						y_delta;
	int16_t						z_delta;
	int16_t						e_delta;
	int16_t						f_delta;

	int32_t						x_counter;
	int32_t						y_counter;
	int32_t						z_counter;
	int32_t						e_counter;
	int32_t						f_counter;

	uint32_t					total_steps;

	uint16_t					f_scale;
	uint32_t					distance;
	uint32_t					move_duration;
} DDA;

extern uint8_t	mb_head;
extern uint8_t	mb_tail;
extern DDA movebuffer[MOVEBUFFER_SIZE];

uint32_t approx_distance( int32_t dx, int32_t dy );

void dda_create(GCODE_COMMAND *cmd, DDA *dda);
void dda_start(DDA *dda);
void dda_step(DDA *dda);

#endif	/* _DDA_H */
