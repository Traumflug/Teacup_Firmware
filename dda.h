#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"pinout.h"
#include	"machine.h"

/*
	types
*/

// target is simply a point in space/time
typedef struct {
	int32_t						X;
	int32_t						Y;
	int32_t						Z;
	int32_t						E;
	uint32_t					F;
} TARGET;

// this is a digital differential analyser data struct
typedef struct {
	// this is where we should finish
	TARGET						endpoint;

	union {
		struct {
			// status fields
			uint8_t						nullmove			:1;
			uint8_t						live					:1;
			uint8_t						accel					:1;

			// wait for temperature to stabilise flag
			uint8_t						waitfor_temp	:1;

			// directions
			uint8_t						x_direction		:1;
			uint8_t						y_direction		:1;
			uint8_t						z_direction		:1;
			uint8_t						e_direction		:1;
		};
		uint8_t							allflags;	// used for clearing all flags
	};

	// distances
	uint32_t					x_delta;
	uint32_t					y_delta;
	uint32_t					z_delta;
	uint32_t					e_delta;

	// bresenham counters
	int32_t						x_counter;
	int32_t						y_counter;
	int32_t						z_counter;
	int32_t						e_counter;

	// total number of steps: set to max(x_delta, y_delta, z_delta, e_delta)
	uint32_t					total_steps;

	// linear acceleration variables: c and end_c are 24.8 fixed point timer values, n is the tracking variable
	uint32_t					c;
	uint32_t					end_c;
	int32_t						n;
} DDA;

/*
	variables
*/

// steptimeout is set to zero when we step, and increases over time so we can turn the motors off when they've been idle for a while
extern uint8_t steptimeout;

// startpoint holds the endpoint of the most recently created DDA, so we know where the next one created starts
// could also be called last_endpoint
extern TARGET startpoint;

// current_position holds the machine's current position. this is only updated when we step, or when G92 (set home) is received.
extern TARGET current_position;

/*
	methods
*/

uint32_t approx_distance( uint32_t dx, uint32_t dy )								__attribute__ ((hot));
uint32_t approx_distance_3( uint32_t dx, uint32_t dy, uint32_t dz )	__attribute__ ((hot));

// const because return value is always the same given the same v
const uint8_t	msbloc (uint32_t v)																		__attribute__ ((const));

// create a DDA
void dda_create(DDA *dda, TARGET *target);

// start a created DDA (called from timer interrupt)
void dda_start(DDA *dda)																						__attribute__ ((hot));

// DDA takes one step (called from timer interrupt)
void dda_step(DDA *dda)																							__attribute__ ((hot));

#endif	/* _DDA_H */
