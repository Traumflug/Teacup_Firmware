#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"config.h"

// Used in distance calculation during DDA setup
/// micrometers per step X
#define	UM_PER_STEP_X		1000L / ((uint32_t) STEPS_PER_MM_X)
/// micrometers per step Y
#define	UM_PER_STEP_Y		1000L / ((uint32_t) STEPS_PER_MM_Y)
/// micrometers per step Z
#define	UM_PER_STEP_Z		1000L / ((uint32_t) STEPS_PER_MM_Z)
/// micrometers per step E
#define	UM_PER_STEP_E		1000L / ((uint32_t) STEPS_PER_MM_E)

#ifdef ACCELERATION_REPRAP
	#ifdef ACCELERATION_RAMPING
		#error Cant use ACCELERATION_REPRAP and ACCELERATION_RAMPING together.
	#endif
#endif

/*
	enums
*/
// wether we accelerate, run at full speed, break down, etc.
typedef enum {
	RAMP_UP,
	RAMP_MAX,
	RAMP_DOWN
} ramp_state_t;

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

/**
	\struct DDA
	\brief this is a digital differential analyser data struct

	This struct holds all the details of an individual multi-axis move, including pre-calculated acceleration data.
	This struct is filled in by dda_create(), called from enqueue(), called mostly from gcode_process() and from a few other places too (eg \file homing.c)
*/
typedef struct {
	/// this is where we should finish
	TARGET						endpoint;

	union {
		struct {
			// status fields
			uint8_t						nullmove			:1; ///< bool: no axes move, maybe we wait for temperatures or change speed
			uint8_t						live					:1; ///< bool: this DDA is running and still has steps to do
			#ifdef ACCELERATION_REPRAP
			uint8_t						accel					:1; ///< bool: speed changes during this move, run accel code
			#endif

			// wait for temperature to stabilise flag
			uint8_t						waitfor_temp	:1; ///< bool: wait for temperatures to reach their set values

			// directions
			uint8_t						x_direction		:1; ///< direction flag for X axis
			uint8_t						y_direction		:1; ///< direction flag for Y axis
			uint8_t						z_direction		:1; ///< direction flag for Z axis
			uint8_t						e_direction		:1; ///< direction flag for E axis
		};
		uint8_t							allflags;	///< used for clearing all flags
	};

	// distances
	uint32_t					x_delta; ///< number of steps on X axis
	uint32_t					y_delta; ///< number of steps on Y axis
	uint32_t					z_delta; ///< number of steps on Z axis
	uint32_t					e_delta; ///< number of steps on E axis

	// bresenham counters
	int32_t						x_counter; ///< counter for total_steps vs this axis, used for bresenham calculations.
	int32_t						y_counter; ///< counter for total_steps vs this axis, used for bresenham calculations.
	int32_t						z_counter; ///< counter for total_steps vs this axis, used for bresenham calculations.
	int32_t						e_counter; ///< counter for total_steps vs this axis, used for bresenham calculations.

	// step counters
	uint32_t					x_steps; ///< number of steps on X axis
	uint32_t					y_steps; ///< number of steps on Y axis
	uint32_t					z_steps; ///< number of steps on Z axis
	uint32_t					e_steps; ///< number of steps on E axis

	/// total number of steps: set to \f$\max(\Delta x, \Delta y, \Delta z, \Delta e)\f$
	uint32_t					total_steps;

	// linear acceleration variables: c and end_c are 24.8 fixed point timer values, n is the tracking variable
	uint32_t					c; ///< time until next step
	#ifdef ACCELERATION_REPRAP
	uint32_t					end_c; ///< time between 2nd last step and last step
	int32_t						n; ///< precalculated step time offset variable. At every step we calculate \f$c = c - (2 c / n)\f$; \f$n+=4\f$. See http://www.embedded.com/columns/technicalinsights/56800129?printable=true for full description
	#endif
	#ifdef ACCELERATION_RAMPING
	/// start of down-ramp, intitalized with total_steps / 2
	uint32_t					ramp_steps;
	/// counts actual steps done
	uint32_t					step_no;
	/// 24.8 fixed point timer value, maximum speed
	uint32_t					c_min;
	/// tracking variable
	int32_t						n;
	/// keep track of whether we're ramping up, down, or plateauing
	ramp_state_t			ramp_state;
	#endif
} DDA;

/*
	variables
*/

/// steptimeout is set to zero when we step, and increases over time so we can turn the motors off when they've been idle for a while
/// It is also used inside and outside of interrupts, which is why it has been made volatile
extern volatile uint8_t steptimeout;

/// startpoint holds the endpoint of the most recently created DDA, so we know where the next one created starts. could also be called last_endpoint
extern TARGET startpoint;

/// current_position holds the machine's current position. this is only updated when we step, or when G92 (set home) is received.
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

// update current_position
void update_position(void);

#endif	/* _DDA_H */
