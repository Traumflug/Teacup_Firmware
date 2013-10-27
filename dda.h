#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"config.h"

#ifdef ACCELERATION_REPRAP
	#ifdef ACCELERATION_RAMPING
		#error Cant use ACCELERATION_REPRAP and ACCELERATION_RAMPING together.
	#endif
#endif

/*
	types
*/

// Enum to denote an axis
enum axis_e { X, Y, Z, E };

/**
	\struct TARGET
	\brief target is simply a point in space/time

	X, Y, Z and E are in micrometers unless explcitely stated. F is in mm/min.
*/
typedef struct {
// TODO TODO: We should really make up a loop for all axes.
//            Think of what happens when a sixth axis (multi colour extruder)
//            appears?
	int32_t						X;
	int32_t						Y;
	int32_t						Z;
	int32_t						E;
	uint32_t					F;

	uint8_t		e_relative				:1; ///< bool: e axis relative? Overrides all_relative
} TARGET;

/**
 \struct VECTOR4D
 \brief 4 dimensional vector used to describe the difference between moves.

  Units are in micrometers and usually based off 'TARGET'.
*/
typedef struct {
  int32_t X;
  int32_t Y;
  int32_t Z;
  int32_t E;
} VECTOR4D;

/**
	\struct MOVE_STATE
	\brief this struct is made for tracking the current state of the movement

	Parts of this struct are initialised only once per reboot, so make sure dda_step() leaves them with a value compatible to begin a new movement at the end of the movement. Other parts are filled in by dda_start().
*/
typedef struct {
	// bresenham counters
	int32_t						x_counter; ///< counter for total_steps vs this axis
	int32_t						y_counter; ///< counter for total_steps vs this axis
	int32_t						z_counter; ///< counter for total_steps vs this axis
	int32_t						e_counter; ///< counter for total_steps vs this axis

	// step counters
	uint32_t					x_steps; ///< number of steps on X axis
	uint32_t					y_steps; ///< number of steps on Y axis
	uint32_t					z_steps; ///< number of steps on Z axis
	uint32_t					e_steps; ///< number of steps on E axis

	#ifdef ACCELERATION_RAMPING
	/// counts actual steps done
	uint32_t					step_no;
	#endif
	#ifdef ACCELERATION_TEMPORAL
	uint32_t					x_time; ///< time of the last x step
	uint32_t					y_time; ///< time of the last y step
	uint32_t					z_time; ///< time of the last z step
	uint32_t					e_time; ///< time of the last e step
	uint32_t					all_time; ///< time of the last step of any axis
	#endif

	/// Endstop handling.
  uint8_t endstop_stop; ///< Stop due to endstop trigger
	uint8_t debounce_count_xmin, debounce_count_ymin, debounce_count_zmin;
	uint8_t debounce_count_xmax, debounce_count_ymax, debounce_count_zmax;
} MOVE_STATE;

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

	/// total number of steps: set to \f$\max(\Delta x, \Delta y, \Delta z, \Delta e)\f$
	uint32_t					total_steps;

	uint32_t					c; ///< time until next step, 24.8 fixed point

	#ifdef ACCELERATION_REPRAP
	uint32_t					end_c; ///< time between 2nd last step and last step
	#endif
	#ifdef ACCELERATION_RAMPING
  /// precalculated step time offset variable
  int32_t           n;
	/// number of steps accelerating
	uint32_t					rampup_steps;
	/// number of last step before decelerating
	uint32_t					rampdown_steps;
	/// 24.8 fixed point timer value, maximum speed
	uint32_t					c_min;
  #ifdef LOOKAHEAD
  // With the look-ahead functionality, it is possible to retain physical
  // movement between G1 moves. These variables keep track of the entry and
  // exit speeds between moves.
  uint32_t          F_start;
  uint32_t          start_steps; ///< steps to reach F_start
  uint32_t          F_end;
  // Displacement vector, in um, based between the difference of the starting
  // point and the target. Required to obtain the jerk between 2 moves.
  // Note: x_delta and co are in steps, not um.
  VECTOR4D          delta;
  // Number the moves to be able to test at the end of lookahead if the moves
  // are the same. Note: we do not need a lot of granularity here: more than
  // MOVEBUFFER_SIZE is already enough.
  uint8_t           id;
  #endif
	#endif
	#ifdef ACCELERATION_TEMPORAL
	uint32_t					x_step_interval; ///< time between steps on X axis
	uint32_t					y_step_interval; ///< time between steps on Y axis
	uint32_t					z_step_interval; ///< time between steps on Z axis
	uint32_t					e_step_interval; ///< time between steps on E axis
	uint8_t						axis_to_step;    ///< axis to be stepped on the next interrupt
	#endif

	/// Endstop homing
	uint8_t endstop_check; ///< Do we need to check endstops? 0x1=Check X, 0x2=Check Y, 0x4=Check Z
	uint8_t endstop_stop_cond; ///< Endstop condition on which to stop motion: 0=Stop on detrigger, 1=Stop on trigger
} DDA;

/*
	variables
*/

/// startpoint holds the endpoint of the most recently created DDA, so we know where the next one created starts. could also be called last_endpoint
extern TARGET startpoint;

/// the same as above, counted in motor steps
extern TARGET startpoint_steps;

/// current_position holds the machine's current position. this is only updated when we step, or when G92 (set home) is received.
extern TARGET current_position;

/*
	methods
*/

// initialize dda structures
void dda_init(void);

// distribute a new startpoint
void dda_new_startpoint(void);

// create a DDA
void dda_create(DDA *dda, TARGET *target, DDA *prev_dda);

// start a created DDA (called from timer interrupt)
void dda_start(DDA *dda)																						__attribute__ ((hot));

// DDA takes one step (called from timer interrupt)
void dda_step(DDA *dda)																							__attribute__ ((hot));

// regular movement maintenance
void dda_clock(void);

// update current_position
void update_current_position(void);

#endif	/* _DDA_H */
