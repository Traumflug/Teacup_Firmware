#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"config_wrapper.h"

#ifdef ACCELERATION_REPRAP
	#ifdef ACCELERATION_RAMPING
		#error Cant use ACCELERATION_REPRAP and ACCELERATION_RAMPING together.
	#endif
#endif


// Enum to denote an axis
enum axis_e { X = 0, Y, Z, E, AXIS_COUNT };

/**
  \typedef axes_uint32_t
  \brief n-dimensional vector used to describe uint32_t axis information.

  Stored value can be anything unsigned. Units should be specified when declared.
*/
typedef uint32_t axes_uint32_t[AXIS_COUNT];

/**
  \typedef axes_int32_t
  \brief n-dimensional vector used to describe int32_t axis information.

  Stored value can be anything unsigned. Units should be specified when declared.
*/
typedef int32_t axes_int32_t[AXIS_COUNT];

// Period over which we calculate new velocity; should be at least TICK_TIME/2
#define QUANTUM (TICK_TIME*2)

// Acceleration per QUANTUM.
// Normalized to q8.24; allows up to 2^8=256 in mantissa (steps per quantum)
#define ACCEL_P_SHIFT 24

extern const axes_uint32_t PROGMEM accel_P ;

/**
	\struct TARGET
	\brief target is simply a point in space/time

	X, Y, Z and E are in micrometers unless explcitely stated. F is in mm/min.
*/
typedef struct {
  axes_int32_t axis;
  uint32_t  F;

  uint16_t  e_multiplier;
  uint16_t  f_multiplier;
  uint8_t   e_relative        :1; ///< bool: e axis relative? Overrides all_relative
} TARGET;

/**
	\struct MOVE_STATE
	\brief this struct is made for tracking the current state of the movement

	Parts of this struct are initialised only once per reboot, so make sure dda_step() leaves them with a value compatible to begin a new movement at the end of the movement. Other parts are filled in by dda_start().
*/
typedef struct {
	// bresenham counters
  axes_int32_t      counter; ///< counter for total_steps vs each axis

  #if ! defined ACCELERATION_TEMPORAL
    uint32_t          position;     // Calculated fast-axis position planned
    uint32_t          velocity;     // fast axis velocity updated on each dda_clock call
    uint32_t          remainder;    // calculated fractional position between dda_clock intervals

    uint32_t          accel_per_tick;     // fast axis acceleration per TICK_TIME, 8.24 fixed point
    #define SUB_MOVE_QUEUE_SIZE 4
    uint32_t          curr_c;                         // Speed at end of current queue
    int32_t           next_dc[SUB_MOVE_QUEUE_SIZE];   // delta speed of next steps
    uint32_t          next_n[SUB_MOVE_QUEUE_SIZE];    // Number of steps in the next movement; 0 when taken by dda
    uint8_t           head;                           // Index of next movement queue
    uint8_t           tail;                           // Index of last movement queue
    uint8_t           accel    :1 ;                   // bool: accel or decel/cruise
	/// counts actual steps done
	uint32_t					step_no;
	#else
  axes_uint32_t     time;       ///< time of the last step on each axis
  uint32_t          last_time;  ///< time of the last step of any axis
	#endif

  #if defined ACCELERATION_TEMPORAL || defined ACCELERATION_RAMPING
  axes_uint32_t   steps;      ///< number of steps on each axis
  #endif

	/// Endstop handling.
  uint8_t endstop_stop; ///< Stop due to endstop trigger
  uint8_t debounce_count_x, debounce_count_y, debounce_count_z;
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
			volatile uint8_t  live          :1; ///< bool: this DDA is running and still has steps to do
      uint8_t           done          :1; ///< bool: this DDA is done.
			#ifdef ACCELERATION_REPRAP
			uint8_t						accel					:1; ///< bool: speed changes during this move, run accel code
			#endif

			// directions
      // As we have muldiv() now, overflows became much less an issue and
      // it's likely time to get rid of these flags and use int instead of
      // uint for distance/speed calculations. --Traumflug 2014-07-04
			uint8_t						x_direction		:1; ///< direction flag for X axis
			uint8_t						y_direction		:1; ///< direction flag for Y axis
			uint8_t						z_direction		:1; ///< direction flag for Z axis
			uint8_t						e_direction		:1; ///< direction flag for E axis
		};
    uint16_t            allflags; ///< used for clearing all flags
	};

	// distances
  axes_uint32_t     delta;       ///< number of steps on each axis

  // uint8_t        fast_axis;   (see below)
  uint32_t          total_steps; ///< steps of the "fast" axis
  uint32_t          fast_um;     ///< movement length of this fast axis

	uint32_t					c; ///< time until next step, 24.8 fixed point


	#ifdef ACCELERATION_REPRAP
	uint32_t					end_c; ///< time between 2nd last step and last step
	int32_t						n;     ///< precalculated step time offset variable
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
  /// Maximum velocity in steps/QUANTUM; 8.24 fixed point value
  uint32_t          vmax;
  #ifdef LOOKAHEAD
  // With the look-ahead functionality, it is possible to retain physical
  // movement between G1 moves. These variables keep track of the entry and
  // exit speeds between moves.
  uint32_t          distance;
  uint32_t          crossF;
  // These two are based on the "fast" axis, the axis with the most steps.
  uint32_t          start_steps; ///< would be required to reach start feedrate
  uint32_t          end_steps; ///< would be required to stop from end feedrate

  // /// Start velocity in steps/QUANTUM; 8.24 fixed point value
  // uint32_t          start_v;
  // /// End velocity in steps/QUANTUM; 8.24 fixed point value
  // uint32_t          end_v;
  /// Max safe start velocity respecting Jerk (calculated once between this and prev move)
  uint32_t          v_jerk;
  /// Max start velocity which still allows me to decelerate to v_end (calc between this and next move)
  /// During move, this is set to MIN(v_limit, v_jerk) so we have just one value to consider
  uint32_t          v_start;
  /// Max end velocity which matches v_start of next move, but for our fast-axis instead of theirs
  uint32_t          v_end;
  // v_limit = sqrt(2*accel_per_tick*total_steps + v_end * v_end)

  //uint32_t          v_start = MIN(v_limit, v_jerk);
  //uint32_t          v_end=next->v_start, or 0 if no next
  // Calc v_end in dda_start
  // Recalc v_start when v_limit changes
  // Recalc v_limit when next->v_start changes
  // Strategy: Calculate deceleration from v_limit for every move.  When f(v_limit, t) < velocity, begin deceleration to match.
  //   Problem: calculating v_limit involves sqrt.  We don't need perfect accuracy, but slow (sqrt) and must eat extra steps (accuracy)
  //   Alternatives:
  //       1. Calculate rampdown_steps(v_start-v_end) and iteratively cut speed {in half, by accel_per_tick} until steps <= total_steps
  //       2. Calc A=steps_per_quantum(v_end), B=steps_per_quantum(v_jerk), C=(v_end-v_jerk)/accel;
  //               if (A+B)*C/2 < total_steps,then use v_jerk
  //               else calculate B' which satisfies inequality and use that to determine speed for v_limit
  //               ** Not accurate for small A values **
  //       3. Keep track of accel_steps = "accel steps to reach current velocity"
  //          if v_start < v_end, then { record accel_steps when velocity==v_end; begin decel when accel_steps-accel_steps(v_end) == total_steps-step_no
  //          else if accel_steps <= total_steps, then { ??? }
  //       4. Pre-calculate steps required to reach several velocities and use these granular velocities as our cross-v places
  #endif
  // Number the moves to be able to test at the end of lookahead if the moves
  // are the same. Note: we do not need a lot of granularity here: more than
  // MOVEBUFFER_SIZE is already enough.
  uint8_t           id;
	#endif
	#ifdef ACCELERATION_TEMPORAL
  axes_uint32_t     step_interval;   ///< time between steps on each axis
	uint8_t						axis_to_step;    ///< axis to be stepped on the next interrupt
	#endif

  /// Small variables. Many CPUs can access 32-bit variables at word or double
  /// word boundaries only and fill smaller variables in between with gaps,
  /// so keep small variables grouped together to reduce the amount of these
  /// gaps. See e.g. NXP application note AN10963, page 10f.
  uint8_t           fast_axis;       ///< number of the fast axis

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

int8_t get_direction(DDA *dda, enum axis_e n);

// initialize dda structures
void dda_init(void);

// distribute a new startpoint
void dda_new_startpoint(void);

// create a DDA
void dda_create(DDA *dda, const TARGET *target);

// start a created DDA (called from timer interrupt)
void dda_start(DDA *dda);

// DDA takes one step (called from timer interrupt)
void dda_step(DDA *dda);

// regular movement maintenance
void dda_clock(void);

// update current_position
void update_current_position(void);

#endif	/* _DDA_H */
