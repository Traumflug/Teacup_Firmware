#ifndef	_DDA_H
#define	_DDA_H

#include	<stdint.h>

#include	"config.h"

/*
	micrometer to steps conversion

	handle a few cases to avoid overflow while keeping reasonable accuracy
	input is up to 20 bits, so we can multiply by 4096 at most
*/
#if	STEPS_PER_M_X >= 4096000
	#define	um_to_steps_x(dest, src) \
		do { dest = (src * (STEPS_PER_M_X / 10000L) + 50L) / 100L; } while (0)
#elif	STEPS_PER_M_X >= 409600
	#define	um_to_steps_x(dest, src) \
		do { dest = (src * (STEPS_PER_M_X / 1000L) + 500L) / 1000L; } while (0)
#elif	STEPS_PER_M_X >= 40960
	#define	um_to_steps_x(dest, src) \
		do { dest = (src * (STEPS_PER_M_X / 100L) + 5000L) / 10000L; } while (0)
#elif	STEPS_PER_M_X >= 4096
	#define	um_to_steps_x(dest, src) \
		do { dest = (src * (STEPS_PER_M_X / 10L) + 50000L) / 100000L; } while (0)
#else
	#define	um_to_steps_x(dest, src) \
		do { dest = (src * (STEPS_PER_M_X / 1L) + 500000L) / 1000000L; } while (0)
#endif

#if	STEPS_PER_M_Y >= 4096000
	#define	um_to_steps_y(dest, src) \
		do { dest = (src * (STEPS_PER_M_Y / 10000L) + 50L) / 100L; } while (0)
#elif	STEPS_PER_M_Y >= 409600
	#define	um_to_steps_y(dest, src) \
		do { dest = (src * (STEPS_PER_M_Y / 1000L) + 500L) / 1000L; } while (0)
#elif	STEPS_PER_M_Y >= 40960
	#define	um_to_steps_y(dest, src) \
		do { dest = (src * (STEPS_PER_M_Y / 100L) + 5000L) / 10000L; } while (0)
#elif	STEPS_PER_M_Y >= 4096
	#define	um_to_steps_y(dest, src) \
		do { dest = (src * (STEPS_PER_M_Y / 10L) + 50000L) / 100000L; } while (0)
#else
	#define	um_to_steps_y(dest, src) \
		do { dest = (src * (STEPS_PER_M_Y / 1L) + 500000L) / 1000000L; } while (0)
#endif

#if	STEPS_PER_M_Z >= 4096000
	#define	um_to_steps_z(dest, src) \
		do { dest = (src * (STEPS_PER_M_Z / 10000L) + 50L) / 100L; } while (0)
#elif	STEPS_PER_M_Z >= 409600
	#define	um_to_steps_z(dest, src) \
		do { dest = (src * (STEPS_PER_M_Z / 1000L) + 500L) / 1000L; } while (0)
#elif	STEPS_PER_M_Z >= 40960
	#define	um_to_steps_z(dest, src) \
		do { dest = (src * (STEPS_PER_M_Z / 100L) + 5000L) / 10000L; } while (0)
#elif	STEPS_PER_M_Z >= 4096
	#define	um_to_steps_z(dest, src) \
		do { dest = (src * (STEPS_PER_M_Z / 10L) + 50000L) / 100000L; } while (0)
#else
	#define	um_to_steps_z(dest, src) \
		do { dest = (src * (STEPS_PER_M_Z / 1L) + 500000L) / 1000000L; } while (0)
#endif

#if	STEPS_PER_M_E >= 4096000
	#define	um_to_steps_e(dest, src) \
		do { dest = (src * (STEPS_PER_M_E / 10000L) + 50L) / 100L; } while (0)
#elif	STEPS_PER_M_E >= 409600
	#define	um_to_steps_e(dest, src) \
		do { dest = (src * (STEPS_PER_M_E / 1000L) + 500L) / 1000L; } while (0)
#elif	STEPS_PER_M_E >= 40960
	#define	um_to_steps_e(dest, src) \
		do { dest = (src * (STEPS_PER_M_E / 100L) + 5000L) / 10000L; } while (0)
#elif	STEPS_PER_M_E >= 4096
	#define	um_to_steps_e(dest, src) \
		do { dest = (src * (STEPS_PER_M_E / 10L) + 50000L) / 100000L; } while (0)
#else
	#define	um_to_steps_e(dest, src) \
		do { dest = (src * (STEPS_PER_M_E / 1L) + 500000L) / 1000000L; } while (0)
#endif


#ifdef ACCELERATION_REPRAP
	#ifdef ACCELERATION_RAMPING
		#error Cant use ACCELERATION_REPRAP and ACCELERATION_RAMPING together.
	#endif
#endif

/*
	types
*/

/**
	\struct TARGET
	\brief target is simply a point in space/time

	X, Y, Z and E are in micrometers unless explcitely stated. F is in mm/min.
*/
typedef struct {
	int32_t						X;
	int32_t						Y;
	int32_t						Z;
	int32_t						E;
	uint32_t					F;
} TARGET;

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
	/// time until next step
	uint32_t					c;
	/// tracking variable
	int32_t						n;
	#endif
	#ifdef ACCELERATION_TEMPORAL
	uint32_t					x_time; ///< time of the last x step
	uint32_t					y_time; ///< time of the last y step
	uint32_t					z_time; ///< time of the last z step
	uint32_t					e_time; ///< time of the last e step
	uint32_t					all_time; ///< time of the last step of any axis
	#endif

	/// Endstop debouncing
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
	int32_t						n; ///< precalculated step time offset variable. At every step we calculate \f$c = c - (2 c / n)\f$; \f$n+=4\f$. See http://www.embedded.com/columns/technicalinsights/56800129?printable=true for full description
	#endif
	#ifdef ACCELERATION_RAMPING
	/// number of steps accelerating
	uint32_t					rampup_steps;
	/// number of last step before decelerating
	uint32_t					rampdown_steps;
	/// 24.8 fixed point timer value, maximum speed
	uint32_t					c_min;
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
 * Dynamic config
 */
typedef struct {
	struct {
		// Operational params
		uint8_t					e_absolute	:1; ///< bool: E axis is specified in absolute/relative position
	};
} CONFIG ;

/*
	variables
*/

/// steptimeout is set to zero when we step, and increases over time so we can turn the motors off when they've been idle for a while
/// It is also used inside and outside of interrupts, which is why it has been made volatile
extern volatile uint8_t steptimeout;

/// startpoint holds the endpoint of the most recently created DDA, so we know where the next one created starts. could also be called last_endpoint
extern TARGET startpoint;

/// the same as above, counted in motor steps
extern TARGET startpoint_steps;

/// current_position holds the machine's current position. this is only updated when we step, or when G92 (set home) is received.
extern TARGET current_position;

/// holds dynamic config settings
extern CONFIG config;

/*
	methods
*/

uint32_t approx_distance( uint32_t dx, uint32_t dy )								__attribute__ ((hot));
uint32_t approx_distance_3( uint32_t dx, uint32_t dy, uint32_t dz )	__attribute__ ((hot));

// const because return value is always the same given the same v
const uint8_t	msbloc (uint32_t v)																		__attribute__ ((const));

// initialize dda structures
void dda_init(void);

// distribute a new startpoint
void dda_new_startpoint(void);

// create a DDA
void dda_create(DDA *dda, TARGET *target);

// start a created DDA (called from timer interrupt)
void dda_start(DDA *dda)																						__attribute__ ((hot));

// DDA takes one step (called from timer interrupt)
void dda_step(DDA *dda)																							__attribute__ ((hot));

// update current_position
void update_current_position(void);

#endif	/* _DDA_H */
