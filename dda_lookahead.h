#ifndef DDA_LOOKAHEAD_H_
#define DDA_LOOKAHEAD_H_

#include <stdint.h>
#include "config.h"
#include "dda.h"

#ifndef ACCELERATION_RAMPING
// Only enable the lookahead bits if ramping acceleration is enabled
#undef LOOKAHEAD
#endif

#ifdef LOOKAHEAD

// Sanity: make sure the defines are in place
#ifndef LOOKAHEAD_MAX_JERK_XY
#error Your config.h does not specify LOOKAHEAD_MAX_JERK_XY while LOOKAHEAD is enabled!
#endif
#ifndef LOOKAHEAD_MAX_JERK_E
#error Your config.h does not specify LOOKAHEAD_MAX_JERK_E while LOOKAHEAD is enabled!
#endif

// Sanity: the acceleration of Teacup is not implemented properly; as such we can only
// do move joining when all axis use the same steps per mm. This is usually not an issue
// for X and Y.
#if STEPS_PER_M_X != STEPS_PER_M_Y
#error "Look-ahead requires steps per m to be identical on the X and Y axis (for now)"
#endif

// This is the same to ACCELERATE_RAMP_LEN but now the steps per m can be switched.
// Note: use this with a macro so the float is removed by the preprocessor
#define ACCELERATE_RAMP_SCALER(spm) (uint32_t)((7200000.0f * ACCELERATION) / (float)spm)
#define ACCELERATE_RAMP_LEN2(speed, scaler) (((speed)*(speed)) / (scaler))

// Pre-calculated factors to determine ramp lengths for all axis
#define ACCELERATE_SCALER_X ACCELERATE_RAMP_SCALER(STEPS_PER_M_X)
#define ACCELERATE_SCALER_Y ACCELERATE_RAMP_SCALER(STEPS_PER_M_Y)
#define ACCELERATE_SCALER_Z ACCELERATE_RAMP_SCALER(STEPS_PER_M_Z)
#define ACCELERATE_SCALER_E ACCELERATE_RAMP_SCALER(STEPS_PER_M_E)

// To have a oneliner to fetch the correct scaler (pass the enum axis_e here)
#define ACCELERATE_SCALER(axis) ((axis==X)?ACCELERATE_SCALER_X:((axis==Y)?ACCELERATE_SCALER_Y:((axis==Z)?ACCELERATE_SCALER_Z:ACCELERATE_SCALER_E)))

#define MAX(a,b)  (((a)>(b))?(a):(b))
#define MIN(a,b)  (((a)<(b))?(a):(b))

/**
 * Join 2 moves by removing the full stop between them, where possible.
 * To join the moves, the expected jerk - or force - of the change in direction is calculated.
 * The jerk is used to scale the common feed rate between both moves to obtain an acceptable speed
 * to transition between 'prev' and 'current'.
 *
 * Premise: we currently join the last move in the queue and the one before it (if any).
 * This means the feed rate at the end of the 'current' move is 0.
 *
 * Premise: the 'current' move is not dispatched in the queue: it should remain constant while this
 * function is running.
 *
 * Note: the planner always makes sure the movement can be stopped within the
 * last move (= 'current'); as a result a lot of small moves will still limit the speed.
 */
void dda_join_moves(DDA *prev, DDA *current);

// Debug counters
extern uint32_t lookahead_joined;
extern uint32_t lookahead_timeout;

#endif /* LOOKAHEAD */
#endif /* DDA_LOOKAHEAD_H_ */
