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
#if ! defined MAX_JERK_X || ! defined MAX_JERK_Y || \
    ! defined MAX_JERK_Z || ! defined MAX_JERK_E
#error Your config.h does not specify one of MAX_JERK_X,
#error MAX_JERK_Y, MAX_JERK_Z or MAX_JERK_E while LOOKAHEAD is enabled!
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

void dda_find_crossing_speed(DDA *prev, DDA *current, uint32_t curr_distance);
void dda_join_moves(DDA *prev, DDA *current);

// Debug counters
extern uint32_t lookahead_joined;
extern uint32_t lookahead_timeout;

#endif /* LOOKAHEAD */
#endif /* DDA_LOOKAHEAD_H_ */
