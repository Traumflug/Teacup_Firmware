#ifndef DDA_LOOKAHEAD_H_
#define DDA_LOOKAHEAD_H_

#include <stdint.h>
#include "config_wrapper.h"
#include "dda.h"
#include "debug.h"

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

#define MAX(a,b)  (((a)>(b))?(a):(b))
#define MIN(a,b)  (((a)<(b))?(a):(b))

void dda_find_crossing_speed(DDA *prev, DDA *current);
void dda_join_moves(DDA *prev, DDA *current);

#endif /* LOOKAHEAD */
#endif /* DDA_LOOKAHEAD_H_ */
