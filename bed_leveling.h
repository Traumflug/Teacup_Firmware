#ifndef	_BED_LEVELING_H
#define	_BED_LEVELING_H

#include "config_wrapper.h"

#include <stdint.h>

#include "dda.h"

#ifdef BED_LEVELING

// Clears all registered points and disables dynamic leveling
void bed_level_reset(void);

// Returns true if bed leveling a plane is mapped and leveling is active
int bed_leveling_active(void);

// Report information about bed leveling calculations
void bed_level_report(void);

// Read the z-adjustment for the given x,y position
int32_t bed_level_adjustment(int32_t x, int32_t y);

// Register a point as being "on the bed plane".  Three points are required
// to define a plane.  After three non-colinear points are registered, the
// adjustment is active and can be read from bed_level_adjustment.
// Note: units for x, y and z are um but the three (X, Y) points should be
// distinct enough in mm to define an accurate plane.
void bed_level_register(int32_t x, int32_t y, int32_t z);

#endif /* BED_LEVELING */

static int32_t bed_level_offset(const axes_int32_t) __attribute__ ((always_inline));
inline int32_t bed_level_offset(const axes_int32_t axis) {
  #ifdef BED_LEVELING
    return bed_level_adjustment(axis[X], axis[Y]);
  #else
    return 0;
  #endif
}

#endif
