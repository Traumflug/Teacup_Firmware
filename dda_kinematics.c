#include "dda_kinematics.h"

/** \file G-code axis system to stepper motor axis system conversion.
*/

#include <stdlib.h>

#include "dda_maths.h"
#include "bed_leveling.h"

void
carthesian_to_carthesian(const TARGET *startpoint, const TARGET *target,
                         axes_uint32_t delta_um, axes_int32_t steps) {
  delta_um[X] = (uint32_t)labs(target->axis[X] - startpoint->axis[X]);
  delta_um[Y] = (uint32_t)labs(target->axis[Y] - startpoint->axis[Y]);
  delta_um[Z] = (uint32_t)labs(target->axis[Z] - startpoint->axis[Z]);

  axes_um_to_steps_cartesian(target->axis, steps);
}

void
carthesian_to_corexy(const TARGET *startpoint, const TARGET *target,
                     axes_uint32_t delta_um, axes_int32_t steps) {

  delta_um[X] = (uint32_t)labs((target->axis[X] - startpoint->axis[X]) +
                               (target->axis[Y] - startpoint->axis[Y]));
  delta_um[Y] = (uint32_t)labs((target->axis[X] - startpoint->axis[X]) -
                               (target->axis[Y] - startpoint->axis[Y]));
  delta_um[Z] = (uint32_t)labs(target->axis[Z] - startpoint->axis[Z]);
  axes_um_to_steps_corexy(target->axis, steps);
}

void axes_um_to_steps_cartesian(const axes_int32_t um, axes_int32_t steps) {
  steps[X] = um_to_steps(um[X], X);
  steps[Y] = um_to_steps(um[Y], Y);
  steps[Z] = um_to_steps(um[Z] + bed_level_offset(um), Z);
}

void axes_um_to_steps_corexy(const axes_int32_t um, axes_int32_t steps) {
  steps[X] = um_to_steps(um[X] + um[Y], X);
  steps[Y] = um_to_steps(um[X] - um[Y], Y);
  steps[Z] = um_to_steps(um[Z] + bed_level_offset(um), Z);
}

void delta_to_axes_cartesian(axes_int32_t delta) {
  // nothing to do for cartesian
}

void delta_to_axes_corexy(axes_int32_t delta) {
  // recalculate only dedicated axes
  int32_t x_axis, y_axis;
  x_axis = (delta[X] + delta[Y]) / 2;
  y_axis = (delta[X] - delta[Y]) / 2;
  delta[X] = x_axis;
  delta[Y] = y_axis;
}
