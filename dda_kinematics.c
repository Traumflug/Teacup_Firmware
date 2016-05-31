#include "dda_kinematics.h"

/** \file G-code axis system to stepper motor axis system conversion.
*/

#include <stdlib.h>

#include "dda_maths.h"


void
carthesian_to_carthesian(TARGET *startpoint, TARGET *target,
                         axes_uint32_t delta_um, axes_int32_t steps) {
  enum axis_e i;

  for (i = X; i < AXIS_COUNT; i++) {
    delta_um[i] = (uint32_t)labs(target->axis[i] - startpoint->axis[i]);
    steps[i] = um_to_steps(target->axis[i], i);
  }
}

void
carthesian_to_corexy(TARGET *startpoint, TARGET *target,
                     axes_uint32_t delta_um, axes_int32_t steps) {

  delta_um[X] = (uint32_t)labs((target->axis[X] - startpoint->axis[X]) +
                               (target->axis[Y] - startpoint->axis[Y]));
  delta_um[Y] = (uint32_t)labs((target->axis[X] - startpoint->axis[X]) -
                               (target->axis[Y] - startpoint->axis[Y]));
  delta_um[Z] = (uint32_t)labs(target->axis[Z] - startpoint->axis[Z]);
  delta_um[E] = (uint32_t)labs(target->axis[E] - startpoint->axis[E]);
  axes_um_to_steps_corexy(target->axis, steps);
}

void axes_um_to_steps_cartesian(const axes_int32_t um, axes_int32_t steps) {
  enum axis_e i;

  for (i = X; i < AXIS_COUNT; i++) {
    steps[i] = um_to_steps(um[i], i);
  }
}

void axes_um_to_steps_corexy(const axes_int32_t um, axes_int32_t steps) {
  steps[X] = um_to_steps(um[X] + um[Y], X);
  steps[Y] = um_to_steps(um[X] - um[Y], Y);
  steps[Z] = um_to_steps(um[Z], Z);
  steps[E] = um_to_steps(um[E], E);
}
