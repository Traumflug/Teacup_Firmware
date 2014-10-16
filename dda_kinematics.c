#include "dda_kinematics.h"

/** \file G-code axis system to stepper motor axis system conversion.
*/

#include <stdlib.h>

#include "dda_maths.h"


void
carthesian_to_carthesian(TARGET *startpoint, TARGET *target,
                         axes_uint32_t delta_um, axes_int32_t steps) {
  enum axis_e i;

  for (i = X; i < E; i++) {
    delta_um[i] = (uint32_t)labs(target->axis[i] - startpoint->axis[i]);
    steps[i] = um_to_steps(target->axis[i], i);
  }
}

void
carthesian_to_corexy(TARGET *startpoint, TARGET *target,
                     axes_uint32_t delta_um, axes_int32_t steps) {

  delta_um[X] = (uint32_t)labs((target->axis[X] - startpoint->axis[X]) +
                               (target->axis[Y] - startpoint->axis[Y]));
  steps[X] = um_to_steps(target->axis[X] + target->axis[Y], X);

  delta_um[Y] = (uint32_t)labs((target->axis[X] - startpoint->axis[X]) -
                               (target->axis[Y] - startpoint->axis[Y]));
  steps[Y] = um_to_steps(target->axis[X] - target->axis[Y], Y);

  delta_um[Z] = (uint32_t)labs(target->axis[Z] - startpoint->axis[Z]);
  steps[Z] = um_to_steps(target->axis[Z], Z);
}
