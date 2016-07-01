#ifndef _DDA_KINEMATICS_H
#define _DDA_KINEMATICS_H

#include "config_wrapper.h"

#include <stdint.h>

#include "dda.h"


void carthesian_to_carthesian(const TARGET *startpoint, const TARGET *target,
                              axes_uint32_t delta_um, axes_int32_t steps);

void carthesian_to_corexy(const TARGET *startpoint, const TARGET *target,
                          axes_uint32_t delta_um, axes_int32_t steps);

//void carthesian_to_scara(TARGET *startpoint, TARGET *target,
//                         axes_uint32_t delta_um, axes_int32_t steps);

static void code_axes_to_stepper_axes(const TARGET *, const TARGET *, axes_uint32_t,
                                      axes_int32_t)
                                      __attribute__ ((always_inline));
inline void code_axes_to_stepper_axes(const TARGET *startpoint, const TARGET *target,
                                      axes_uint32_t delta_um,
                                      axes_int32_t steps) {
  #if defined KINEMATICS_STRAIGHT
    carthesian_to_carthesian(startpoint, target, delta_um, steps);
  #elif defined KINEMATICS_COREXY
    carthesian_to_corexy(startpoint, target, delta_um, steps);
//  #elif defined KINEMATICS_SCARA
//    return carthesian_to_scara(startpoint, target, delta_um, steps);
  #else
    #error KINEMATICS not defined or unknown, edit your config.h.
  #endif
}

void axes_um_to_steps_cartesian(const axes_int32_t um, axes_int32_t steps);
void axes_um_to_steps_corexy(const axes_int32_t um, axes_int32_t steps);
// void axes_um_to_steps_scara(const axes_int32_t um, axes_int32_t steps);

static void axes_um_to_steps(const axes_int32_t, axes_int32_t)
                                      __attribute__ ((always_inline));
inline void axes_um_to_steps(const axes_int32_t um, axes_int32_t steps) {
  #if defined KINEMATICS_STRAIGHT
    axes_um_to_steps_cartesian(um, steps);
  #elif defined KINEMATICS_COREXY
    axes_um_to_steps_corexy(um, steps);
//  #elif defined KINEMATICS_SCARA
//    axes_um_to_steps_scara(um, steps);
  #else
    #error KINEMATICS not defined or unknown, edit your config.h.
  #endif
}

#endif /* _DDA_KINEMATICS_H */
