#include "dda_kinematics.h"

/** \file G-code axis system to stepper motor axis system conversion.
*/

#include <stdlib.h>

#include "dda_maths.h"
#include "sersendf.h"
#include "debug.h"


void
carthesian_to_carthesian(const TARGET *startpoint, const TARGET *target,
                         axes_uint32_t delta_um, axes_int32_t steps) {
  enum axis_e i;

  for (i = X; i < E; i++) {
    delta_um[i] = (uint32_t)labs(target->axis[i] - startpoint->axis[i]);
    steps[i] = um_to_steps(target->axis[i], i);
  }

  /* Replacing the above five lines with this costs about 200 bytes binary
     size on AVR, but also takes about 120 clock cycles less during movement
     preparation. The smaller version was kept for our Arduino Nano friends.
  delta_um[X] = (uint32_t)labs(target->axis[X] - startpoint->axis[X]);
  steps[X] = um_to_steps(target->axis[X], X);
  delta_um[Y] = (uint32_t)labs(target->axis[Y] - startpoint->axis[Y]);
  steps[Y] = um_to_steps(target->axis[Y], Y);
  delta_um[Z] = (uint32_t)labs(target->axis[Z] - startpoint->axis[Z]);
  steps[Z] = um_to_steps(target->axis[Z], Z);
  */
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

#ifdef DELTA_PRINTER
void
carthesian_to_delta(TARGET *startpoint, TARGET *target,
                    axes_uint32_t delta_um, axes_int32_t steps) {

  enum axis_e i;
  TARGET delkin_start,delkin_target;

  if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA)){
    sersendf_P(PSTR("Kin Before: Start(%ld,%ld,%ld,E:%ld F:%ld ER:%u) Target(%ld,%ld,%ld,E:%ld F:%ld ER:%u) \n"),
                startpoint->axis[X], startpoint->axis[Y], startpoint->axis[Z], startpoint->axis[E],startpoint->F,startpoint->e_relative,
                target->axis[X], target->axis[Y], target->axis[Z], target->axis[E],target->F,target->e_relative);
  }

//  sersendf_P(PSTR("Kin Before: Tower1(%ld,%ld), Tower2(%ld,%ld), Tower3(%ld,%ld) \n"),
//                delta_tower1_x,delta_tower1_y,delta_tower2_x,delta_tower2_y,delta_tower3_x,delta_tower3_y);

  if (bypass_delta==0)
  {
    delkin_start  = delta_from_cartesian(startpoint);
    delkin_target = delta_from_cartesian(target);

    for (i = X; i < E; i++){
      delta_um[i] = (uint32_t)(labs(delkin_target.axis[i] - delkin_start.axis[i]));
      steps[i]    = um_to_steps(delkin_target.axis[i], i);
    }

    if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA)){
      sersendf_P(PSTR("Kin After : d_Start(%ld,%ld,%ld,E:%ld F:%ld ER:%u) d_Target(%ld,%ld,%ld,E:%ld F:%ld ER:%u) \n"),
                   delkin_start.axis[X], delkin_start.axis[Y], delkin_start.axis[Z],delkin_start.axis[E],delkin_start.F,delkin_start.e_relative,
                   delkin_target.axis[X], delkin_target.axis[Y], delkin_target.axis[Z],delkin_target.axis[E],delkin_target.F,delkin_target.e_relative);
      sersendf_P(PSTR("Kin After : Delta_um(%ld,%ld,%ld) Steps(%ld,%ld,%ld) \n"),
                   delta_um[X], delta_um[Y], delta_um[Z],
                   steps[X], steps[Y], steps[Z]);
    }
  } else {
    carthesian_to_carthesian(startpoint, target,delta_um,steps);
    if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA))
      sersendf_P(PSTR("Kin Bypass After: Delta_um(%ld,%ld,%ld) Steps(%ld,%ld,%ld) \n"),
                   delta_um[X], delta_um[Y], delta_um[Z],
                   steps[X], steps[Y], steps[Z]);
  }
}

//Transform a single TARGET to delta coordinates
TARGET
delta_from_cartesian(TARGET *t){
  enum axis_e i;
  TARGET t_d;
  int32_t delta_x, delta_y, delta_z;

  for (i = X; i < E; i++){
    t_d.axis[i] = (t->axis[i] >> 4); //scale to allow squares
  }

  delta_x = SquareRoot32(DELTA_DIAGONAL_ROD_2
                     - (delta_tower1_x - t_d.axis[X]) * (delta_tower1_x - t_d.axis[X])
                     - (delta_tower1_y - t_d.axis[Y]) * (delta_tower1_y - t_d.axis[Y])
                     ) + t_d.axis[Z];
  delta_y = SquareRoot32(DELTA_DIAGONAL_ROD_2
                     - (delta_tower2_x - t_d.axis[X]) * (delta_tower2_x - t_d.axis[X])
                     - (delta_tower2_y - t_d.axis[Y]) * (delta_tower2_y - t_d.axis[Y])
                     ) + t_d.axis[Z];
  delta_z = SquareRoot32(DELTA_DIAGONAL_ROD_2
                     - (delta_tower3_x - t_d.axis[X]) * (delta_tower3_x - t_d.axis[X])
                     - (delta_tower3_y - t_d.axis[Y]) * (delta_tower3_y - t_d.axis[Y])
                     ) + t_d.axis[Z];

  t_d.axis[X]    = delta_x << 4;
  t_d.axis[Y]    = delta_y << 4;
  t_d.axis[Z]    = delta_z << 4;
  t_d.axis[E]    = t->axis[E];
  t_d.F          = t->F;
  t_d.e_relative = t->e_relative;

  return t_d;
}
#endif


void axes_um_to_steps_cartesian(const axes_int32_t um, axes_int32_t steps) {
  enum axis_e i;

  for (i = X; i < E; i++) {
    steps[i] = um_to_steps(um[i], i);
  }
}

void axes_um_to_steps_corexy(const axes_int32_t um, axes_int32_t steps) {
  steps[X] = um_to_steps(um[X] + um[Y], X);
  steps[Y] = um_to_steps(um[X] - um[Y], Y);
  steps[Z] = um_to_steps(um[Z], Z);
}
#ifdef DELTA_PRINTER
void axes_um_to_steps_delta(const axes_int32_t um, axes_int32_t steps) {
    enum axis_e i;

  
  if (bypass_delta==0)
  {
int32_t delta_x, delta_y, delta_z;
axes_int32_t t_d;
  for (i = X; i < E; i++){
    t_d[i] = (um[i] >> 4); //scale to allow squares
  }

  delta_x = SquareRoot32(DELTA_DIAGONAL_ROD_2
                     - (delta_tower1_x - t_d[X]) * (delta_tower1_x - t_d[X])
                     - (delta_tower1_y - t_d[Y]) * (delta_tower1_y - t_d[Y])
                     ) + t_d[Z];
  delta_y = SquareRoot32(DELTA_DIAGONAL_ROD_2
                     - (delta_tower2_x - t_d[X]) * (delta_tower2_x - t_d[X])
                     - (delta_tower2_y - t_d[Y]) * (delta_tower2_y - t_d[Y])
                     ) + t_d[Z];
  delta_z = SquareRoot32(DELTA_DIAGONAL_ROD_2
                     - (delta_tower3_x - t_d[X]) * (delta_tower3_x - t_d[X])
                     - (delta_tower3_y - t_d[Y]) * (delta_tower3_y - t_d[Y])
                     ) + t_d[Z];

  steps[X]    = um_to_steps(delta_x << 4,X);
  steps[Y]    = um_to_steps(delta_y << 4,Y);
  steps[Z]    = um_to_steps(delta_z << 4,Z);
    
  } else {


  for (i = X; i < E; i++) {
    steps[i] = um_to_steps(um[i], i);
  }}


}
#endif
