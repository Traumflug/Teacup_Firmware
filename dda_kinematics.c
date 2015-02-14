#include "dda_kinematics.h"
#include "sersendf.h"
#include "debug.h"

/** \file G-code axis system to stepper motor axis system conversion.
*/

#include <stdlib.h>

#include "dda_maths.h"

void carthesian_to_carthesian(TARGET *startpoint, TARGET *target,
                         axes_uint32_t delta_um, axes_int32_t steps) {
  enum axis_e i;

  for (i = X; i < E; i++) {
    delta_um[i] = (uint32_t)labs(target->axis[i] - startpoint->axis[i]);
    steps[i] = um_to_steps(target->axis[i], i);
  }
}

void carthesian_to_corexy(TARGET *startpoint, TARGET *target,
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

#ifdef DELTA_PRINTER
void carthesian_to_delta(TARGET *startpoint, TARGET *target,
                     axes_uint32_t delta_um, axes_int32_t steps) {

  enum axis_e i;
  TARGET startpoint_d,target_d;
  TARGET delkin_start,delkin_target;

  if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA)){
     sersendf_P(PSTR("Kin Before: Start(%ld,%ld,%ld,F:%ld) Target(%ld,%ld,%ld,F:%ld) \n"),
                startpoint->axis[X], startpoint->axis[Y], startpoint->axis[Z], startpoint->F,
                target->axis[X], target->axis[Y], target->axis[Z], target->F);
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
        sersendf_P(PSTR("Kin After: d_Start(%ld,%ld,%ld) d_Target(%ld,%ld,%ld) \n"),
                   delkin_start.axis[X], delkin_start.axis[Y], delkin_start.axis[Z],
                   delkin_target.axis[X], delkin_target.axis[Y], delkin_target.axis[Z]);
        sersendf_P(PSTR("Kin After: Delta_um(%ld,%ld,%ld) Steps(%ld,%ld,%ld) \n"),
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
TARGET delta_from_cartesian(TARGET *t){
   enum axis_e i;
   TARGET t_d;
   int32_t delta_x, delta_y, delta_z;
   
   for (i = X; i < E; i++){
      t_d.axis[i] = (t->axis[i] >> 4); //scale to allow squares
   }
   
   delta_x = sqrt(DELTA_DIAGONAL_ROD_2
                     - (delta_tower1_x - t_d.axis[X]) * (delta_tower1_x - t_d.axis[X])
                     - (delta_tower1_y - t_d.axis[Y]) * (delta_tower1_y - t_d.axis[Y])
                     ) + t_d.axis[Z];
   delta_y = sqrt(DELTA_DIAGONAL_ROD_2
                     - (delta_tower2_x - t_d.axis[X]) * (delta_tower2_x - t_d.axis[X])
                     - (delta_tower2_y - t_d.axis[Y]) * (delta_tower2_y - t_d.axis[Y])
                     ) + t_d.axis[Z];
   delta_z = sqrt(DELTA_DIAGONAL_ROD_2
                     - (delta_tower3_x - t_d.axis[X]) * (delta_tower3_x - t_d.axis[X])
                     - (delta_tower3_y - t_d.axis[Y]) * (delta_tower3_y - t_d.axis[Y])
                     ) + t_d.axis[Z];
                     
   t_d.axis[X] = delta_x << 4;
   t_d.axis[Y] = delta_y << 4;
   t_d.axis[Z] = delta_z << 4;
   t_d.axis[E] = t->axis[E];
   t_d.F       = t->F; 
   
   return t_d;
}
#endif
