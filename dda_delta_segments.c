
/** \file
  \function to create delta segments from cartesian movements

  Patterned off dda_split branch dda_split

  To allow for look-ahead, some sub-movements will have to be held back,
  until following movement(s) come in. Still, the functions here will
  never block.
*/

#include "dda_delta_segments.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "sersendf.h"
#include "dda_queue.h"
#include "dda_maths.h"
#include "dda.h"
#include "debug.h"


void delta_segments_create(TARGET *target) {
  uint32_t dist,cartesian_move_sec;
  uint32_t s,fraction;
  int32_t segment_total,diff_frac_x,diff_frac_y,diff_frac_z,diff_frac_e;

  enum axis_e i;
  TARGET orig_startpoint;
  TARGET segment;

  if (target == NULL) {
    // it's a wait for temp
    enqueue_home(NULL, 0, 0);
    return;
  }

  orig_startpoint = startpoint;

  /* distance whole movement */
  //dist = approx_distance_3((uint32_t)labs(target->axis[X] - orig_startpoint.axis[X]),
  //                         (uint32_t)labs(target->axis[Y] - orig_startpoint.axis[Y]),
  //                         (uint32_t)labs(target->axis[Z] - orig_startpoint.axis[Z]));


  //dist = sqrt((target->axis[X] - orig_startpoint.axis[X])*(target->axis[X]) - orig_startpoint.axis[X])
  //           +(target->axis[Y] - orig_startpoint.axis[Y])*(target->axis[Y]) - orig_startpoint.axis[Y])
  //           +(target->axis[Z] - orig_startpoint.axis[Z])*(target->axis[Z]) - orig_startpoint.axis[Z]));

  dist = sqrt( pow((target->axis[X] - orig_startpoint.axis[X])>>4,2) +
               pow((target->axis[Y] - orig_startpoint.axis[Y])>>4,2) +
               pow((target->axis[Z] - orig_startpoint.axis[Z])>>4,2));

  dist = dist << 4;

  if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA)){
    sersendf_P(PSTR("\nSeg_Start (%ld,%ld,%ld)\nSeg_Target(%ld,%ld,%ld,F:%lu) dist:%lu\n"),
                orig_startpoint.axis[X], orig_startpoint.axis[Y], orig_startpoint.axis[Z],
                target->axis[X], target->axis[Y], target->axis[Z],target->F,dist);
  }

  cartesian_move_sec = (dist / target->F) * 60 * 0.001;   //distance(um) * 1mm/1000um * (Feedrate)1min/mm * 60sec/min

  if (target->F > 0)
    segment_total = DELTA_SEGMENTS_PER_SECOND * (dist / target->F) * 60 * 0.001;   //distance(um) * 1mm/1000um * (Feedrate)1min/mm * 60sec/min
  else
    segment_total = 0;


  if (segment_total < 1)
  {
    segment_total = 1;
    segment = *target;
    enqueue_home(&segment,0,0);

    if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA)){
      sersendf_P(PSTR("Seg: ALL move_sec: %lu segs: %lu\n"),
                cartesian_move_sec, segment_total);
     }
  }
  else
  {
    diff_frac_x =  (target->axis[X] - orig_startpoint.axis[X]) / segment_total;
    diff_frac_y =  (target->axis[Y] - orig_startpoint.axis[Y]) / segment_total;
    diff_frac_z =  (target->axis[Z] - orig_startpoint.axis[Z]) / segment_total;
    diff_frac_e =  (target->axis[E] - orig_startpoint.axis[E]) / segment_total;

    if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA)){
      sersendf_P(PSTR("Seg: df(%ld,%ld,%ld,E:%ld) move_sec: %lu segs: %lu\n"),
                diff_frac_x,diff_frac_y,diff_frac_z,diff_frac_e,
                cartesian_move_sec, segment_total);
    }
    //if you do all segments, rounding error reduces total - error will accumulate
    for (s=1;s<segment_total;s++){
      if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA)){
        sersendf_P(PSTR("Seg: %d\n"),s);
      }
      segment.axis[X] = (orig_startpoint.axis[X] + (s * diff_frac_x));
      segment.axis[Y] = (orig_startpoint.axis[Y] + (s * diff_frac_y));
      segment.axis[Z] = (orig_startpoint.axis[Z] + (s * diff_frac_z));
      segment.axis[E] = (orig_startpoint.axis[E] + (s * diff_frac_e));
      segment.F = target->F;
      enqueue_home(&segment,0,0);
    }
    //last bit to make sure we end up at the unsegmented target
    if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA)){
      sersendf_P(PSTR("Seg: Final"));
    }
    segment = *target;
    enqueue_home(&segment,0,0);
  }
}
