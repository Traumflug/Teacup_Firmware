
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
#include	<avr/eeprom.h>

int32_t EEMEM EE_deltasegment;
#ifdef DELTASEGMENTS_TIME
uint32_t _DELTA_SEGMENTS=DELTA_SEGMENTS_PER_SECOND*1000;
#else
uint32_t _DELTA_SEGMENTS=DELTA_SEGMENTS_UM;
#endif

//#define debug2a
void delta_segments_create(TARGET *target) {
  int32_t dist;
  int32_t s,seg_size,cartesian_move_sec;
  int32_t segment_total;

  enum axis_e i;
  TARGET orig_startpoint;
  TARGET segment;
  TARGET diff,diff_frac;//,diff_scaled;

  if (target == NULL) {
    // it's a wait for temp
    enqueue_home(NULL, 0, 0);
    return;
  }

   orig_startpoint = startpoint;

   seg_size = _DELTA_SEGMENTS;

   for (i = X; i < AXIS_COUNT; i++){
      diff.axis[i] = target->axis[i] - orig_startpoint.axis[i];
     
   }

  dist = approx_distance(labs(diff.axis[X]),labs(diff.axis[Y]));//,labs(diff.axis[Z]));

  #ifdef debug2
  //if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA))
    sersendf_P(PSTR("\n\nSeg_Start (%ld,%ld,%ld,E:%ld F:%lu ER:%u)Seg_Target(%ld,%ld,%ld,E:%ld F:%lu ER:%u) dist:%lu\n"),
                orig_startpoint.axis[X], orig_startpoint.axis[Y], orig_startpoint.axis[Z],orig_startpoint.axis[E],orig_startpoint.F,orig_startpoint.e_relative,
                target->axis[X], target->axis[Y], target->axis[Z],target->axis[E],target->F,target->e_relative,dist);
  #endif

  //The Marlin/Repetier Approach
#ifdef DELTASEGMENTS_TIME
  cartesian_move_sec = (dist / target->F) * 60 >> 4;   //milisec distance(um) * 1mm/1000um * (Feedrate)1min/mm * 60sec/min

  if (target->F > 0)
    segment_total = (_DELTA_SEGMENTS * cartesian_move_sec/1000) >>6;   //distance(um) * 1mm/1000um * (Feedrate)1min/mm * 60sec/min
  else
    segment_total = 0;
  //sersendf_P(PSTR("SEG:Z or small: dist: %lu segs: %lu move_sec: %lu\n"),dist,segment_total,cartesian_move_sec);  
#endif /* DELTASEGMENTS_TIME */
#ifdef DELTASEGMENTS_DISTANCE
  segment_total = dist / seg_size+1;
#endif

#ifdef DELTASEGMENTS_DISTANCE
  if ((diff.axis[X] == 0 && diff.axis[Y] == 0) || dist < (2 * seg_size))
#endif /* DELTASEGMENTS_DISTANCE */
#ifdef DELTASEGMENTS_TIME
  if ((diff.axis[X] == 0 && diff.axis[Y] == 0) || segment_total < 2)
#endif /* DELTASEGMENTS_TIME */
  {
    segment_total = 1;
#ifdef debug2a

    #ifdef DELTASEGMENTS_TIME
        //if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA))
          sersendf_P(PSTR("SEG:Z or small: dist: %lu segs: %lu move_sec: %lu\n"),
                      dist,segment_total,cartesian_move_sec);
        
    #endif
    #ifdef DELTASEGMENTS_DISTANCE
      //if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA))
        sersendf_P(PSTR("SEG:Z or small: dist: %lu segs: %lu seg_size: %lu\n"),
            dist,segment_total,seg_size);

    #endif
#endif
    //sersendf_P(PSTR("1seg\n"));
    //segment = *target;
    enqueue_home(target,0,0);
  } else {

    for (i = X; i < AXIS_COUNT; i++)
      diff_frac.axis[i] = diff.axis[i] / segment_total;
#ifdef debug2
    //if (DEBUG_DELTA && (debug_flags & DEBUG_DELTA))
      sersendf_P(PSTR("SEG: Frac(%ld,%ld,%ld,E:%ld) dist: %lu segs: %lu move_sec: %lu\n"),
                diff_frac.axis[X],diff_frac.axis[Y],diff_frac.axis[Z],diff_frac.axis[E],
                dist, segment_total,cartesian_move_sec);
#endif
    //if you do all segments, rounding error reduces total - error will accumulate
    for (s=1; s<segment_total; s++){
      for (i = X; i < AXIS_COUNT; i++)
        segment.axis[i] = (orig_startpoint.axis[i] + (s * diff_frac.axis[i]));
        //sersendf_P(PSTR("SEG: %lu "),s);
        segment.F=target->F;
        segment.e_relative = target->e_relative;
        
        enqueue_home(&segment,0,0);
    }
    //last bit to make sure we end up at the unsegmented target
    //segment = *target;
    enqueue_home(target,0,0);
  }
}
