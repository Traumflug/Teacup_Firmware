#include "dda_split.h"

/** \file
  \brief Movement splitter.

  After lots and lots of failed tries to squeeze proper look-ahead and smooth
  movement junctions into approaches with an 1:1 ratio of G1 commands and
  movement queue entries, this file was created. It splits each movement
  into one or more sub-movements, each of which is queued up.

  These sub-movements can be:

   - linear, unaccelerated move
   - linear, accelerated/decelerated move
   - curved, unaccelerated move

  To allow for look-ahead, some sub-movements will have to be held back,
  until following movement(s) come in. Still, the functions here will
  never block.
*/

#include "dda_queue.h"
#include "dda_maths.h"

/**
  To cover two adjectant movements, we have to create up to 8 segments:

  1)   acceleration on the first segment up to target speed
  2)   linear, unaccelerated movement at target speed
  3)   deceleration to junction speed
  4/5) one or two junction curves (curves have to be split at quadrant borders)
  6)   acceleration to target speed
  7)   linear, unaccelerated movement at target speed
  8)   deceleration to stop.
*/

TARGET sp1 = {0, 0, 0, 0};  // start of 1)

void split_create(TARGET *target) {
  uint32_t dist, acc_dist;
  TARGET segment;

  if (target == NULL) {
    // it's a wait for temp
    enqueue_home(NULL, 0, 0);
    return;
  }

  /* distance whole movement */
  dist = approx_distance_3((uint32_t)labs(target->X - sp1.X),
                           (uint32_t)labs(target->Y - sp1.Y),
                           (uint32_t)labs(target->Z - sp1.Z));

  /* acceleration/deceleration distance stop to target speed
     s = 2 * v^2 / a    */
  acc_dist = muldiv(2000, (uint32_t)target->F * (uint32_t)target->F, 3600 * ACCELERATION);
  //sersendf_P(PSTR("dist %lq, acc_dist %lq\n"), dist, acc_dist);

  /* the general short case: accelerate, decelerate */
  if (2 * acc_dist <= dist) {
    segment.X = (target->X + sp1.X) / 2;
    segment.Y = (target->Y + sp1.Y) / 2;
    segment.Z = (target->Z + sp1.Z) / 2;
    segment.F = muldiv(target->F, dist / 2, acc_dist);
    enqueue_home(&segment, 0, 0);

    segment = *target;
    segment.F = 0;
    enqueue_home(&segment, 0, 0);
  }
  /* the general long case: accelerate, move, decelerate */
  else {
  }
  sp1 = *target;
}

