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

