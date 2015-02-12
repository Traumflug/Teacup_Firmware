#ifndef _DDA_DELTA_SEGMENTS_H
#define _DDA_DELTA_SEGMENTS_H

#include <stdint.h>

#include "config.h"
#include "dda.h"

#ifdef DELTA_PRINTER
  #if (MOVEBUFFER_SIZE < 16)
    #error MOVEBUFFER_SIZE has to be at least 16 (and 255 maximum).
  #endif
#endif


void delta_segments_create(TARGET *target);

#endif /* _DDA_DELTA_SEGMENTS_H */
