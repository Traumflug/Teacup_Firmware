#ifndef _DDA_SPLIT_H
#define _DDA_SPLIT_H

#include <stdint.h>

#include "config.h"

#ifdef ACCELERATION_SPLIT
  #ifdef ACCELERATION_RAMPING
    #error Cant use ACCELERATION_SPLIT and ACCELERATION_RAMPING together.
  #endif
  #ifndef ACCELERATION_REPRAP
    #error ACCELERATION_SPLIT requires ACCELERATION_REPRAP, too.
  #endif
  #if (MOVEBUFFER_SIZE < 16)
    #error MOVEBUFFER_SIZE has to be at least 16 (and 255 maximum).
  #endif
#endif


#endif /* _DDA_SPLIT_H */
