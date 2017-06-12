#ifndef _DDA_DELTA_SEGMENTS_H
#define _DDA_DELTA_SEGMENTS_H

#include <stdint.h>

#include "config.h"
#include "dda.h"
#include	<avr/eeprom.h>

#ifdef DELTA_PRINTER
  #if (MOVEBUFFER_SIZE < 2)
    #error MOVEBUFFER_SIZE has to be at least 1 (and 255 maximum).
  #endif
#endif

extern int32_t EEMEM EE_deltasegment;
extern uint32_t _DELTA_SEGMENTS; 
void delta_segments_create(TARGET *target);

#endif /* _DDA_DELTA_SEGMENTS_H */
