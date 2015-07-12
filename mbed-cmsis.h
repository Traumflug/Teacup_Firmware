/* mbed Microcontroller Library - CMSIS
 * Copyright (C) 2009-2011 ARM Limited. All rights reserved.
 * 
 * A generic CMSIS include header, pulling in LPC11U24 specifics
 */
/*
  Notes for Teacup:

  Copied from $(MBED)/libraries/mbed/targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/cmsis.h.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.

  - Prefixed names of #include files with mbed- to match the names of the
    copies in the Teacup repo.
*/

#ifndef MBED_CMSIS_H
#define MBED_CMSIS_H

#include "mbed-LPC11xx.h"
#include "mbed-cmsis_nvic.h"
#include "mbed-bitfields.h"

#endif
