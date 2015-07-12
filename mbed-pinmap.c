/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
  Notes for Teacup:

  Copied from $(MBED)/libraries/mbed/targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/pinmap.c.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.

  - Prefixed names of #include files with mbed- to match the names of the
    copies in the Teacup repo.
  - Wrapped the whole file in #ifdef __ARMEL__ to not cause conflicts with
    AVR builds.
*/
#ifdef __ARMEL__
#include "mbed-pinmap.h"

void pin_function(PinName pin, int function) {
    uint32_t offset = (uint32_t)pin & 0xff;
    __IO uint32_t *reg = (__IO uint32_t*)(LPC_IOCON_BASE + offset);

    // pin function bits: [2:0] -> 111 = (0x7)
    *reg = (*reg & ~0x7) | (function & 0x7);
}

void pin_mode(PinName pin, PinMode mode) {
    uint32_t offset = (uint32_t)pin & 0xff;
    uint32_t drain = ((uint32_t)mode & (uint32_t)OpenDrain) >> 2;
    
    __IO uint32_t *reg = (__IO uint32_t*)(LPC_IOCON_BASE + offset);
    uint32_t tmp = *reg;
    
    // pin mode bits: [4:3] -> 11000 = (0x3 << 3)
    tmp &= ~(0x3 << 3);
    tmp |= (mode & 0x3) << 3;
    
    // drain
    tmp &= ~(0x1 << 10);
    tmp |= drain << 10;
    
    *reg = tmp;
}
#endif /* __ARMEL__ */
