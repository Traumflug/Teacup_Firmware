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

  Copied from $(MBED)/libraries/mbed/hal/pinmap.h.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.

  - Prefixed names of #include files with mbed- to match the names of the
    copies in the Teacup repo.
*/
#ifndef MBED_PINMAP_H
#define MBED_PINMAP_H

#include "mbed-PinNames.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PinName pin;
    int peripheral;
    int function;
} PinMap;

void pin_function(PinName pin, int function);
void pin_mode    (PinName pin, PinMode mode);

#ifdef __cplusplus
}
#endif

#endif
