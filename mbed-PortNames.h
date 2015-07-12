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

  Copied from $(MBED)/libraries/mbed/targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/PortNames.h.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.
*/
#ifndef MBED_PORTNAMES_H
#define MBED_PORTNAMES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    Port0 = 0,
    Port1 = 1,
    Port2 = 2,
    Port3 = 3
} PortName;

#ifdef __cplusplus
}
#endif
#endif
