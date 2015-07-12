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

  Copied from $(MBED)/mbed/libraries/mbed/targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/gpio_object.h.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.

  - Prefixed names of #include files with mbed- to match the names of the
    copies in the Teacup repo.
*/
#ifndef MBED_GPIO_OBJECT_H
#define MBED_GPIO_OBJECT_H

#include "mbed-mbed_assert.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PinName  pin;
    __I  uint32_t *reg_mask_read;
    __IO uint32_t *reg_dir;
    __IO uint32_t *reg_write;
} gpio_t;

static inline void gpio_write(gpio_t *obj, int value) {
    MBED_ASSERT(obj->pin != (PinName)NC);
    uint32_t pin_number = ((obj->pin & 0x0F00) >> 8);
    if (value)
        *obj->reg_write |= (1 << pin_number);
    else
        *obj->reg_write &= ~(1 << pin_number);
}

static inline int gpio_read(gpio_t *obj) {
    MBED_ASSERT(obj->pin != (PinName)NC);
    return ((*obj->reg_mask_read) ? 1 : 0);
}

static inline int gpio_is_connected(const gpio_t *obj) {
    return obj->pin != (PinName)NC;
}

#ifdef __cplusplus
}
#endif

#endif
