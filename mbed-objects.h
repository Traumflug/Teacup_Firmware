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

  Copied from $(MBED)/libraries/mbed/targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/objects.h.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.

  - Prefixed names of #include files with mbed- to match the names of the
    copies in the Teacup repo.
*/
#ifndef MBED_OBJECTS_H
#define MBED_OBJECTS_H

#include "mbed-cmsis.h"
#include "mbed-PortNames.h"
#include "mbed-PeripheralNames.h"
#include "mbed-PinNames.h"

#ifdef __cplusplus
extern "C" {
#endif

struct gpio_irq_s {
    uint32_t ch;
    PinName pin;
    __I  uint32_t *reg_mask_read;
};

struct port_s {
    __IO uint32_t *reg_dir;
    __IO uint32_t *reg_data;
    PortName port;
    uint32_t mask;
};

struct pwmout_s {
    PWMName pwm;
};

struct serial_s {
    LPC_UART_TypeDef *uart;
    int index;
};

struct analogin_s {
    ADCName adc;
};

struct i2c_s {
    LPC_I2C_TypeDef *i2c;
};

struct spi_s {
    LPC_SSP_TypeDef *spi;
};

#if DEVICE_CAN
struct can_s {
    int index;
};
#endif

#include "mbed-gpio_object.h"

#ifdef __cplusplus
}
#endif

#endif
