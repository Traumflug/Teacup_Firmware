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

  Copied from $(MBED)/libraries/mbed/targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/PeripheralNames.h.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.

  - Prefixed names of #include files with mbed- to match the names of the
    copies in the Teacup repo.
*/
#ifndef MBED_PERIPHERALNAMES_H
#define MBED_PERIPHERALNAMES_H

#include "mbed-cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART_0 = (int)LPC_UART_BASE
} UARTName;

typedef enum {
    I2C_0 = (int)LPC_I2C_BASE
} I2CName;

typedef enum {
    ADC0_0 = 0,
    ADC0_1,
    ADC0_2,
    ADC0_3,
    ADC0_4,
    ADC0_5,
    ADC0_6,
    ADC0_7
} ADCName;

typedef enum {
    SPI_0 = (int)LPC_SSP0_BASE,
    SPI_1 = (int)LPC_SSP1_BASE
} SPIName;

typedef enum {
    PWM_1 = 0,
    PWM_2,
    PWM_3,
    PWM_4,
    PWM_5
} PWMName;

#define STDIO_UART_TX     USBTX
#define STDIO_UART_RX     USBRX
#define STDIO_UART        UART_0

#ifdef __cplusplus
}
#endif

#endif
