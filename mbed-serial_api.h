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

  Copied from $(MBED)/libraries/mbed/hal/serial_api.h.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.

  - Prefixed names of #include files with mbed- to match the names of the
    copies in the Teacup repo.
  - Prefixed function names with mbed_ to not conflict with Teacup names.
  - Different device.h for STM32 and LPC
*/
#ifndef MBED_SERIAL_API_H
#define MBED_SERIAL_API_H

#ifdef __ARM_LPC1114__
  #include "mbed-device.h"
#elif __ARM_STM32F411__
  #include "mbed-device_stm32.h"
#endif

#if DEVICE_SERIAL

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ParityNone = 0,
    ParityOdd = 1,
    ParityEven = 2,
    ParityForced1 = 3,
    ParityForced0 = 4
} SerialParity;

typedef enum {
    RxIrq,
    TxIrq
} SerialIrq;

typedef enum {
    FlowControlNone,
    FlowControlRTS,
    FlowControlCTS,
    FlowControlRTSCTS
} FlowControl;

typedef void (*uart_irq_handler)(uint32_t id, SerialIrq event);

typedef struct serial_s serial_t;

void mbed_serial_init       (serial_t *obj, PinName tx, PinName rx);
void mbed_serial_free       (serial_t *obj);
void mbed_serial_baud       (serial_t *obj, int baudrate);
void mbed_serial_format     (serial_t *obj, int data_bits, SerialParity parity, int stop_bits);

void mbed_serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id);
void mbed_serial_irq_set    (serial_t *obj, SerialIrq irq, uint32_t enable);

int  mbed_serial_getc       (serial_t *obj);
void mbed_serial_putc       (serial_t *obj, int c);
int  mbed_serial_readable   (serial_t *obj);
int  mbed_serial_writable   (serial_t *obj);
void mbed_serial_clear      (serial_t *obj);

void mbed_serial_break_set  (serial_t *obj);
void mbed_serial_break_clear(serial_t *obj);

void mbed_serial_set_flow_control(serial_t *obj, FlowControl type, PinName rxflow, PinName txflow);

#ifdef __cplusplus
}
#endif

#endif

#endif
