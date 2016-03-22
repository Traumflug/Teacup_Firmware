
/** \file
  \brief MCU pin mappings.

  Here we map the pins required by Teacup to the names known by CMSIS.
*/

/** I/O pins.

  In MBED, I/O pin handling is rather complicated. Lots of enums, lots of
  functions, spread over various files, slow execution (pin toggling about
  15 times slower than what we have here).

  FASTIO by setting the BSRR (bit set/reset register),
  - Bit0-15 to set
  - Bit16-31 to reset.

  Pins set for Nucleo F411RE, for other STM32F4-boards you need to add them.
*/

#include "cmsis-stm32f4xx.h"

#define F_CPU __SYSTEM_CLOCK

/** Pins for UART, the serial port.
*/
#define RXD_PORT        PA_3_PORT
#define RXD_PIN         PA_3_PIN
#define TXD_PORT        PA_2_PORT
#define TXD_PIN         PA_2_PIN

/**
  We define only pins available on the Nucleo F411RE here.
  Use alphas for PORT and numerics for PIN, close to the design.
*/

#define PA_0_PIN      0
#define PA_0_PORT     GPIOA
#define PA_0_ADC      0
#define PA_0_AF       1
#define PA_0_TIMER    TIM2
#define PA_0_CHANNEL  1
#define PA_0_INVERT   0

#define PA_1_PIN      1
#define PA_1_PORT     GPIOA
#define PA_1_ADC      1
#define PA_1_AF       1
#define PA_1_TIMER    TIM2
#define PA_1_CHANNEL  2
#define PA_1_INVERT   0

#define PA_2_PIN      2
#define PA_2_PORT     GPIOA
// #define PA_2_ADC      2
// #define PA_2_AF       3
// #define PA_2_TIMER    TIM9
// #define PA_2_CHANNEL  1
// #define PA_2_INVERT   0

#define PA_3_PIN      3
#define PA_3_PORT     GPIOA
// #define PA_3_ADC      3
// #define PA_3_AF       3
// #define PA_3_TIMER    TIM9
// #define PA_3_CHANNEL  2
// #define PA_3_INVERT   0

#define PA_4_PIN      4
#define PA_4_PORT     GPIOA
#define PA_4_ADC      4

#define PA_5_PIN      5
#define PA_5_PORT     GPIOA
#define PA_5_ADC      5
#define PA_5_AF       1
#define PA_5_TIMER    TIM2
#define PA_5_CHANNEL  1
#define PA_5_INVERT   0

#define PA_6_PIN      6
#define PA_6_PORT     GPIOA
#define PA_6_ADC      6
// #define PA_6_AF       2
// #define PA_6_TIMER    TIM3
// #define PA_6_CHANNEL  1
// #define PA_6_INVERT   0

#define PA_7_PIN      7
#define PA_7_PORT     GPIOA
#define PA_7_ADC      7
// #define PA_7_AF       1
// #define PA_7_TIMER    TIM1
// #define PA_7_CHANNEL  1
// #define PA_7_INVERT   1

#define PA_8_PIN      8
#define PA_8_PORT     GPIOA
#define PA_8_AF       1
#define PA_8_TIMER    TIM1
#define PA_8_CHANNEL  1
#define PA_8_INVERT   0

#define PA_9_PIN      9
#define PA_9_PORT     GPIOA
#define PA_9_AF       1
#define PA_9_TIMER    TIM1
#define PA_9_CHANNEL  2
#define PA_9_INVERT   0

#define PA_10_PIN     10
#define PA_10_PORT    GPIOA
#define PA_10_AF      1
#define PA_10_TIMER   TIM1
#define PA_10_CHANNEL 3
#define PA_10_INVERT  0

#define PA_11_PIN     11
#define PA_11_PORT    GPIOA
#define PA_11_AF       1
#define PA_11_TIMER    TIM1
#define PA_11_CHANNEL  4
#define PA_11_INVERT   0

#define PA_12_PIN     12
#define PA_12_PORT    GPIOA

#define PA_13_PIN     13
#define PA_13_PORT    GPIOA

#define PA_14_PIN     14
#define PA_14_PORT    GPIOA

#define PA_15_PIN     15
#define PA_15_PORT    GPIOA
// #define PA_15_AF      1
// #define PA_15_TIMER   TIM2
// #define PA_15_CHANNEL 1
// #define PA_15_INVERT  0

#define PB_0_PIN      0
#define PB_0_PORT     GPIOB
#define PB_0_ADC      8
#define PB_0_AF       2
#define PB_0_TIMER    TIM3
#define PB_0_CHANNEL  3
#define PB_0_INVERT   0

#define PB_1_PIN      1
#define PB_1_PORT     GPIOB
#define PB_1_ADC      9
#define PB_1_AF       2
#define PB_1_TIMER    TIM3
#define PB_1_CHANNEL  4
#define PB_1_INVERT   0

#define PB_2_PIN      2
#define PB_2_PORT     GPIOB

#define PB_3_PIN      3
#define PB_3_PORT     GPIOB
// #define PB_3_AF       1
// #define PB_3_TIMER    TIM2
// #define PB_3_CHANNEL  2
// #define PB_3_INVERT   0

#define PB_4_PIN      4
#define PB_4_PORT     GPIOB
#define PB_4_AF       2
#define PB_4_TIMER    TIM3
#define PB_4_CHANNEL  1
#define PB_4_INVERT   0

#define PB_5_PIN      5
#define PB_5_PORT     GPIOB
#define PB_5_AF       2
#define PB_5_TIMER    TIM3
#define PB_5_CHANNEL  1
#define PB_5_INVERT   0

#define PB_6_PIN      6
#define PB_6_PORT     GPIOB
#define PB_6_AF       2
#define PB_6_TIMER    TIM4
#define PB_6_CHANNEL  1
#define PB_6_INVERT   0

#define PB_7_PIN      7
#define PB_7_PORT     GPIOB
#define PB_7_AF       2
#define PB_7_TIMER    TIM4
#define PB_7_CHANNEL  2
#define PB_7_INVERT   0

#define PB_8_PIN      8
#define PB_8_PORT     GPIOB
#define PB_8_AF       2
#define PB_8_TIMER    TIM4
#define PB_8_CHANNEL  3
#define PB_8_INVERT   0

#define PB_9_PIN      9
#define PB_9_PORT     GPIOB
#define PB_9_AF       2
#define PB_9_TIMER    TIM4
#define PB_9_CHANNEL  4
#define PB_9_INVERT   0

#define PB_10_PIN     10
#define PB_10_PORT    GPIOB
#define PB_10_AF      1
#define PB_10_TIMER   TIM2
#define PB_10_CHANNEL 3
#define PB_10_INVERT  0

#define PB_12_PIN     12
#define PB_12_PORT    GPIOB

#define PB_13_PIN     13
#define PB_13_PORT    GPIOB
// #define PB_13_AF      1
// #define PB_13_TIMER   TIM1
// #define PB_13_CHANNEL 1
// #define PB_13_INVERT  1

#define PB_14_PIN     14
#define PB_14_PORT    GPIOB
// #define PB_14_AF      1
// #define PB_14_TIMER   TIM1
// #define PB_14_CHANNEL 2
// #define PB_14_INVERT  1

#define PB_15_PIN     15
#define PB_15_PORT    GPIOB
// #define PB_15_AF      1
// #define PB_15_TIMER   TIM1
// #define PB_15_CHANNEL 3
// #define PB_15_INVERT  1

#define PC_0_PIN      0
#define PC_0_PORT     GPIOC
#define PC_0_ADC      10

#define PC_1_PIN      1
#define PC_1_PORT     GPIOC
#define PC_1_ADC      11

#define PC_2_PIN      2
#define PC_2_PORT     GPIOC
#define PC_2_ADC      12

#define PC_3_PIN      3
#define PC_3_PORT     GPIOC
#define PC_3_ADC      13

#define PC_4_PIN      4
#define PC_4_PORT     GPIOC
#define PC_4_ADC      14

#define PC_5_PIN      5
#define PC_5_PORT     GPIOC
#define PC_5_ADC      15

#define PC_6_PIN      6
#define PC_6_PORT     GPIOC
// #define PC_6_AF       2
// #define PC_6_TIMER    TIM3
// #define PC_6_CHANNEL  1
// #define PC_6_INVERT   0

#define PC_7_PIN      7
#define PC_7_PORT     GPIOC
// #define PC_7_AF       2
// #define PC_7_TIMER    TIM3
// #define PC_7_CHANNEL  2
// #define PC_7_INVERT   0

#define PC_8_PIN      8
#define PC_8_PORT     GPIOC
// #define PC_8_AF       2
// #define PC_8_TIMER    TIM3
// #define PC_8_CHANNEL  3
// #define PC_8_INVERT   0

#define PC_9_PIN      9
#define PC_9_PORT     GPIOC
// #define PC_9_AF       2
// #define PC_9_TIMER    TIM3
// #define PC_9_CHANNEL  4
// #define PC_9_INVERT   0

#define PC_10_PIN     10
#define PC_10_PORT    GPIOC

#define PC_11_PIN     11
#define PC_11_PORT    GPIOC

#define PC_12_PIN     12
#define PC_12_PORT    GPIOC

#define PC_13_PIN     13
#define PC_13_PORT    GPIOC

#define PC_14_PIN     14
#define PC_14_PORT    GPIOC

#define PC_15_PIN     15
#define PC_15_PORT    GPIOC

#define PD_2_PIN      2
#define PD_2_PORT     GPIOD

#define PH_0_PIN      0
#define PH_0_PORT     GPIOH

#define PH_1_PIN      1
#define PH_1_PORT     GPIOH
