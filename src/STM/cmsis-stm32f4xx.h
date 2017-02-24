/**
  ******************************************************************************
  * @file    stm32f4xx.h
  * @author  MCD Application Team
  * @version V2.3.2
  * @date    26-June-2015
  * @brief   CMSIS STM32F4xx Device Peripheral Access Layer Header File.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32F4xx device used in the target application
  *              - To use or not the peripheral’s drivers in application code(i.e.
  *                code will be based on direct access to peripheral’s registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_HAL_DRIVER"
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*
  Notes for Teacup:

  Copied from $(MBED)/libraries/mbed/targets/cmsis/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F411RE/stm32f411xe.h and stm32f4xx.h

  - Prefixed names of #include files with cmsis- to match the names of the
    copies in the Teacup repo.
  - Prefixed function names with mbed_ to not conflict with Teacup names.
  - Comment out HAL_DRIVERS
*/

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx
  * @{
  */

#ifndef __STM32F4xx_H
#define __STM32F4xx_H

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief STM32 Family
  */
#if !defined  (STM32F4)
#define STM32F4
#endif /* STM32F4 */

/* Uncomment the line below according to the target STM32 device used in your
   application
  */
#if !defined (STM32F405xx) && !defined (STM32F415xx) && !defined (STM32F407xx) && !defined (STM32F417xx) && \
    !defined (STM32F427xx) && !defined (STM32F437xx) && !defined (STM32F429xx) && !defined (STM32F439xx) && \
    !defined (STM32F401xC) && !defined (STM32F401xE) && !defined (STM32F411xE) && !defined (STM32F446xx)
  /* #define STM32F405xx */   /*!< STM32F405RG, STM32F405VG and STM32F405ZG Devices */
  /* #define STM32F415xx */   /*!< STM32F415RG, STM32F415VG and STM32F415ZG Devices */
  /* #define STM32F407xx */   /*!< STM32F407VG, STM32F407VE, STM32F407ZG, STM32F407ZE, STM32F407IG  and STM32F407IE Devices */
  /* #define STM32F417xx */   /*!< STM32F417VG, STM32F417VE, STM32F417ZG, STM32F417ZE, STM32F417IG and STM32F417IE Devices */
  /* #define STM32F427xx */   /*!< STM32F427VG, STM32F427VI, STM32F427ZG, STM32F427ZI, STM32F427IG and STM32F427II Devices */
  /* #define STM32F437xx */   /*!< STM32F437VG, STM32F437VI, STM32F437ZG, STM32F437ZI, STM32F437IG and STM32F437II Devices */
  /* #define STM32F429xx */   /*!< STM32F429VG, STM32F429VI, STM32F429ZG, STM32F429ZI, STM32F429BG, STM32F429BI, STM32F429NG,
                                   STM32F439NI, STM32F429IG  and STM32F429II Devices */
  /* #define STM32F439xx */   /*!< STM32F439VG, STM32F439VI, STM32F439ZG, STM32F439ZI, STM32F439BG, STM32F439BI, STM32F439NG,
                                   STM32F439NI, STM32F439IG and STM32F439II Devices */
  /* #define STM32F401xC */   /*!< STM32F401CB, STM32F401CC, STM32F401RB, STM32F401RC, STM32F401VB and STM32F401VC Devices */
  /* #define STM32F401xE */   /*!< STM32F401CD, STM32F401RD, STM32F401VD, STM32F401CE, STM32F401RE and STM32F401VE Devices */
  #define STM32F411xE         /*!< STM32F411CD, STM32F411RD, STM32F411VD, STM32F411CE, STM32F411RE and STM32F411VE Devices */
  /* #define STM32F446xx */   /*!< STM32F446MC, STM32F446ME, STM32F446RC, STM32F446RE, STM32F446VC, STM32F446VE, STM32F446ZC,
                                   and STM32F446ZE Devices */
#endif

/**
  * @brief CMSIS Device version number V2.3.2
  */
#define __STM32F4xx_CMSIS_DEVICE_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM32F4xx_CMSIS_DEVICE_VERSION_SUB1   (0x03) /*!< [23:16] sub1 version */
#define __STM32F4xx_CMSIS_DEVICE_VERSION_SUB2   (0x02) /*!< [15:8]  sub2 version */
#define __STM32F4xx_CMSIS_DEVICE_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32F4xx_CMSIS_DEVICE_VERSION        ((__STM32F4xx_CMSIS_DEVICE_VERSION_MAIN << 24)\
                                                |(__STM32F4xx_CMSIS_DEVICE_VERSION_SUB1 << 16)\
                                                |(__STM32F4xx_CMSIS_DEVICE_VERSION_SUB2 << 8 )\
                                                |(__STM32F4xx_CMSIS_DEVICE_VERSION))

/**
  * @}
  */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
  */
#define __CM4_REV                 0x0001  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present                                   */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85      /*!< SPI5 global Interrupt                                             */
} IRQn_Type;

/**
  * @}
  */

#include "cmsis-core_cm4.h"             /* Cortex-M4 processor and core peripherals */
#include "cmsis-system_stm32f4xx.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */

typedef struct
{
  __IO uint32_t SR;     /*!< ADC status register,                         Address offset: 0x00 */
  __IO uint32_t CR1;    /*!< ADC control register 1,                      Address offset: 0x04 */
  __IO uint32_t CR2;    /*!< ADC control register 2,                      Address offset: 0x08 */
  __IO uint32_t SMPR1;  /*!< ADC sample time register 1,                  Address offset: 0x0C */
  __IO uint32_t SMPR2;  /*!< ADC sample time register 2,                  Address offset: 0x10 */
  __IO uint32_t JOFR1;  /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
  __IO uint32_t JOFR2;  /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
  __IO uint32_t JOFR3;  /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
  __IO uint32_t JOFR4;  /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
  __IO uint32_t HTR;    /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
  __IO uint32_t LTR;    /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
  __IO uint32_t SQR1;   /*!< ADC regular sequence register 1,             Address offset: 0x2C */
  __IO uint32_t SQR2;   /*!< ADC regular sequence register 2,             Address offset: 0x30 */
  __IO uint32_t SQR3;   /*!< ADC regular sequence register 3,             Address offset: 0x34 */
  __IO uint32_t JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38*/
  __IO uint32_t JDR1;   /*!< ADC injected data register 1,                Address offset: 0x3C */
  __IO uint32_t JDR2;   /*!< ADC injected data register 2,                Address offset: 0x40 */
  __IO uint32_t JDR3;   /*!< ADC injected data register 3,                Address offset: 0x44 */
  __IO uint32_t JDR4;   /*!< ADC injected data register 4,                Address offset: 0x48 */
  __IO uint32_t DR;     /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
  __IO uint32_t CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
  __IO uint32_t CDR;    /*!< ADC common regular data register for dual
                             AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

/**
  * @brief CRC calculation unit
  */

typedef struct
{
  __IO uint32_t DR;         /*!< CRC Data register,             Address offset: 0x00 */
  __IO uint8_t  IDR;        /*!< CRC Independent data register, Address offset: 0x04 */
  uint8_t       RESERVED0;  /*!< Reserved, 0x05                                      */
  uint16_t      RESERVED1;  /*!< Reserved, 0x06                                      */
  __IO uint32_t CR;         /*!< CRC Control register,          Address offset: 0x08 */
} CRC_TypeDef;

/**
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;  /*!< MCU device ID code,               Address offset: 0x00 */
  __IO uint32_t CR;      /*!< Debug MCU configuration register, Address offset: 0x04 */
  __IO uint32_t APB1FZ;  /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
  __IO uint32_t APB2FZ;  /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
}DBGMCU_TypeDef;


/**
  * @brief DMA Controller
  */

typedef struct
{
  __IO uint32_t CR;     /*!< DMA stream x configuration register      */
  __IO uint32_t NDTR;   /*!< DMA stream x number of data register     */
  __IO uint32_t PAR;    /*!< DMA stream x peripheral address register */
  __IO uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  __IO uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  __IO uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
  __IO uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  __IO uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  __IO uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  __IO uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;


/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  __IO uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  __IO uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  __IO uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  __IO uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  __IO uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

/**
  * @brief FLASH Registers
  */

typedef struct
{
  __IO uint32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  __IO uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  __IO uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  __IO uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
  __IO uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
  __IO uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  __IO uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;

/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register       Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/**
  * @brief System configuration controller
  */

typedef struct
{
  __IO uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __IO uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  __IO uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  __IO uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  __IO uint32_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  __IO uint32_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  __IO uint32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
  __IO uint32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  __IO uint32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  __IO uint32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  __IO uint32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
  __IO uint32_t FLTR;       /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_TypeDef;

/**
  * @brief Independent WATCHDOG
  */

typedef struct
{
  __IO uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_TypeDef;

/**
  * @brief Power Control
  */

typedef struct
{
  __IO uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  __IO uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */

} RCC_TypeDef;

/**
  * @brief Real-Time Clock
  */

typedef struct
{
  __IO uint32_t TR;      /*!< RTC time register,                                        Address offset: 0x00 */
  __IO uint32_t DR;      /*!< RTC date register,                                        Address offset: 0x04 */
  __IO uint32_t CR;      /*!< RTC control register,                                     Address offset: 0x08 */
  __IO uint32_t ISR;     /*!< RTC initialization and status register,                   Address offset: 0x0C */
  __IO uint32_t PRER;    /*!< RTC prescaler register,                                   Address offset: 0x10 */
  __IO uint32_t WUTR;    /*!< RTC wakeup timer register,                                Address offset: 0x14 */
  __IO uint32_t CALIBR;  /*!< RTC calibration register,                                 Address offset: 0x18 */
  __IO uint32_t ALRMAR;  /*!< RTC alarm A register,                                     Address offset: 0x1C */
  __IO uint32_t ALRMBR;  /*!< RTC alarm B register,                                     Address offset: 0x20 */
  __IO uint32_t WPR;     /*!< RTC write protection register,                            Address offset: 0x24 */
  __IO uint32_t SSR;     /*!< RTC sub second register,                                  Address offset: 0x28 */
  __IO uint32_t SHIFTR;  /*!< RTC shift control register,                               Address offset: 0x2C */
  __IO uint32_t TSTR;    /*!< RTC time stamp time register,                             Address offset: 0x30 */
  __IO uint32_t TSDR;    /*!< RTC time stamp date register,                             Address offset: 0x34 */
  __IO uint32_t TSSSR;   /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
  __IO uint32_t CALR;    /*!< RTC calibration register,                                 Address offset: 0x3C */
  __IO uint32_t TAFCR;   /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
  __IO uint32_t ALRMASSR;/*!< RTC alarm A sub second register,                          Address offset: 0x44 */
  __IO uint32_t ALRMBSSR;/*!< RTC alarm B sub second register,                          Address offset: 0x48 */
  uint32_t RESERVED7;    /*!< Reserved, 0x4C                                                                 */
  __IO uint32_t BKP0R;   /*!< RTC backup register 1,                                    Address offset: 0x50 */
  __IO uint32_t BKP1R;   /*!< RTC backup register 1,                                    Address offset: 0x54 */
  __IO uint32_t BKP2R;   /*!< RTC backup register 2,                                    Address offset: 0x58 */
  __IO uint32_t BKP3R;   /*!< RTC backup register 3,                                    Address offset: 0x5C */
  __IO uint32_t BKP4R;   /*!< RTC backup register 4,                                    Address offset: 0x60 */
  __IO uint32_t BKP5R;   /*!< RTC backup register 5,                                    Address offset: 0x64 */
  __IO uint32_t BKP6R;   /*!< RTC backup register 6,                                    Address offset: 0x68 */
  __IO uint32_t BKP7R;   /*!< RTC backup register 7,                                    Address offset: 0x6C */
  __IO uint32_t BKP8R;   /*!< RTC backup register 8,                                    Address offset: 0x70 */
  __IO uint32_t BKP9R;   /*!< RTC backup register 9,                                    Address offset: 0x74 */
  __IO uint32_t BKP10R;  /*!< RTC backup register 10,                                   Address offset: 0x78 */
  __IO uint32_t BKP11R;  /*!< RTC backup register 11,                                   Address offset: 0x7C */
  __IO uint32_t BKP12R;  /*!< RTC backup register 12,                                   Address offset: 0x80 */
  __IO uint32_t BKP13R;  /*!< RTC backup register 13,                                   Address offset: 0x84 */
  __IO uint32_t BKP14R;  /*!< RTC backup register 14,                                   Address offset: 0x88 */
  __IO uint32_t BKP15R;  /*!< RTC backup register 15,                                   Address offset: 0x8C */
  __IO uint32_t BKP16R;  /*!< RTC backup register 16,                                   Address offset: 0x90 */
  __IO uint32_t BKP17R;  /*!< RTC backup register 17,                                   Address offset: 0x94 */
  __IO uint32_t BKP18R;  /*!< RTC backup register 18,                                   Address offset: 0x98 */
  __IO uint32_t BKP19R;  /*!< RTC backup register 19,                                   Address offset: 0x9C */
} RTC_TypeDef;


/**
  * @brief SD host Interface
  */

typedef struct
{
  __IO uint32_t POWER;          /*!< SDIO power control register,    Address offset: 0x00 */
  __IO uint32_t CLKCR;          /*!< SDI clock control register,     Address offset: 0x04 */
  __IO uint32_t ARG;            /*!< SDIO argument register,         Address offset: 0x08 */
  __IO uint32_t CMD;            /*!< SDIO command register,          Address offset: 0x0C */
  __I uint32_t  RESPCMD;        /*!< SDIO command response register, Address offset: 0x10 */
  __I uint32_t  RESP1;          /*!< SDIO response 1 register,       Address offset: 0x14 */
  __I uint32_t  RESP2;          /*!< SDIO response 2 register,       Address offset: 0x18 */
  __I uint32_t  RESP3;          /*!< SDIO response 3 register,       Address offset: 0x1C */
  __I uint32_t  RESP4;          /*!< SDIO response 4 register,       Address offset: 0x20 */
  __IO uint32_t DTIMER;         /*!< SDIO data timer register,       Address offset: 0x24 */
  __IO uint32_t DLEN;           /*!< SDIO data length register,      Address offset: 0x28 */
  __IO uint32_t DCTRL;          /*!< SDIO data control register,     Address offset: 0x2C */
  __I uint32_t  DCOUNT;         /*!< SDIO data counter register,     Address offset: 0x30 */
  __I uint32_t  STA;            /*!< SDIO status register,           Address offset: 0x34 */
  __IO uint32_t ICR;            /*!< SDIO interrupt clear register,  Address offset: 0x38 */
  __IO uint32_t MASK;           /*!< SDIO mask register,             Address offset: 0x3C */
  uint32_t      RESERVED0[2];   /*!< Reserved, 0x40-0x44                                  */
  __I uint32_t  FIFOCNT;        /*!< SDIO FIFO counter register,     Address offset: 0x48 */
  uint32_t      RESERVED1[13];  /*!< Reserved, 0x4C-0x7C                                  */
  __IO uint32_t FIFO;           /*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_TypeDef;

/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  __IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;

/**
  * @brief TIM
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  __IO uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  __IO uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  __IO uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  __IO uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  __IO uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  __IO uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  __IO uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  __IO uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;

/**
  * @brief Window WATCHDOG
  */

typedef struct
{
  __IO uint32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;


/**
  * @brief __USB_OTG_Core_register
  */
typedef struct
{
  __IO uint32_t GOTGCTL;              /*!<  USB_OTG Control and Status Register    Address offset : 0x00      */
  __IO uint32_t GOTGINT;              /*!<  USB_OTG Interrupt Register             Address offset : 0x04      */
  __IO uint32_t GAHBCFG;              /*!<  Core AHB Configuration Register        Address offset : 0x08      */
  __IO uint32_t GUSBCFG;              /*!<  Core USB Configuration Register        Address offset : 0x0C      */
  __IO uint32_t GRSTCTL;              /*!<  Core Reset Register                    Address offset : 0x10      */
  __IO uint32_t GINTSTS;              /*!<  Core Interrupt Register                Address offset : 0x14      */
  __IO uint32_t GINTMSK;              /*!<  Core Interrupt Mask Register           Address offset : 0x18      */
  __IO uint32_t GRXSTSR;              /*!<  Receive Sts Q Read Register            Address offset : 0x1C      */
  __IO uint32_t GRXSTSP;              /*!<  Receive Sts Q Read & POP Register      Address offset : 0x20      */
  __IO uint32_t GRXFSIZ;              /* Receive FIFO Size Register                Address offset : 0x24      */
  __IO uint32_t DIEPTXF0_HNPTXFSIZ;   /*!<  EP0 / Non Periodic Tx FIFO Size Register Address offset : 0x28    */
  __IO uint32_t HNPTXSTS;             /*!<  Non Periodic Tx FIFO/Queue Sts reg     Address offset : 0x2C      */
  uint32_t Reserved30[2];             /* Reserved                                  Address offset : 0x30      */
  __IO uint32_t GCCFG;                /*!<  General Purpose IO Register            Address offset : 0x38      */
  __IO uint32_t CID;                  /*!< User ID Register                          Address offset : 0x3C      */
  uint32_t  Reserved40[48];           /*!< Reserved                                  Address offset : 0x40-0xFF */
  __IO uint32_t HPTXFSIZ;             /*!< Host Periodic Tx FIFO Size Reg            Address offset : 0x100 */
  __IO uint32_t DIEPTXF[0x0F];        /*!< dev Periodic Transmit FIFO */
}
USB_OTG_GlobalTypeDef;



/**
  * @brief __device_Registers
  */
typedef struct
{
  __IO uint32_t DCFG;         /*!< dev Configuration Register   Address offset : 0x800 */
  __IO uint32_t DCTL;         /*!< dev Control Register         Address offset : 0x804 */
  __IO uint32_t DSTS;         /*!< dev Status Register (RO)     Address offset : 0x808 */
  uint32_t Reserved0C;        /*!< Reserved                     Address offset : 0x80C */
  __IO uint32_t DIEPMSK;      /* !< dev IN Endpoint Mask        Address offset : 0x810 */
  __IO uint32_t DOEPMSK;      /*!< dev OUT Endpoint Mask        Address offset : 0x814 */
  __IO uint32_t DAINT;        /*!< dev All Endpoints Itr Reg    Address offset : 0x818 */
  __IO uint32_t DAINTMSK;     /*!< dev All Endpoints Itr Mask   Address offset : 0x81C */
  uint32_t  Reserved20;       /*!< Reserved                     Address offset : 0x820 */
  uint32_t Reserved9;         /*!< Reserved                     Address offset : 0x824 */
  __IO uint32_t DVBUSDIS;     /*!< dev VBUS discharge Register  Address offset : 0x828 */
  __IO uint32_t DVBUSPULSE;   /*!< dev VBUS Pulse Register      Address offset : 0x82C */
  __IO uint32_t DTHRCTL;      /*!< dev thr                      Address offset : 0x830 */
  __IO uint32_t DIEPEMPMSK;   /*!< dev empty msk                Address offset : 0x834 */
  __IO uint32_t DEACHINT;     /*!< dedicated EP interrupt       Address offset : 0x838 */
  __IO uint32_t DEACHMSK;     /*!< dedicated EP msk             Address offset : 0x83C */
  uint32_t Reserved40;        /*!< dedicated EP mask            Address offset : 0x840 */
  __IO uint32_t DINEP1MSK;    /*!< dedicated EP mask            Address offset : 0x844 */
  uint32_t  Reserved44[15];   /*!< Reserved                     Address offset : 0x844-0x87C */
  __IO uint32_t DOUTEP1MSK;   /*!< dedicated EP msk             Address offset : 0x884 */
}
USB_OTG_DeviceTypeDef;


/**
  * @brief __IN_Endpoint-Specific_Register
  */
typedef struct
{
  __IO uint32_t DIEPCTL;        /* dev IN Endpoint Control Reg 900h + (ep_num * 20h) + 00h     */
  uint32_t Reserved04;          /* Reserved                       900h + (ep_num * 20h) + 04h  */
  __IO uint32_t DIEPINT;        /* dev IN Endpoint Itr Reg     900h + (ep_num * 20h) + 08h     */
  uint32_t Reserved0C;          /* Reserved                       900h + (ep_num * 20h) + 0Ch  */
  __IO uint32_t DIEPTSIZ;       /* IN Endpoint Txfer Size   900h + (ep_num * 20h) + 10h        */
  __IO uint32_t DIEPDMA;        /* IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h  */
  __IO uint32_t DTXFSTS;        /*IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h   */
  uint32_t Reserved18;           /* Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch */
}
USB_OTG_INEndpointTypeDef;


/**
  * @brief __OUT_Endpoint-Specific_Registers
  */
typedef struct
{
  __IO uint32_t DOEPCTL;       /* dev OUT Endpoint Control Reg  B00h + (ep_num * 20h) + 00h*/
  uint32_t Reserved04;         /* Reserved                      B00h + (ep_num * 20h) + 04h*/
  __IO uint32_t DOEPINT;       /* dev OUT Endpoint Itr Reg      B00h + (ep_num * 20h) + 08h*/
  uint32_t Reserved0C;         /* Reserved                      B00h + (ep_num * 20h) + 0Ch*/
  __IO uint32_t DOEPTSIZ;      /* dev OUT Endpoint Txfer Size   B00h + (ep_num * 20h) + 10h*/
  __IO uint32_t DOEPDMA;       /* dev OUT Endpoint DMA Address  B00h + (ep_num * 20h) + 14h*/
  uint32_t Reserved18[2];      /* Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch*/
}
USB_OTG_OUTEndpointTypeDef;


/**
  * @brief __Host_Mode_Register_Structures
  */
typedef struct
{
  __IO uint32_t HCFG;             /* Host Configuration Register    400h*/
  __IO uint32_t HFIR;             /* Host Frame Interval Register   404h*/
  __IO uint32_t HFNUM;            /* Host Frame Nbr/Frame Remaining 408h*/
  uint32_t Reserved40C;           /* Reserved                       40Ch*/
  __IO uint32_t HPTXSTS;          /* Host Periodic Tx FIFO/ Queue Status 410h*/
  __IO uint32_t HAINT;            /* Host All Channels Interrupt Register 414h*/
  __IO uint32_t HAINTMSK;         /* Host All Channels Interrupt Mask 418h*/
}
USB_OTG_HostTypeDef;


/**
  * @brief __Host_Channel_Specific_Registers
  */
typedef struct
{
  __IO uint32_t HCCHAR;
  __IO uint32_t HCSPLT;
  __IO uint32_t HCINT;
  __IO uint32_t HCINTMSK;
  __IO uint32_t HCTSIZ;
  __IO uint32_t HCDMA;
  uint32_t Reserved[2];
}
USB_OTG_HostChannelTypeDef;


/**
  * @brief Peripheral_memory_map
  */
#define FLASH_BASE            (0x08000000UL) /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define CCMDATARAM_BASE       (0x10000000UL) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define SRAM1_BASE            (0x20000000UL) /*!< SRAM1(112 KB) base address in the alias region                             */
#define SRAM2_BASE            (0x2001C000UL) /*!< SRAM2(16 KB) base address in the alias region                              */
#define PERIPH_BASE           (0x40000000UL) /*!< Peripheral base address in the alias region                                */
#define BKPSRAM_BASE          (0x40024000UL) /*!< Backup SRAM(4 KB) base address in the alias region                         */
#define SRAM1_BB_BASE         (0x22000000UL) /*!< SRAM1(112 KB) base address in the bit-band region                             */
#define SRAM2_BB_BASE         (0x22380000UL) /*!< SRAM2(16 KB) base address in the bit-band region                              */
#define PERIPH_BB_BASE        (0x42000000UL) /*!< Peripheral base address in the bit-band region                                */
#define BKPSRAM_BB_BASE       (0x42480000UL) /*!< Backup SRAM(4 KB) base address in the bit-band region                         */
#define FLASH_END             (0x0807FFFFUL) /*!< FLASH end address */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE


/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000)
#define ADC_BASE              (APB2PERIPH_BASE + 0x2300)
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800)
#define SPI5_BASE             (APB2PERIPH_BASE + 0x5000)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8)

/* Debug MCU registers base address */
#define DBGMCU_BASE           (0xE0042000UL)

/*!< USB registers base address */
#define USB_OTG_FS_PERIPH_BASE               (0x50000000UL)

#define USB_OTG_GLOBAL_BASE                  (0x000UL)
#define USB_OTG_DEVICE_BASE                  (0x800UL)
#define USB_OTG_IN_ENDPOINT_BASE             (0x900UL)
#define USB_OTG_OUT_ENDPOINT_BASE            (0xB00UL)
#define USB_OTG_EP_REG_SIZE                  (0x20UL)
#define USB_OTG_HOST_BASE                    (0x400UL)
#define USB_OTG_HOST_PORT_BASE               (0x440UL)
#define USB_OTG_HOST_CHANNEL_BASE            (0x500UL)
#define USB_OTG_HOST_CHANNEL_SIZE            (0x20UL)
#define USB_OTG_PCGCCTL_BASE                 (0xE00UL)
#define USB_OTG_FIFO_BASE                    (0x1000UL)
#define USB_OTG_FIFO_SIZE                    (0x1000UL)

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define I2S3ext             ((SPI_TypeDef *) I2S3ext_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define ADC                 ((ADC_Common_TypeDef *) ADC_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI4                ((SPI_TypeDef *) SPI4_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define SPI5                ((SPI_TypeDef *) SPI5_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_TypeDef *) DMA2_Stream7_BASE)

#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for ADC_SR register  ********************/
#define  ADC_SR_AWD                          (0x00000001UL)       /*!<Analog watchdog flag */
#define  ADC_SR_EOC                          (0x00000002UL)       /*!<End of conversion */
#define  ADC_SR_JEOC                         (0x00000004UL)       /*!<Injected channel end of conversion */
#define  ADC_SR_JSTRT                        (0x00000008UL)       /*!<Injected channel Start flag */
#define  ADC_SR_STRT                         (0x00000010UL)       /*!<Regular channel Start flag */
#define  ADC_SR_OVR                          (0x00000020UL)       /*!<Overrun flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       (0x0000001FUL)        /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     (0x00000001UL)        /*!<Bit 0 */
#define  ADC_CR1_AWDCH_1                     (0x00000002UL)        /*!<Bit 1 */
#define  ADC_CR1_AWDCH_2                     (0x00000004UL)        /*!<Bit 2 */
#define  ADC_CR1_AWDCH_3                     (0x00000008UL)        /*!<Bit 3 */
#define  ADC_CR1_AWDCH_4                     (0x00000010UL)        /*!<Bit 4 */
#define  ADC_CR1_EOCIE                       (0x00000020UL)        /*!<Interrupt enable for EOC */
#define  ADC_CR1_AWDIE                       (0x00000040UL)        /*!<AAnalog Watchdog interrupt enable */
#define  ADC_CR1_JEOCIE                      (0x00000080UL)        /*!<Interrupt enable for injected channels */
#define  ADC_CR1_SCAN                        (0x00000100UL)        /*!<Scan mode */
#define  ADC_CR1_AWDSGL                      (0x00000200UL)        /*!<Enable the watchdog on a single channel in scan mode */
#define  ADC_CR1_JAUTO                       (0x00000400UL)        /*!<Automatic injected group conversion */
#define  ADC_CR1_DISCEN                      (0x00000800UL)        /*!<Discontinuous mode on regular channels */
#define  ADC_CR1_JDISCEN                     (0x00001000UL)        /*!<Discontinuous mode on injected channels */
#define  ADC_CR1_DISCNUM                     (0x0000E000UL)        /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CR1_DISCNUM_0                   (0x00002000UL)        /*!<Bit 0 */
#define  ADC_CR1_DISCNUM_1                   (0x00004000UL)        /*!<Bit 1 */
#define  ADC_CR1_DISCNUM_2                   (0x00008000UL)        /*!<Bit 2 */
#define  ADC_CR1_JAWDEN                      (0x00400000UL)        /*!<Analog watchdog enable on injected channels */
#define  ADC_CR1_AWDEN                       (0x00800000UL)        /*!<Analog watchdog enable on regular channels */
#define  ADC_CR1_RES                         (0x03000000UL)        /*!<RES[2:0] bits (Resolution) */
#define  ADC_CR1_RES_0                       (0x01000000UL)        /*!<Bit 0 */
#define  ADC_CR1_RES_1                       (0x02000000UL)        /*!<Bit 1 */
#define  ADC_CR1_OVRIE                       (0x04000000UL)         /*!<overrun interrupt enable */

/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        (0x00000001UL)        /*!<A/D Converter ON / OFF */
#define  ADC_CR2_CONT                        (0x00000002UL)        /*!<Continuous Conversion */
#define  ADC_CR2_DMA                         (0x00000100UL)        /*!<Direct Memory access mode */
#define  ADC_CR2_DDS                         (0x00000200UL)        /*!<DMA disable selection (Single ADC) */
#define  ADC_CR2_EOCS                        (0x00000400UL)        /*!<End of conversion selection */
#define  ADC_CR2_ALIGN                       (0x00000800UL)        /*!<Data Alignment */
#define  ADC_CR2_JEXTSEL                     (0x000F0000UL)        /*!<JEXTSEL[3:0] bits (External event select for injected group) */
#define  ADC_CR2_JEXTSEL_0                   (0x00010000UL)        /*!<Bit 0 */
#define  ADC_CR2_JEXTSEL_1                   (0x00020000UL)        /*!<Bit 1 */
#define  ADC_CR2_JEXTSEL_2                   (0x00040000UL)        /*!<Bit 2 */
#define  ADC_CR2_JEXTSEL_3                   (0x00080000UL)        /*!<Bit 3 */
#define  ADC_CR2_JEXTEN                      (0x00300000UL)        /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
#define  ADC_CR2_JEXTEN_0                    (0x00100000UL)        /*!<Bit 0 */
#define  ADC_CR2_JEXTEN_1                    (0x00200000UL)        /*!<Bit 1 */
#define  ADC_CR2_JSWSTART                    (0x00400000UL)        /*!<Start Conversion of injected channels */
#define  ADC_CR2_EXTSEL                      (0x0F000000UL)        /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
#define  ADC_CR2_EXTSEL_0                    (0x01000000UL)        /*!<Bit 0 */
#define  ADC_CR2_EXTSEL_1                    (0x02000000UL)        /*!<Bit 1 */
#define  ADC_CR2_EXTSEL_2                    (0x04000000UL)        /*!<Bit 2 */
#define  ADC_CR2_EXTSEL_3                    (0x08000000UL)        /*!<Bit 3 */
#define  ADC_CR2_EXTEN                       (0x30000000UL)        /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
#define  ADC_CR2_EXTEN_0                     (0x10000000UL)        /*!<Bit 0 */
#define  ADC_CR2_EXTEN_1                     (0x20000000UL)        /*!<Bit 1 */
#define  ADC_CR2_SWSTART                     (0x40000000UL)        /*!<Start Conversion of regular channels */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  ADC_SMPR1_SMP10                     (0x00000007UL)        /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPR1_SMP10_0                   (0x00000001UL)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP10_1                   (0x00000002UL)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP10_2                   (0x00000004UL)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP11                     (0x00000038UL)        /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPR1_SMP11_0                   (0x00000008UL)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP11_1                   (0x00000010UL)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP11_2                   (0x00000020UL)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP12                     (0x000001C0UL)        /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPR1_SMP12_0                   (0x00000040UL)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP12_1                   (0x00000080UL)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP12_2                   (0x00000100UL)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP13                     (0x00000E00UL)        /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPR1_SMP13_0                   (0x00000200UL)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP13_1                   (0x00000400UL)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP13_2                   (0x00000800UL)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP14                     (0x00007000UL)        /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPR1_SMP14_0                   (0x00001000UL)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP14_1                   (0x00002000UL)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP14_2                   (0x00004000UL)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP15                     (0x00038000UL)        /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPR1_SMP15_0                   (0x00008000UL)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP15_1                   (0x00010000UL)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP15_2                   (0x00020000UL)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP16                     (0x001C0000UL)        /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPR1_SMP16_0                   (0x00040000UL)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP16_1                   (0x00080000UL)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP16_2                   (0x00100000UL)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP17                     (0x00E00000UL)        /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPR1_SMP17_0                   (0x00200000UL)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP17_1                   (0x00400000UL)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP17_2                   (0x00800000UL)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP18                     (0x07000000UL)        /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
#define  ADC_SMPR1_SMP18_0                   (0x01000000UL)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP18_1                   (0x02000000UL)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP18_2                   (0x04000000UL)        /*!<Bit 2 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  ADC_SMPR2_SMP0                      (0x00000007UL)        /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPR2_SMP0_0                    (0x00000001UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP0_1                    (0x00000002UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP0_2                    (0x00000004UL)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP1                      (0x00000038UL)        /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPR2_SMP1_0                    (0x00000008UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP1_1                    (0x00000010UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP1_2                    (0x00000020UL)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP2                      (0x000001C0UL)        /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMPR2_SMP2_0                    (0x00000040UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP2_1                    (0x00000080UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP2_2                    (0x00000100UL)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP3                      (0x00000E00UL)        /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMPR2_SMP3_0                    (0x00000200UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP3_1                    (0x00000400UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP3_2                    (0x00000800UL)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP4                      (0x00007000UL)        /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMPR2_SMP4_0                    (0x00001000UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP4_1                    (0x00002000UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP4_2                    (0x00004000UL)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP5                      (0x00038000UL)        /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMPR2_SMP5_0                    (0x00008000UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP5_1                    (0x00010000UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP5_2                    (0x00020000UL)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP6                      (0x001C0000UL)        /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMPR2_SMP6_0                    (0x00040000UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP6_1                    (0x00080000UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP6_2                    (0x00100000UL)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP7                      (0x00E00000UL)        /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMPR2_SMP7_0                    (0x00200000UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP7_1                    (0x00400000UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP7_2                    (0x00800000UL)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP8                      (0x07000000UL)        /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMPR2_SMP8_0                    (0x01000000UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP8_1                    (0x02000000UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP8_2                    (0x04000000UL)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP9                      (0x38000000UL)        /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMPR2_SMP9_0                    (0x08000000UL)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP9_1                    (0x10000000UL)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP9_2                    (0x20000000UL)        /*!<Bit 2 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define  ADC_JOFR1_JOFFSET1                  (0x0FFFUL)            /*!<Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  (0x0FFFUL)            /*!<Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  (0x0FFFUL)            /*!<Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  (0x0FFFUL)            /*!<Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          (0x0FFFUL)            /*!<Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          (0x0FFFUL)            /*!<Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define  ADC_SQR1_SQ13                       (0x0000001FUL)        /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_SQR1_SQ13_0                     (0x00000001UL)        /*!<Bit 0 */
#define  ADC_SQR1_SQ13_1                     (0x00000002UL)        /*!<Bit 1 */
#define  ADC_SQR1_SQ13_2                     (0x00000004UL)        /*!<Bit 2 */
#define  ADC_SQR1_SQ13_3                     (0x00000008UL)        /*!<Bit 3 */
#define  ADC_SQR1_SQ13_4                     (0x00000010UL)        /*!<Bit 4 */
#define  ADC_SQR1_SQ14                       (0x000003E0UL)        /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_SQR1_SQ14_0                     (0x00000020UL)        /*!<Bit 0 */
#define  ADC_SQR1_SQ14_1                     (0x00000040UL)        /*!<Bit 1 */
#define  ADC_SQR1_SQ14_2                     (0x00000080UL)        /*!<Bit 2 */
#define  ADC_SQR1_SQ14_3                     (0x00000100UL)        /*!<Bit 3 */
#define  ADC_SQR1_SQ14_4                     (0x00000200UL)        /*!<Bit 4 */
#define  ADC_SQR1_SQ15                       (0x00007C00UL)        /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_SQR1_SQ15_0                     (0x00000400UL)        /*!<Bit 0 */
#define  ADC_SQR1_SQ15_1                     (0x00000800UL)        /*!<Bit 1 */
#define  ADC_SQR1_SQ15_2                     (0x00001000UL)        /*!<Bit 2 */
#define  ADC_SQR1_SQ15_3                     (0x00002000UL)        /*!<Bit 3 */
#define  ADC_SQR1_SQ15_4                     (0x00004000UL)        /*!<Bit 4 */
#define  ADC_SQR1_SQ16                       (0x000F8000UL)        /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_SQR1_SQ16_0                     (0x00008000UL)        /*!<Bit 0 */
#define  ADC_SQR1_SQ16_1                     (0x00010000UL)        /*!<Bit 1 */
#define  ADC_SQR1_SQ16_2                     (0x00020000UL)        /*!<Bit 2 */
#define  ADC_SQR1_SQ16_3                     (0x00040000UL)        /*!<Bit 3 */
#define  ADC_SQR1_SQ16_4                     (0x00080000UL)        /*!<Bit 4 */
#define  ADC_SQR1_L                          (0x00F00000UL)        /*!<L[3:0] bits (Regular channel sequence length) */
#define  ADC_SQR1_L_0                        (0x00100000UL)        /*!<Bit 0 */
#define  ADC_SQR1_L_1                        (0x00200000UL)        /*!<Bit 1 */
#define  ADC_SQR1_L_2                        (0x00400000UL)        /*!<Bit 2 */
#define  ADC_SQR1_L_3                        (0x00800000UL)        /*!<Bit 3 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define  ADC_SQR2_SQ7                        (0x0000001FUL)        /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_SQR2_SQ7_0                      (0x00000001UL)        /*!<Bit 0 */
#define  ADC_SQR2_SQ7_1                      (0x00000002UL)        /*!<Bit 1 */
#define  ADC_SQR2_SQ7_2                      (0x00000004UL)        /*!<Bit 2 */
#define  ADC_SQR2_SQ7_3                      (0x00000008UL)        /*!<Bit 3 */
#define  ADC_SQR2_SQ7_4                      (0x00000010UL)        /*!<Bit 4 */
#define  ADC_SQR2_SQ8                        (0x000003E0UL)        /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_SQR2_SQ8_0                      (0x00000020UL)        /*!<Bit 0 */
#define  ADC_SQR2_SQ8_1                      (0x00000040UL)        /*!<Bit 1 */
#define  ADC_SQR2_SQ8_2                      (0x00000080UL)        /*!<Bit 2 */
#define  ADC_SQR2_SQ8_3                      (0x00000100UL)        /*!<Bit 3 */
#define  ADC_SQR2_SQ8_4                      (0x00000200UL)        /*!<Bit 4 */
#define  ADC_SQR2_SQ9                        (0x00007C00UL)        /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_SQR2_SQ9_0                      (0x00000400UL)        /*!<Bit 0 */
#define  ADC_SQR2_SQ9_1                      (0x00000800UL)        /*!<Bit 1 */
#define  ADC_SQR2_SQ9_2                      (0x00001000UL)        /*!<Bit 2 */
#define  ADC_SQR2_SQ9_3                      (0x00002000UL)        /*!<Bit 3 */
#define  ADC_SQR2_SQ9_4                      (0x00004000UL)        /*!<Bit 4 */
#define  ADC_SQR2_SQ10                       (0x000F8000UL)        /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_SQR2_SQ10_0                     (0x00008000UL)        /*!<Bit 0 */
#define  ADC_SQR2_SQ10_1                     (0x00010000UL)        /*!<Bit 1 */
#define  ADC_SQR2_SQ10_2                     (0x00020000UL)        /*!<Bit 2 */
#define  ADC_SQR2_SQ10_3                     (0x00040000UL)        /*!<Bit 3 */
#define  ADC_SQR2_SQ10_4                     (0x00080000UL)        /*!<Bit 4 */
#define  ADC_SQR2_SQ11                       (0x01F00000UL)        /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_SQR2_SQ11_0                     (0x00100000UL)        /*!<Bit 0 */
#define  ADC_SQR2_SQ11_1                     (0x00200000UL)        /*!<Bit 1 */
#define  ADC_SQR2_SQ11_2                     (0x00400000UL)        /*!<Bit 2 */
#define  ADC_SQR2_SQ11_3                     (0x00800000UL)        /*!<Bit 3 */
#define  ADC_SQR2_SQ11_4                     (0x01000000UL)        /*!<Bit 4 */
#define  ADC_SQR2_SQ12                       (0x3E000000UL)        /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_SQR2_SQ12_0                     (0x02000000UL)        /*!<Bit 0 */
#define  ADC_SQR2_SQ12_1                     (0x04000000UL)        /*!<Bit 1 */
#define  ADC_SQR2_SQ12_2                     (0x08000000UL)        /*!<Bit 2 */
#define  ADC_SQR2_SQ12_3                     (0x10000000UL)        /*!<Bit 3 */
#define  ADC_SQR2_SQ12_4                     (0x20000000UL)        /*!<Bit 4 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define  ADC_SQR3_SQ1                        (0x0000001FUL)        /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_SQR3_SQ1_0                      (0x00000001UL)        /*!<Bit 0 */
#define  ADC_SQR3_SQ1_1                      (0x00000002UL)        /*!<Bit 1 */
#define  ADC_SQR3_SQ1_2                      (0x00000004UL)        /*!<Bit 2 */
#define  ADC_SQR3_SQ1_3                      (0x00000008UL)        /*!<Bit 3 */
#define  ADC_SQR3_SQ1_4                      (0x00000010UL)        /*!<Bit 4 */
#define  ADC_SQR3_SQ2                        (0x000003E0UL)        /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_SQR3_SQ2_0                      (0x00000020UL)        /*!<Bit 0 */
#define  ADC_SQR3_SQ2_1                      (0x00000040UL)        /*!<Bit 1 */
#define  ADC_SQR3_SQ2_2                      (0x00000080UL)        /*!<Bit 2 */
#define  ADC_SQR3_SQ2_3                      (0x00000100UL)        /*!<Bit 3 */
#define  ADC_SQR3_SQ2_4                      (0x00000200UL)        /*!<Bit 4 */
#define  ADC_SQR3_SQ3                        (0x00007C00UL)        /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_SQR3_SQ3_0                      (0x00000400UL)        /*!<Bit 0 */
#define  ADC_SQR3_SQ3_1                      (0x00000800UL)        /*!<Bit 1 */
#define  ADC_SQR3_SQ3_2                      (0x00001000UL)        /*!<Bit 2 */
#define  ADC_SQR3_SQ3_3                      (0x00002000UL)        /*!<Bit 3 */
#define  ADC_SQR3_SQ3_4                      (0x00004000UL)        /*!<Bit 4 */
#define  ADC_SQR3_SQ4                        (0x000F8000UL)        /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_SQR3_SQ4_0                      (0x00008000UL)        /*!<Bit 0 */
#define  ADC_SQR3_SQ4_1                      (0x00010000UL)        /*!<Bit 1 */
#define  ADC_SQR3_SQ4_2                      (0x00020000UL)        /*!<Bit 2 */
#define  ADC_SQR3_SQ4_3                      (0x00040000UL)        /*!<Bit 3 */
#define  ADC_SQR3_SQ4_4                      (0x00080000UL)        /*!<Bit 4 */
#define  ADC_SQR3_SQ5                        (0x01F00000UL)        /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_SQR3_SQ5_0                      (0x00100000UL)        /*!<Bit 0 */
#define  ADC_SQR3_SQ5_1                      (0x00200000UL)        /*!<Bit 1 */
#define  ADC_SQR3_SQ5_2                      (0x00400000UL)        /*!<Bit 2 */
#define  ADC_SQR3_SQ5_3                      (0x00800000UL)        /*!<Bit 3 */
#define  ADC_SQR3_SQ5_4                      (0x01000000UL)        /*!<Bit 4 */
#define  ADC_SQR3_SQ6                        (0x3E000000UL)        /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_SQR3_SQ6_0                      (0x02000000UL)        /*!<Bit 0 */
#define  ADC_SQR3_SQ6_1                      (0x04000000UL)        /*!<Bit 1 */
#define  ADC_SQR3_SQ6_2                      (0x08000000UL)        /*!<Bit 2 */
#define  ADC_SQR3_SQ6_3                      (0x10000000UL)        /*!<Bit 3 */
#define  ADC_SQR3_SQ6_4                      (0x20000000UL)        /*!<Bit 4 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define  ADC_JSQR_JSQ1                       (0x0000001FUL)        /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */
#define  ADC_JSQR_JSQ1_0                     (0x00000001UL)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ1_1                     (0x00000002UL)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ1_2                     (0x00000004UL)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ1_3                     (0x00000008UL)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ1_4                     (0x00000010UL)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ2                       (0x000003E0UL)        /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQR_JSQ2_0                     (0x00000020UL)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ2_1                     (0x00000040UL)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ2_2                     (0x00000080UL)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ2_3                     (0x00000100UL)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ2_4                     (0x00000200UL)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ3                       (0x00007C00UL)        /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQR_JSQ3_0                     (0x00000400UL)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ3_1                     (0x00000800UL)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ3_2                     (0x00001000UL)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ3_3                     (0x00002000UL)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ3_4                     (0x00004000UL)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ4                       (0x000F8000UL)        /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQR_JSQ4_0                     (0x00008000UL)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ4_1                     (0x00010000UL)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ4_2                     (0x00020000UL)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ4_3                     (0x00040000UL)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ4_4                     (0x00080000UL)        /*!<Bit 4 */
#define  ADC_JSQR_JL                         (0x00300000UL)        /*!<JL[1:0] bits (Injected Sequence length) */
#define  ADC_JSQR_JL_0                       (0x00100000UL)        /*!<Bit 0 */
#define  ADC_JSQR_JL_1                       (0x00200000UL)        /*!<Bit 1 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define  ADC_JDR1_JDATA                      (0xFFFFUL)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      (0xFFFFUL)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      (0xFFFFUL)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      (0xFFFFUL)            /*!<Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         (0x0000FFFFUL)        /*!<Regular data */
#define  ADC_DR_ADC2DATA                     (0xFFFF0000UL)        /*!<ADC2 data */

/*******************  Bit definition for ADC_CSR register  ********************/
#define  ADC_CSR_AWD1                        (0x00000001UL)        /*!<ADC1 Analog watchdog flag */
#define  ADC_CSR_EOC1                        (0x00000002UL)        /*!<ADC1 End of conversion */
#define  ADC_CSR_JEOC1                       (0x00000004UL)        /*!<ADC1 Injected channel end of conversion */
#define  ADC_CSR_JSTRT1                      (0x00000008UL)        /*!<ADC1 Injected channel Start flag */
#define  ADC_CSR_STRT1                       (0x00000010UL)        /*!<ADC1 Regular channel Start flag */
#define  ADC_CSR_DOVR1                       (0x00000020UL)        /*!<ADC1 DMA overrun  flag */
#define  ADC_CSR_AWD2                        (0x00000100UL)        /*!<ADC2 Analog watchdog flag */
#define  ADC_CSR_EOC2                        (0x00000200UL)        /*!<ADC2 End of conversion */
#define  ADC_CSR_JEOC2                       (0x00000400UL)        /*!<ADC2 Injected channel end of conversion */
#define  ADC_CSR_JSTRT2                      (0x00000800UL)        /*!<ADC2 Injected channel Start flag */
#define  ADC_CSR_STRT2                       (0x00001000UL)        /*!<ADC2 Regular channel Start flag */
#define  ADC_CSR_DOVR2                       (0x00002000UL)        /*!<ADC2 DMA overrun  flag */
#define  ADC_CSR_AWD3                        (0x00010000UL)        /*!<ADC3 Analog watchdog flag */
#define  ADC_CSR_EOC3                        (0x00020000UL)        /*!<ADC3 End of conversion */
#define  ADC_CSR_JEOC3                       (0x00040000UL)        /*!<ADC3 Injected channel end of conversion */
#define  ADC_CSR_JSTRT3                      (0x00080000UL)        /*!<ADC3 Injected channel Start flag */
#define  ADC_CSR_STRT3                       (0x00100000UL)        /*!<ADC3 Regular channel Start flag */
#define  ADC_CSR_DOVR3                       (0x00200000UL)        /*!<ADC3 DMA overrun  flag */

/*******************  Bit definition for ADC_CCR register  ********************/
#define  ADC_CCR_MULTI                       (0x0000001FUL)        /*!<MULTI[4:0] bits (Multi-ADC mode selection) */
#define  ADC_CCR_MULTI_0                     (0x00000001UL)        /*!<Bit 0 */
#define  ADC_CCR_MULTI_1                     (0x00000002UL)        /*!<Bit 1 */
#define  ADC_CCR_MULTI_2                     (0x00000004UL)        /*!<Bit 2 */
#define  ADC_CCR_MULTI_3                     (0x00000008UL)        /*!<Bit 3 */
#define  ADC_CCR_MULTI_4                     (0x00000010UL)        /*!<Bit 4 */
#define  ADC_CCR_DELAY                       (0x00000F00UL)        /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */
#define  ADC_CCR_DELAY_0                     (0x00000100UL)        /*!<Bit 0 */
#define  ADC_CCR_DELAY_1                     (0x00000200UL)        /*!<Bit 1 */
#define  ADC_CCR_DELAY_2                     (0x00000400UL)        /*!<Bit 2 */
#define  ADC_CCR_DELAY_3                     (0x00000800UL)        /*!<Bit 3 */
#define  ADC_CCR_DDS                         (0x00002000UL)        /*!<DMA disable selection (Multi-ADC mode) */
#define  ADC_CCR_DMA                         (0x0000C000UL)        /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */
#define  ADC_CCR_DMA_0                       (0x00004000UL)        /*!<Bit 0 */
#define  ADC_CCR_DMA_1                       (0x00008000UL)        /*!<Bit 1 */
#define  ADC_CCR_ADCPRE                      (0x00030000UL)        /*!<ADCPRE[1:0] bits (ADC prescaler) */
#define  ADC_CCR_ADCPRE_0                    (0x00010000UL)        /*!<Bit 0 */
#define  ADC_CCR_ADCPRE_1                    (0x00020000UL)        /*!<Bit 1 */
#define  ADC_CCR_VBATE                       (0x00400000UL)        /*!<VBAT Enable */
#define  ADC_CCR_TSVREFE                     (0x00800000UL)        /*!<Temperature Sensor and VREFINT Enable */

/*******************  Bit definition for ADC_CDR register  ********************/
#define  ADC_CDR_DATA1                      (0x0000FFFFUL)         /*!<1st data of a pair of regular conversions */
#define  ADC_CDR_DATA2                      (0xFFFF0000UL)         /*!<2nd data of a pair of regular conversions */

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           (0xFFFFFFFFUL) /*!< Data register bits */


/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         (0xFFUL)        /*!< General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        (0x01UL)        /*!< RESET bit */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DMA_SxCR register  *****************/
#define DMA_SxCR_CHSEL                       (0x0E000000UL)
#define DMA_SxCR_CHSEL_0                     (0x02000000UL)
#define DMA_SxCR_CHSEL_1                     (0x04000000UL)
#define DMA_SxCR_CHSEL_2                     (0x08000000UL)
#define DMA_SxCR_MBURST                      (0x01800000UL)
#define DMA_SxCR_MBURST_0                    (0x00800000UL)
#define DMA_SxCR_MBURST_1                    (0x01000000UL)
#define DMA_SxCR_PBURST                      (0x00600000UL)
#define DMA_SxCR_PBURST_0                    (0x00200000UL)
#define DMA_SxCR_PBURST_1                    (0x00400000UL)
#define DMA_SxCR_ACK                         (0x00100000UL)
#define DMA_SxCR_CT                          (0x00080000UL)
#define DMA_SxCR_DBM                         (0x00040000UL)
#define DMA_SxCR_PL                          (0x00030000UL)
#define DMA_SxCR_PL_0                        (0x00010000UL)
#define DMA_SxCR_PL_1                        (0x00020000UL)
#define DMA_SxCR_PINCOS                      (0x00008000UL)
#define DMA_SxCR_MSIZE                       (0x00006000UL)
#define DMA_SxCR_MSIZE_0                     (0x00002000UL)
#define DMA_SxCR_MSIZE_1                     (0x00004000UL)
#define DMA_SxCR_PSIZE                       (0x00001800UL)
#define DMA_SxCR_PSIZE_0                     (0x00000800UL)
#define DMA_SxCR_PSIZE_1                     (0x00001000UL)
#define DMA_SxCR_MINC                        (0x00000400UL)
#define DMA_SxCR_PINC                        (0x00000200UL)
#define DMA_SxCR_CIRC                        (0x00000100UL)
#define DMA_SxCR_DIR                         (0x000000C0UL)
#define DMA_SxCR_DIR_0                       (0x00000040UL)
#define DMA_SxCR_DIR_1                       (0x00000080UL)
#define DMA_SxCR_PFCTRL                      (0x00000020UL)
#define DMA_SxCR_TCIE                        (0x00000010UL)
#define DMA_SxCR_HTIE                        (0x00000008UL)
#define DMA_SxCR_TEIE                        (0x00000004UL)
#define DMA_SxCR_DMEIE                       (0x00000002UL)
#define DMA_SxCR_EN                          (0x00000001UL)

/********************  Bits definition for DMA_SxCNDTR register  **************/
#define DMA_SxNDT                            (0x0000FFFFUL)
#define DMA_SxNDT_0                          (0x00000001UL)
#define DMA_SxNDT_1                          (0x00000002UL)
#define DMA_SxNDT_2                          (0x00000004UL)
#define DMA_SxNDT_3                          (0x00000008UL)
#define DMA_SxNDT_4                          (0x00000010UL)
#define DMA_SxNDT_5                          (0x00000020UL)
#define DMA_SxNDT_6                          (0x00000040UL)
#define DMA_SxNDT_7                          (0x00000080UL)
#define DMA_SxNDT_8                          (0x00000100UL)
#define DMA_SxNDT_9                          (0x00000200UL)
#define DMA_SxNDT_10                         (0x00000400UL)
#define DMA_SxNDT_11                         (0x00000800UL)
#define DMA_SxNDT_12                         (0x00001000UL)
#define DMA_SxNDT_13                         (0x00002000UL)
#define DMA_SxNDT_14                         (0x00004000UL)
#define DMA_SxNDT_15                         (0x00008000UL)

/********************  Bits definition for DMA_SxFCR register  ****************/
#define DMA_SxFCR_FEIE                       (0x00000080UL)
#define DMA_SxFCR_FS                         (0x00000038UL)
#define DMA_SxFCR_FS_0                       (0x00000008UL)
#define DMA_SxFCR_FS_1                       (0x00000010UL)
#define DMA_SxFCR_FS_2                       (0x00000020UL)
#define DMA_SxFCR_DMDIS                      (0x00000004UL)
#define DMA_SxFCR_FTH                        (0x00000003UL)
#define DMA_SxFCR_FTH_0                      (0x00000001UL)
#define DMA_SxFCR_FTH_1                      (0x00000002UL)

/********************  Bits definition for DMA_LISR register  *****************/
#define DMA_LISR_TCIF3                       (0x08000000UL)
#define DMA_LISR_HTIF3                       (0x04000000UL)
#define DMA_LISR_TEIF3                       (0x02000000UL)
#define DMA_LISR_DMEIF3                      (0x01000000UL)
#define DMA_LISR_FEIF3                       (0x00400000UL)
#define DMA_LISR_TCIF2                       (0x00200000UL)
#define DMA_LISR_HTIF2                       (0x00100000UL)
#define DMA_LISR_TEIF2                       (0x00080000UL)
#define DMA_LISR_DMEIF2                      (0x00040000UL)
#define DMA_LISR_FEIF2                       (0x00010000UL)
#define DMA_LISR_TCIF1                       (0x00000800UL)
#define DMA_LISR_HTIF1                       (0x00000400UL)
#define DMA_LISR_TEIF1                       (0x00000200UL)
#define DMA_LISR_DMEIF1                      (0x00000100UL)
#define DMA_LISR_FEIF1                       (0x00000040UL)
#define DMA_LISR_TCIF0                       (0x00000020UL)
#define DMA_LISR_HTIF0                       (0x00000010UL)
#define DMA_LISR_TEIF0                       (0x00000008UL)
#define DMA_LISR_DMEIF0                      (0x00000004UL)
#define DMA_LISR_FEIF0                       (0x00000001UL)

/********************  Bits definition for DMA_HISR register  *****************/
#define DMA_HISR_TCIF7                       (0x08000000UL)
#define DMA_HISR_HTIF7                       (0x04000000UL)
#define DMA_HISR_TEIF7                       (0x02000000UL)
#define DMA_HISR_DMEIF7                      (0x01000000UL)
#define DMA_HISR_FEIF7                       (0x00400000UL)
#define DMA_HISR_TCIF6                       (0x00200000UL)
#define DMA_HISR_HTIF6                       (0x00100000UL)
#define DMA_HISR_TEIF6                       (0x00080000UL)
#define DMA_HISR_DMEIF6                      (0x00040000UL)
#define DMA_HISR_FEIF6                       (0x00010000UL)
#define DMA_HISR_TCIF5                       (0x00000800UL)
#define DMA_HISR_HTIF5                       (0x00000400UL)
#define DMA_HISR_TEIF5                       (0x00000200UL)
#define DMA_HISR_DMEIF5                      (0x00000100UL)
#define DMA_HISR_FEIF5                       (0x00000040UL)
#define DMA_HISR_TCIF4                       (0x00000020UL)
#define DMA_HISR_HTIF4                       (0x00000010UL)
#define DMA_HISR_TEIF4                       (0x00000008UL)
#define DMA_HISR_DMEIF4                      (0x00000004UL)
#define DMA_HISR_FEIF4                       (0x00000001UL)

/********************  Bits definition for DMA_LIFCR register  ****************/
#define DMA_LIFCR_CTCIF3                     (0x08000000UL)
#define DMA_LIFCR_CHTIF3                     (0x04000000UL)
#define DMA_LIFCR_CTEIF3                     (0x02000000UL)
#define DMA_LIFCR_CDMEIF3                    (0x01000000UL)
#define DMA_LIFCR_CFEIF3                     (0x00400000UL)
#define DMA_LIFCR_CTCIF2                     (0x00200000UL)
#define DMA_LIFCR_CHTIF2                     (0x00100000UL)
#define DMA_LIFCR_CTEIF2                     (0x00080000UL)
#define DMA_LIFCR_CDMEIF2                    (0x00040000UL)
#define DMA_LIFCR_CFEIF2                     (0x00010000UL)
#define DMA_LIFCR_CTCIF1                     (0x00000800UL)
#define DMA_LIFCR_CHTIF1                     (0x00000400UL)
#define DMA_LIFCR_CTEIF1                     (0x00000200UL)
#define DMA_LIFCR_CDMEIF1                    (0x00000100UL)
#define DMA_LIFCR_CFEIF1                     (0x00000040UL)
#define DMA_LIFCR_CTCIF0                     (0x00000020UL)
#define DMA_LIFCR_CHTIF0                     (0x00000010UL)
#define DMA_LIFCR_CTEIF0                     (0x00000008UL)
#define DMA_LIFCR_CDMEIF0                    (0x00000004UL)
#define DMA_LIFCR_CFEIF0                     (0x00000001UL)

/********************  Bits definition for DMA_HIFCR  register  ****************/
#define DMA_HIFCR_CTCIF7                     (0x08000000UL)
#define DMA_HIFCR_CHTIF7                     (0x04000000UL)
#define DMA_HIFCR_CTEIF7                     (0x02000000UL)
#define DMA_HIFCR_CDMEIF7                    (0x01000000UL)
#define DMA_HIFCR_CFEIF7                     (0x00400000UL)
#define DMA_HIFCR_CTCIF6                     (0x00200000UL)
#define DMA_HIFCR_CHTIF6                     (0x00100000UL)
#define DMA_HIFCR_CTEIF6                     (0x00080000UL)
#define DMA_HIFCR_CDMEIF6                    (0x00040000UL)
#define DMA_HIFCR_CFEIF6                     (0x00010000UL)
#define DMA_HIFCR_CTCIF5                     (0x00000800UL)
#define DMA_HIFCR_CHTIF5                     (0x00000400UL)
#define DMA_HIFCR_CTEIF5                     (0x00000200UL)
#define DMA_HIFCR_CDMEIF5                    (0x00000100UL)
#define DMA_HIFCR_CFEIF5                     (0x00000040UL)
#define DMA_HIFCR_CTCIF4                     (0x00000020UL)
#define DMA_HIFCR_CHTIF4                     (0x00000010UL)
#define DMA_HIFCR_CTEIF4                     (0x00000008UL)
#define DMA_HIFCR_CDMEIF4                    (0x00000004UL)
#define DMA_HIFCR_CFEIF4                     (0x00000001UL)


/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define  EXTI_IMR_MR0                        (0x00000001UL)        /*!< Interrupt Mask on line 0 */
#define  EXTI_IMR_MR1                        (0x00000002UL)        /*!< Interrupt Mask on line 1 */
#define  EXTI_IMR_MR2                        (0x00000004UL)        /*!< Interrupt Mask on line 2 */
#define  EXTI_IMR_MR3                        (0x00000008UL)        /*!< Interrupt Mask on line 3 */
#define  EXTI_IMR_MR4                        (0x00000010UL)        /*!< Interrupt Mask on line 4 */
#define  EXTI_IMR_MR5                        (0x00000020UL)        /*!< Interrupt Mask on line 5 */
#define  EXTI_IMR_MR6                        (0x00000040UL)        /*!< Interrupt Mask on line 6 */
#define  EXTI_IMR_MR7                        (0x00000080UL)        /*!< Interrupt Mask on line 7 */
#define  EXTI_IMR_MR8                        (0x00000100UL)        /*!< Interrupt Mask on line 8 */
#define  EXTI_IMR_MR9                        (0x00000200UL)        /*!< Interrupt Mask on line 9 */
#define  EXTI_IMR_MR10                       (0x00000400UL)        /*!< Interrupt Mask on line 10 */
#define  EXTI_IMR_MR11                       (0x00000800UL)        /*!< Interrupt Mask on line 11 */
#define  EXTI_IMR_MR12                       (0x00001000UL)        /*!< Interrupt Mask on line 12 */
#define  EXTI_IMR_MR13                       (0x00002000UL)        /*!< Interrupt Mask on line 13 */
#define  EXTI_IMR_MR14                       (0x00004000UL)        /*!< Interrupt Mask on line 14 */
#define  EXTI_IMR_MR15                       (0x00008000UL)        /*!< Interrupt Mask on line 15 */
#define  EXTI_IMR_MR16                       (0x00010000UL)        /*!< Interrupt Mask on line 16 */
#define  EXTI_IMR_MR17                       (0x00020000UL)        /*!< Interrupt Mask on line 17 */
#define  EXTI_IMR_MR18                       (0x00040000UL)        /*!< Interrupt Mask on line 18 */
#define  EXTI_IMR_MR19                       (0x00080000UL)        /*!< Interrupt Mask on line 19 */
#define  EXTI_IMR_MR20                       (0x00100000UL)        /*!< Interrupt Mask on line 20 */
#define  EXTI_IMR_MR21                       (0x00200000UL)        /*!< Interrupt Mask on line 21 */
#define  EXTI_IMR_MR22                       (0x00400000UL)        /*!< Interrupt Mask on line 22 */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define  EXTI_EMR_MR0                        (0x00000001UL)        /*!< Event Mask on line 0 */
#define  EXTI_EMR_MR1                        (0x00000002UL)        /*!< Event Mask on line 1 */
#define  EXTI_EMR_MR2                        (0x00000004UL)        /*!< Event Mask on line 2 */
#define  EXTI_EMR_MR3                        (0x00000008UL)        /*!< Event Mask on line 3 */
#define  EXTI_EMR_MR4                        (0x00000010UL)        /*!< Event Mask on line 4 */
#define  EXTI_EMR_MR5                        (0x00000020UL)        /*!< Event Mask on line 5 */
#define  EXTI_EMR_MR6                        (0x00000040UL)        /*!< Event Mask on line 6 */
#define  EXTI_EMR_MR7                        (0x00000080UL)        /*!< Event Mask on line 7 */
#define  EXTI_EMR_MR8                        (0x00000100UL)        /*!< Event Mask on line 8 */
#define  EXTI_EMR_MR9                        (0x00000200UL)        /*!< Event Mask on line 9 */
#define  EXTI_EMR_MR10                       (0x00000400UL)        /*!< Event Mask on line 10 */
#define  EXTI_EMR_MR11                       (0x00000800UL)        /*!< Event Mask on line 11 */
#define  EXTI_EMR_MR12                       (0x00001000UL)        /*!< Event Mask on line 12 */
#define  EXTI_EMR_MR13                       (0x00002000UL)        /*!< Event Mask on line 13 */
#define  EXTI_EMR_MR14                       (0x00004000UL)        /*!< Event Mask on line 14 */
#define  EXTI_EMR_MR15                       (0x00008000UL)        /*!< Event Mask on line 15 */
#define  EXTI_EMR_MR16                       (0x00010000UL)        /*!< Event Mask on line 16 */
#define  EXTI_EMR_MR17                       (0x00020000UL)        /*!< Event Mask on line 17 */
#define  EXTI_EMR_MR18                       (0x00040000UL)        /*!< Event Mask on line 18 */
#define  EXTI_EMR_MR19                       (0x00080000UL)        /*!< Event Mask on line 19 */
#define  EXTI_EMR_MR20                       (0x00100000UL)        /*!< Event Mask on line 20 */
#define  EXTI_EMR_MR21                       (0x00200000UL)        /*!< Event Mask on line 21 */
#define  EXTI_EMR_MR22                       (0x00400000UL)        /*!< Event Mask on line 22 */

/******************  Bit definition for EXTI_RTSR register  *******************/
#define  EXTI_RTSR_TR0                       (0x00000001UL)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR_TR1                       (0x00000002UL)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR_TR2                       (0x00000004UL)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR_TR3                       (0x00000008UL)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR_TR4                       (0x00000010UL)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR_TR5                       (0x00000020UL)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR_TR6                       (0x00000040UL)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR_TR7                       (0x00000080UL)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR_TR8                       (0x00000100UL)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR_TR9                       (0x00000200UL)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR_TR10                      (0x00000400UL)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR_TR11                      (0x00000800UL)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR_TR12                      (0x00001000UL)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR_TR13                      (0x00002000UL)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR_TR14                      (0x00004000UL)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR_TR15                      (0x00008000UL)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR_TR16                      (0x00010000UL)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR_TR17                      (0x00020000UL)        /*!< Rising trigger event configuration bit of line 17 */
#define  EXTI_RTSR_TR18                      (0x00040000UL)        /*!< Rising trigger event configuration bit of line 18 */
#define  EXTI_RTSR_TR19                      (0x00080000UL)        /*!< Rising trigger event configuration bit of line 19 */
#define  EXTI_RTSR_TR20                      (0x00100000UL)        /*!< Rising trigger event configuration bit of line 20 */
#define  EXTI_RTSR_TR21                      (0x00200000UL)        /*!< Rising trigger event configuration bit of line 21 */
#define  EXTI_RTSR_TR22                      (0x00400000UL)        /*!< Rising trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define  EXTI_FTSR_TR0                       (0x00000001UL)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR_TR1                       (0x00000002UL)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR_TR2                       (0x00000004UL)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR_TR3                       (0x00000008UL)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR_TR4                       (0x00000010UL)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR_TR5                       (0x00000020UL)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR_TR6                       (0x00000040UL)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR_TR7                       (0x00000080UL)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR_TR8                       (0x00000100UL)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR_TR9                       (0x00000200UL)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR_TR10                      (0x00000400UL)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR_TR11                      (0x00000800UL)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR_TR12                      (0x00001000UL)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR_TR13                      (0x00002000UL)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR_TR14                      (0x00004000UL)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR_TR15                      (0x00008000UL)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR_TR16                      (0x00010000UL)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR_TR17                      (0x00020000UL)        /*!< Falling trigger event configuration bit of line 17 */
#define  EXTI_FTSR_TR18                      (0x00040000UL)        /*!< Falling trigger event configuration bit of line 18 */
#define  EXTI_FTSR_TR19                      (0x00080000UL)        /*!< Falling trigger event configuration bit of line 19 */
#define  EXTI_FTSR_TR20                      (0x00100000UL)        /*!< Falling trigger event configuration bit of line 20 */
#define  EXTI_FTSR_TR21                      (0x00200000UL)        /*!< Falling trigger event configuration bit of line 21 */
#define  EXTI_FTSR_TR22                      (0x00400000UL)        /*!< Falling trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define  EXTI_SWIER_SWIER0                   (0x00000001UL)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIER_SWIER1                   (0x00000002UL)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIER_SWIER2                   (0x00000004UL)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIER_SWIER3                   (0x00000008UL)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIER_SWIER4                   (0x00000010UL)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIER_SWIER5                   (0x00000020UL)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIER_SWIER6                   (0x00000040UL)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIER_SWIER7                   (0x00000080UL)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIER_SWIER8                   (0x00000100UL)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIER_SWIER9                   (0x00000200UL)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIER_SWIER10                  (0x00000400UL)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIER_SWIER11                  (0x00000800UL)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIER_SWIER12                  (0x00001000UL)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIER_SWIER13                  (0x00002000UL)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIER_SWIER14                  (0x00004000UL)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIER_SWIER15                  (0x00008000UL)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIER_SWIER16                  (0x00010000UL)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIER_SWIER17                  (0x00020000UL)        /*!< Software Interrupt on line 17 */
#define  EXTI_SWIER_SWIER18                  (0x00040000UL)        /*!< Software Interrupt on line 18 */
#define  EXTI_SWIER_SWIER19                  (0x00080000UL)        /*!< Software Interrupt on line 19 */
#define  EXTI_SWIER_SWIER20                  (0x00100000UL)        /*!< Software Interrupt on line 20 */
#define  EXTI_SWIER_SWIER21                  (0x00200000UL)        /*!< Software Interrupt on line 21 */
#define  EXTI_SWIER_SWIER22                  (0x00400000UL)        /*!< Software Interrupt on line 22 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define  EXTI_PR_PR0                         (0x00000001UL)        /*!< Pending bit for line 0 */
#define  EXTI_PR_PR1                         (0x00000002UL)        /*!< Pending bit for line 1 */
#define  EXTI_PR_PR2                         (0x00000004UL)        /*!< Pending bit for line 2 */
#define  EXTI_PR_PR3                         (0x00000008UL)        /*!< Pending bit for line 3 */
#define  EXTI_PR_PR4                         (0x00000010UL)        /*!< Pending bit for line 4 */
#define  EXTI_PR_PR5                         (0x00000020UL)        /*!< Pending bit for line 5 */
#define  EXTI_PR_PR6                         (0x00000040UL)        /*!< Pending bit for line 6 */
#define  EXTI_PR_PR7                         (0x00000080UL)        /*!< Pending bit for line 7 */
#define  EXTI_PR_PR8                         (0x00000100UL)        /*!< Pending bit for line 8 */
#define  EXTI_PR_PR9                         (0x00000200UL)        /*!< Pending bit for line 9 */
#define  EXTI_PR_PR10                        (0x00000400UL)        /*!< Pending bit for line 10 */
#define  EXTI_PR_PR11                        (0x00000800UL)        /*!< Pending bit for line 11 */
#define  EXTI_PR_PR12                        (0x00001000UL)        /*!< Pending bit for line 12 */
#define  EXTI_PR_PR13                        (0x00002000UL)        /*!< Pending bit for line 13 */
#define  EXTI_PR_PR14                        (0x00004000UL)        /*!< Pending bit for line 14 */
#define  EXTI_PR_PR15                        (0x00008000UL)        /*!< Pending bit for line 15 */
#define  EXTI_PR_PR16                        (0x00010000UL)        /*!< Pending bit for line 16 */
#define  EXTI_PR_PR17                        (0x00020000UL)        /*!< Pending bit for line 17 */
#define  EXTI_PR_PR18                        (0x00040000UL)        /*!< Pending bit for line 18 */
#define  EXTI_PR_PR19                        (0x00080000UL)        /*!< Pending bit for line 19 */
#define  EXTI_PR_PR20                        (0x00100000UL)        /*!< Pending bit for line 20 */
#define  EXTI_PR_PR21                        (0x00200000UL)        /*!< Pending bit for line 21 */
#define  EXTI_PR_PR22                        (0x00400000UL)        /*!< Pending bit for line 22 */

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY                    (0x0000000FUL)
#define FLASH_ACR_LATENCY_0WS                (0x00000000UL)
#define FLASH_ACR_LATENCY_1WS                (0x00000001UL)
#define FLASH_ACR_LATENCY_2WS                (0x00000002UL)
#define FLASH_ACR_LATENCY_3WS                (0x00000003UL)
#define FLASH_ACR_LATENCY_4WS                (0x00000004UL)
#define FLASH_ACR_LATENCY_5WS                (0x00000005UL)
#define FLASH_ACR_LATENCY_6WS                (0x00000006UL)
#define FLASH_ACR_LATENCY_7WS                (0x00000007UL)

#define FLASH_ACR_PRFTEN                     (0x00000100UL)
#define FLASH_ACR_ICEN                       (0x00000200UL)
#define FLASH_ACR_DCEN                       (0x00000400UL)
#define FLASH_ACR_ICRST                      (0x00000800UL)
#define FLASH_ACR_DCRST                      (0x00001000UL)
#define FLASH_ACR_BYTE0_ADDRESS              (0x40023C00UL)
#define FLASH_ACR_BYTE2_ADDRESS              (0x40023C03UL)

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP                         (0x00000001UL)
#define FLASH_SR_SOP                         (0x00000002UL)
#define FLASH_SR_WRPERR                      (0x00000010UL)
#define FLASH_SR_PGAERR                      (0x00000020UL)
#define FLASH_SR_PGPERR                      (0x00000040UL)
#define FLASH_SR_PGSERR                      (0x00000080UL)
#define FLASH_SR_BSY                         (0x00010000UL)

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG                          (0x00000001UL)
#define FLASH_CR_SER                         (0x00000002UL)
#define FLASH_CR_MER                         (0x00000004UL)
#define FLASH_CR_SNB                         (0x000000F8UL)
#define FLASH_CR_SNB_0                       (0x00000008UL)
#define FLASH_CR_SNB_1                       (0x00000010UL)
#define FLASH_CR_SNB_2                       (0x00000020UL)
#define FLASH_CR_SNB_3                       (0x00000040UL)
#define FLASH_CR_SNB_4                       (0x00000080UL)
#define FLASH_CR_PSIZE                       (0x00000300UL)
#define FLASH_CR_PSIZE_0                     (0x00000100UL)
#define FLASH_CR_PSIZE_1                     (0x00000200UL)
#define FLASH_CR_STRT                        (0x00010000UL)
#define FLASH_CR_EOPIE                       (0x01000000UL)
#define FLASH_CR_LOCK                        (0x80000000UL)

/*******************  Bits definition for FLASH_OPTCR register  ***************/
#define FLASH_OPTCR_OPTLOCK                 (0x00000001UL)
#define FLASH_OPTCR_OPTSTRT                 (0x00000002UL)
#define FLASH_OPTCR_BOR_LEV_0               (0x00000004UL)
#define FLASH_OPTCR_BOR_LEV_1               (0x00000008UL)
#define FLASH_OPTCR_BOR_LEV                 (0x0000000CUL)

#define FLASH_OPTCR_WDG_SW                  (0x00000020UL)
#define FLASH_OPTCR_nRST_STOP               (0x00000040UL)
#define FLASH_OPTCR_nRST_STDBY              (0x00000080UL)
#define FLASH_OPTCR_RDP                     (0x0000FF00UL)
#define FLASH_OPTCR_RDP_0                   (0x00000100UL)
#define FLASH_OPTCR_RDP_1                   (0x00000200UL)
#define FLASH_OPTCR_RDP_2                   (0x00000400UL)
#define FLASH_OPTCR_RDP_3                   (0x00000800UL)
#define FLASH_OPTCR_RDP_4                   (0x00001000UL)
#define FLASH_OPTCR_RDP_5                   (0x00002000UL)
#define FLASH_OPTCR_RDP_6                   (0x00004000UL)
#define FLASH_OPTCR_RDP_7                   (0x00008000UL)
#define FLASH_OPTCR_nWRP                    (0x0FFF0000UL)
#define FLASH_OPTCR_nWRP_0                  (0x00010000UL)
#define FLASH_OPTCR_nWRP_1                  (0x00020000UL)
#define FLASH_OPTCR_nWRP_2                  (0x00040000UL)
#define FLASH_OPTCR_nWRP_3                  (0x00080000UL)
#define FLASH_OPTCR_nWRP_4                  (0x00100000UL)
#define FLASH_OPTCR_nWRP_5                  (0x00200000UL)
#define FLASH_OPTCR_nWRP_6                  (0x00400000UL)
#define FLASH_OPTCR_nWRP_7                  (0x00800000UL)
#define FLASH_OPTCR_nWRP_8                  (0x01000000UL)
#define FLASH_OPTCR_nWRP_9                  (0x02000000UL)
#define FLASH_OPTCR_nWRP_10                 (0x04000000UL)
#define FLASH_OPTCR_nWRP_11                 (0x08000000UL)

/******************  Bits definition for FLASH_OPTCR1 register  ***************/
#define FLASH_OPTCR1_nWRP                    (0x0FFF0000UL)
#define FLASH_OPTCR1_nWRP_0                  (0x00010000UL)
#define FLASH_OPTCR1_nWRP_1                  (0x00020000UL)
#define FLASH_OPTCR1_nWRP_2                  (0x00040000UL)
#define FLASH_OPTCR1_nWRP_3                  (0x00080000UL)
#define FLASH_OPTCR1_nWRP_4                  (0x00100000UL)
#define FLASH_OPTCR1_nWRP_5                  (0x00200000UL)
#define FLASH_OPTCR1_nWRP_6                  (0x00400000UL)
#define FLASH_OPTCR1_nWRP_7                  (0x00800000UL)
#define FLASH_OPTCR1_nWRP_8                  (0x01000000UL)
#define FLASH_OPTCR1_nWRP_9                  (0x02000000UL)
#define FLASH_OPTCR1_nWRP_10                 (0x04000000UL)
#define FLASH_OPTCR1_nWRP_11                 (0x08000000UL)

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0                    (0x00000003UL)
#define GPIO_MODER_MODER0_0                  (0x00000001UL)
#define GPIO_MODER_MODER0_1                  (0x00000002UL)

#define GPIO_MODER_MODER1                    (0x0000000CUL)
#define GPIO_MODER_MODER1_0                  (0x00000004UL)
#define GPIO_MODER_MODER1_1                  (0x00000008UL)

#define GPIO_MODER_MODER2                    (0x00000030UL)
#define GPIO_MODER_MODER2_0                  (0x00000010UL)
#define GPIO_MODER_MODER2_1                  (0x00000020UL)

#define GPIO_MODER_MODER3                    (0x000000C0UL)
#define GPIO_MODER_MODER3_0                  (0x00000040UL)
#define GPIO_MODER_MODER3_1                  (0x00000080UL)

#define GPIO_MODER_MODER4                    (0x00000300UL)
#define GPIO_MODER_MODER4_0                  (0x00000100UL)
#define GPIO_MODER_MODER4_1                  (0x00000200UL)

#define GPIO_MODER_MODER5                    (0x00000C00UL)
#define GPIO_MODER_MODER5_0                  (0x00000400UL)
#define GPIO_MODER_MODER5_1                  (0x00000800UL)

#define GPIO_MODER_MODER6                    (0x00003000UL)
#define GPIO_MODER_MODER6_0                  (0x00001000UL)
#define GPIO_MODER_MODER6_1                  (0x00002000UL)

#define GPIO_MODER_MODER7                    (0x0000C000UL)
#define GPIO_MODER_MODER7_0                  (0x00004000UL)
#define GPIO_MODER_MODER7_1                  (0x00008000UL)

#define GPIO_MODER_MODER8                    (0x00030000UL)
#define GPIO_MODER_MODER8_0                  (0x00010000UL)
#define GPIO_MODER_MODER8_1                  (0x00020000UL)

#define GPIO_MODER_MODER9                    (0x000C0000UL)
#define GPIO_MODER_MODER9_0                  (0x00040000UL)
#define GPIO_MODER_MODER9_1                  (0x00080000UL)

#define GPIO_MODER_MODER10                   (0x00300000UL)
#define GPIO_MODER_MODER10_0                 (0x00100000UL)
#define GPIO_MODER_MODER10_1                 (0x00200000UL)

#define GPIO_MODER_MODER11                   (0x00C00000UL)
#define GPIO_MODER_MODER11_0                 (0x00400000UL)
#define GPIO_MODER_MODER11_1                 (0x00800000UL)

#define GPIO_MODER_MODER12                   (0x03000000UL)
#define GPIO_MODER_MODER12_0                 (0x01000000UL)
#define GPIO_MODER_MODER12_1                 (0x02000000UL)

#define GPIO_MODER_MODER13                   (0x0C000000UL)
#define GPIO_MODER_MODER13_0                 (0x04000000UL)
#define GPIO_MODER_MODER13_1                 (0x08000000UL)

#define GPIO_MODER_MODER14                   (0x30000000UL)
#define GPIO_MODER_MODER14_0                 (0x10000000UL)
#define GPIO_MODER_MODER14_1                 (0x20000000UL)

#define GPIO_MODER_MODER15                   (0xC0000000UL)
#define GPIO_MODER_MODER15_0                 (0x40000000UL)
#define GPIO_MODER_MODER15_1                 (0x80000000UL)

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT_0                     (0x00000001UL)
#define GPIO_OTYPER_OT_1                     (0x00000002UL)
#define GPIO_OTYPER_OT_2                     (0x00000004UL)
#define GPIO_OTYPER_OT_3                     (0x00000008UL)
#define GPIO_OTYPER_OT_4                     (0x00000010UL)
#define GPIO_OTYPER_OT_5                     (0x00000020UL)
#define GPIO_OTYPER_OT_6                     (0x00000040UL)
#define GPIO_OTYPER_OT_7                     (0x00000080UL)
#define GPIO_OTYPER_OT_8                     (0x00000100UL)
#define GPIO_OTYPER_OT_9                     (0x00000200UL)
#define GPIO_OTYPER_OT_10                    (0x00000400UL)
#define GPIO_OTYPER_OT_11                    (0x00000800UL)
#define GPIO_OTYPER_OT_12                    (0x00001000UL)
#define GPIO_OTYPER_OT_13                    (0x00002000UL)
#define GPIO_OTYPER_OT_14                    (0x00004000UL)
#define GPIO_OTYPER_OT_15                    (0x00008000UL)

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDER_OSPEEDR0               (0x00000003UL)
#define GPIO_OSPEEDER_OSPEEDR0_0             (0x00000001UL)
#define GPIO_OSPEEDER_OSPEEDR0_1             (0x00000002UL)

#define GPIO_OSPEEDER_OSPEEDR1               (0x0000000CUL)
#define GPIO_OSPEEDER_OSPEEDR1_0             (0x00000004UL)
#define GPIO_OSPEEDER_OSPEEDR1_1             (0x00000008UL)

#define GPIO_OSPEEDER_OSPEEDR2               (0x00000030UL)
#define GPIO_OSPEEDER_OSPEEDR2_0             (0x00000010UL)
#define GPIO_OSPEEDER_OSPEEDR2_1             (0x00000020UL)

#define GPIO_OSPEEDER_OSPEEDR3               (0x000000C0UL)
#define GPIO_OSPEEDER_OSPEEDR3_0             (0x00000040UL)
#define GPIO_OSPEEDER_OSPEEDR3_1             (0x00000080UL)

#define GPIO_OSPEEDER_OSPEEDR4               (0x00000300UL)
#define GPIO_OSPEEDER_OSPEEDR4_0             (0x00000100UL)
#define GPIO_OSPEEDER_OSPEEDR4_1             (0x00000200UL)

#define GPIO_OSPEEDER_OSPEEDR5               (0x00000C00UL)
#define GPIO_OSPEEDER_OSPEEDR5_0             (0x00000400UL)
#define GPIO_OSPEEDER_OSPEEDR5_1             (0x00000800UL)

#define GPIO_OSPEEDER_OSPEEDR6               (0x00003000UL)
#define GPIO_OSPEEDER_OSPEEDR6_0             (0x00001000UL)
#define GPIO_OSPEEDER_OSPEEDR6_1             (0x00002000UL)

#define GPIO_OSPEEDER_OSPEEDR7               (0x0000C000UL)
#define GPIO_OSPEEDER_OSPEEDR7_0             (0x00004000UL)
#define GPIO_OSPEEDER_OSPEEDR7_1             (0x00008000UL)

#define GPIO_OSPEEDER_OSPEEDR8               (0x00030000UL)
#define GPIO_OSPEEDER_OSPEEDR8_0             (0x00010000UL)
#define GPIO_OSPEEDER_OSPEEDR8_1             (0x00020000UL)

#define GPIO_OSPEEDER_OSPEEDR9               (0x000C0000UL)
#define GPIO_OSPEEDER_OSPEEDR9_0             (0x00040000UL)
#define GPIO_OSPEEDER_OSPEEDR9_1             (0x00080000UL)

#define GPIO_OSPEEDER_OSPEEDR10              (0x00300000UL)
#define GPIO_OSPEEDER_OSPEEDR10_0            (0x00100000UL)
#define GPIO_OSPEEDER_OSPEEDR10_1            (0x00200000UL)

#define GPIO_OSPEEDER_OSPEEDR11              (0x00C00000UL)
#define GPIO_OSPEEDER_OSPEEDR11_0            (0x00400000UL)
#define GPIO_OSPEEDER_OSPEEDR11_1            (0x00800000UL)

#define GPIO_OSPEEDER_OSPEEDR12              (0x03000000UL)
#define GPIO_OSPEEDER_OSPEEDR12_0            (0x01000000UL)
#define GPIO_OSPEEDER_OSPEEDR12_1            (0x02000000UL)

#define GPIO_OSPEEDER_OSPEEDR13              (0x0C000000UL)
#define GPIO_OSPEEDER_OSPEEDR13_0            (0x04000000UL)
#define GPIO_OSPEEDER_OSPEEDR13_1            (0x08000000UL)

#define GPIO_OSPEEDER_OSPEEDR14              (0x30000000UL)
#define GPIO_OSPEEDER_OSPEEDR14_0            (0x10000000UL)
#define GPIO_OSPEEDER_OSPEEDR14_1            (0x20000000UL)

#define GPIO_OSPEEDER_OSPEEDR15              (0xC0000000UL)
#define GPIO_OSPEEDER_OSPEEDR15_0            (0x40000000UL)
#define GPIO_OSPEEDER_OSPEEDR15_1            (0x80000000UL)

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPDR0                    (0x00000003UL)
#define GPIO_PUPDR_PUPDR0_0                  (0x00000001UL)
#define GPIO_PUPDR_PUPDR0_1                  (0x00000002UL)

#define GPIO_PUPDR_PUPDR1                    (0x0000000CUL)
#define GPIO_PUPDR_PUPDR1_0                  (0x00000004UL)
#define GPIO_PUPDR_PUPDR1_1                  (0x00000008UL)

#define GPIO_PUPDR_PUPDR2                    (0x00000030UL)
#define GPIO_PUPDR_PUPDR2_0                  (0x00000010UL)
#define GPIO_PUPDR_PUPDR2_1                  (0x00000020UL)

#define GPIO_PUPDR_PUPDR3                    (0x000000C0UL)
#define GPIO_PUPDR_PUPDR3_0                  (0x00000040UL)
#define GPIO_PUPDR_PUPDR3_1                  (0x00000080UL)

#define GPIO_PUPDR_PUPDR4                    (0x00000300UL)
#define GPIO_PUPDR_PUPDR4_0                  (0x00000100UL)
#define GPIO_PUPDR_PUPDR4_1                  (0x00000200UL)

#define GPIO_PUPDR_PUPDR5                    (0x00000C00UL)
#define GPIO_PUPDR_PUPDR5_0                  (0x00000400UL)
#define GPIO_PUPDR_PUPDR5_1                  (0x00000800UL)

#define GPIO_PUPDR_PUPDR6                    (0x00003000UL)
#define GPIO_PUPDR_PUPDR6_0                  (0x00001000UL)
#define GPIO_PUPDR_PUPDR6_1                  (0x00002000UL)

#define GPIO_PUPDR_PUPDR7                    (0x0000C000UL)
#define GPIO_PUPDR_PUPDR7_0                  (0x00004000UL)
#define GPIO_PUPDR_PUPDR7_1                  (0x00008000UL)

#define GPIO_PUPDR_PUPDR8                    (0x00030000UL)
#define GPIO_PUPDR_PUPDR8_0                  (0x00010000UL)
#define GPIO_PUPDR_PUPDR8_1                  (0x00020000UL)

#define GPIO_PUPDR_PUPDR9                    (0x000C0000UL)
#define GPIO_PUPDR_PUPDR9_0                  (0x00040000UL)
#define GPIO_PUPDR_PUPDR9_1                  (0x00080000UL)

#define GPIO_PUPDR_PUPDR10                   (0x00300000UL)
#define GPIO_PUPDR_PUPDR10_0                 (0x00100000UL)
#define GPIO_PUPDR_PUPDR10_1                 (0x00200000UL)

#define GPIO_PUPDR_PUPDR11                   (0x00C00000UL)
#define GPIO_PUPDR_PUPDR11_0                 (0x00400000UL)
#define GPIO_PUPDR_PUPDR11_1                 (0x00800000UL)

#define GPIO_PUPDR_PUPDR12                   (0x03000000UL)
#define GPIO_PUPDR_PUPDR12_0                 (0x01000000UL)
#define GPIO_PUPDR_PUPDR12_1                 (0x02000000UL)

#define GPIO_PUPDR_PUPDR13                   (0x0C000000UL)
#define GPIO_PUPDR_PUPDR13_0                 (0x04000000UL)
#define GPIO_PUPDR_PUPDR13_1                 (0x08000000UL)

#define GPIO_PUPDR_PUPDR14                   (0x30000000UL)
#define GPIO_PUPDR_PUPDR14_0                 (0x10000000UL)
#define GPIO_PUPDR_PUPDR14_1                 (0x20000000UL)

#define GPIO_PUPDR_PUPDR15                   (0xC0000000UL)
#define GPIO_PUPDR_PUPDR15_0                 (0x40000000UL)
#define GPIO_PUPDR_PUPDR15_1                 (0x80000000UL)

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR_0                       (0x00000001UL)
#define GPIO_IDR_IDR_1                       (0x00000002UL)
#define GPIO_IDR_IDR_2                       (0x00000004UL)
#define GPIO_IDR_IDR_3                       (0x00000008UL)
#define GPIO_IDR_IDR_4                       (0x00000010UL)
#define GPIO_IDR_IDR_5                       (0x00000020UL)
#define GPIO_IDR_IDR_6                       (0x00000040UL)
#define GPIO_IDR_IDR_7                       (0x00000080UL)
#define GPIO_IDR_IDR_8                       (0x00000100UL)
#define GPIO_IDR_IDR_9                       (0x00000200UL)
#define GPIO_IDR_IDR_10                      (0x00000400UL)
#define GPIO_IDR_IDR_11                      (0x00000800UL)
#define GPIO_IDR_IDR_12                      (0x00001000UL)
#define GPIO_IDR_IDR_13                      (0x00002000UL)
#define GPIO_IDR_IDR_14                      (0x00004000UL)
#define GPIO_IDR_IDR_15                      (0x00008000UL)
/* Old GPIO_IDR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_IDR_0                    GPIO_IDR_IDR_0
#define GPIO_OTYPER_IDR_1                    GPIO_IDR_IDR_1
#define GPIO_OTYPER_IDR_2                    GPIO_IDR_IDR_2
#define GPIO_OTYPER_IDR_3                    GPIO_IDR_IDR_3
#define GPIO_OTYPER_IDR_4                    GPIO_IDR_IDR_4
#define GPIO_OTYPER_IDR_5                    GPIO_IDR_IDR_5
#define GPIO_OTYPER_IDR_6                    GPIO_IDR_IDR_6
#define GPIO_OTYPER_IDR_7                    GPIO_IDR_IDR_7
#define GPIO_OTYPER_IDR_8                    GPIO_IDR_IDR_8
#define GPIO_OTYPER_IDR_9                    GPIO_IDR_IDR_9
#define GPIO_OTYPER_IDR_10                   GPIO_IDR_IDR_10
#define GPIO_OTYPER_IDR_11                   GPIO_IDR_IDR_11
#define GPIO_OTYPER_IDR_12                   GPIO_IDR_IDR_12
#define GPIO_OTYPER_IDR_13                   GPIO_IDR_IDR_13
#define GPIO_OTYPER_IDR_14                   GPIO_IDR_IDR_14
#define GPIO_OTYPER_IDR_15                   GPIO_IDR_IDR_15

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR_0                       (0x00000001UL)
#define GPIO_ODR_ODR_1                       (0x00000002UL)
#define GPIO_ODR_ODR_2                       (0x00000004UL)
#define GPIO_ODR_ODR_3                       (0x00000008UL)
#define GPIO_ODR_ODR_4                       (0x00000010UL)
#define GPIO_ODR_ODR_5                       (0x00000020UL)
#define GPIO_ODR_ODR_6                       (0x00000040UL)
#define GPIO_ODR_ODR_7                       (0x00000080UL)
#define GPIO_ODR_ODR_8                       (0x00000100UL)
#define GPIO_ODR_ODR_9                       (0x00000200UL)
#define GPIO_ODR_ODR_10                      (0x00000400UL)
#define GPIO_ODR_ODR_11                      (0x00000800UL)
#define GPIO_ODR_ODR_12                      (0x00001000UL)
#define GPIO_ODR_ODR_13                      (0x00002000UL)
#define GPIO_ODR_ODR_14                      (0x00004000UL)
#define GPIO_ODR_ODR_15                      (0x00008000UL)
/* Old GPIO_ODR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_ODR_0                    GPIO_ODR_ODR_0
#define GPIO_OTYPER_ODR_1                    GPIO_ODR_ODR_1
#define GPIO_OTYPER_ODR_2                    GPIO_ODR_ODR_2
#define GPIO_OTYPER_ODR_3                    GPIO_ODR_ODR_3
#define GPIO_OTYPER_ODR_4                    GPIO_ODR_ODR_4
#define GPIO_OTYPER_ODR_5                    GPIO_ODR_ODR_5
#define GPIO_OTYPER_ODR_6                    GPIO_ODR_ODR_6
#define GPIO_OTYPER_ODR_7                    GPIO_ODR_ODR_7
#define GPIO_OTYPER_ODR_8                    GPIO_ODR_ODR_8
#define GPIO_OTYPER_ODR_9                    GPIO_ODR_ODR_9
#define GPIO_OTYPER_ODR_10                   GPIO_ODR_ODR_10
#define GPIO_OTYPER_ODR_11                   GPIO_ODR_ODR_11
#define GPIO_OTYPER_ODR_12                   GPIO_ODR_ODR_12
#define GPIO_OTYPER_ODR_13                   GPIO_ODR_ODR_13
#define GPIO_OTYPER_ODR_14                   GPIO_ODR_ODR_14
#define GPIO_OTYPER_ODR_15                   GPIO_ODR_ODR_15

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS_0                       (0x00000001UL)
#define GPIO_BSRR_BS_1                       (0x00000002UL)
#define GPIO_BSRR_BS_2                       (0x00000004UL)
#define GPIO_BSRR_BS_3                       (0x00000008UL)
#define GPIO_BSRR_BS_4                       (0x00000010UL)
#define GPIO_BSRR_BS_5                       (0x00000020UL)
#define GPIO_BSRR_BS_6                       (0x00000040UL)
#define GPIO_BSRR_BS_7                       (0x00000080UL)
#define GPIO_BSRR_BS_8                       (0x00000100UL)
#define GPIO_BSRR_BS_9                       (0x00000200UL)
#define GPIO_BSRR_BS_10                      (0x00000400UL)
#define GPIO_BSRR_BS_11                      (0x00000800UL)
#define GPIO_BSRR_BS_12                      (0x00001000UL)
#define GPIO_BSRR_BS_13                      (0x00002000UL)
#define GPIO_BSRR_BS_14                      (0x00004000UL)
#define GPIO_BSRR_BS_15                      (0x00008000UL)
#define GPIO_BSRR_BR_0                       (0x00010000UL)
#define GPIO_BSRR_BR_1                       (0x00020000UL)
#define GPIO_BSRR_BR_2                       (0x00040000UL)
#define GPIO_BSRR_BR_3                       (0x00080000UL)
#define GPIO_BSRR_BR_4                       (0x00100000UL)
#define GPIO_BSRR_BR_5                       (0x00200000UL)
#define GPIO_BSRR_BR_6                       (0x00400000UL)
#define GPIO_BSRR_BR_7                       (0x00800000UL)
#define GPIO_BSRR_BR_8                       (0x01000000UL)
#define GPIO_BSRR_BR_9                       (0x02000000UL)
#define GPIO_BSRR_BR_10                      (0x04000000UL)
#define GPIO_BSRR_BR_11                      (0x08000000UL)
#define GPIO_BSRR_BR_12                      (0x10000000UL)
#define GPIO_BSRR_BR_13                      (0x20000000UL)
#define GPIO_BSRR_BR_14                      (0x40000000UL)
#define GPIO_BSRR_BR_15                      (0x80000000UL)

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0                       (0x00000001UL)
#define GPIO_LCKR_LCK1                       (0x00000002UL)
#define GPIO_LCKR_LCK2                       (0x00000004UL)
#define GPIO_LCKR_LCK3                       (0x00000008UL)
#define GPIO_LCKR_LCK4                       (0x00000010UL)
#define GPIO_LCKR_LCK5                       (0x00000020UL)
#define GPIO_LCKR_LCK6                       (0x00000040UL)
#define GPIO_LCKR_LCK7                       (0x00000080UL)
#define GPIO_LCKR_LCK8                       (0x00000100UL)
#define GPIO_LCKR_LCK9                       (0x00000200UL)
#define GPIO_LCKR_LCK10                      (0x00000400UL)
#define GPIO_LCKR_LCK11                      (0x00000800UL)
#define GPIO_LCKR_LCK12                      (0x00001000UL)
#define GPIO_LCKR_LCK13                      (0x00002000UL)
#define GPIO_LCKR_LCK14                      (0x00004000UL)
#define GPIO_LCKR_LCK15                      (0x00008000UL)
#define GPIO_LCKR_LCKK                       (0x00010000UL)

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  ********************/
#define  I2C_CR1_PE                          (0x00000001UL)     /*!<Peripheral Enable                             */
#define  I2C_CR1_SMBUS                       (0x00000002UL)     /*!<SMBus Mode                                    */
#define  I2C_CR1_SMBTYPE                     (0x00000008UL)     /*!<SMBus Type                                    */
#define  I2C_CR1_ENARP                       (0x00000010UL)     /*!<ARP Enable                                    */
#define  I2C_CR1_ENPEC                       (0x00000020UL)     /*!<PEC Enable                                    */
#define  I2C_CR1_ENGC                        (0x00000040UL)     /*!<General Call Enable                           */
#define  I2C_CR1_NOSTRETCH                   (0x00000080UL)     /*!<Clock Stretching Disable (Slave mode)  */
#define  I2C_CR1_START                       (0x00000100UL)     /*!<Start Generation                              */
#define  I2C_CR1_STOP                        (0x00000200UL)     /*!<Stop Generation                               */
#define  I2C_CR1_ACK                         (0x00000400UL)     /*!<Acknowledge Enable                            */
#define  I2C_CR1_POS                         (0x00000800UL)     /*!<Acknowledge/PEC Position (for data reception) */
#define  I2C_CR1_PEC                         (0x00001000UL)     /*!<Packet Error Checking                         */
#define  I2C_CR1_ALERT                       (0x00002000UL)     /*!<SMBus Alert                                   */
#define  I2C_CR1_SWRST                       (0x00008000UL)     /*!<Software Reset                                */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_FREQ                        (0x0000003FUL)     /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
#define  I2C_CR2_FREQ_0                      (0x00000001UL)     /*!<Bit 0 */
#define  I2C_CR2_FREQ_1                      (0x00000002UL)     /*!<Bit 1 */
#define  I2C_CR2_FREQ_2                      (0x00000004UL)     /*!<Bit 2 */
#define  I2C_CR2_FREQ_3                      (0x00000008UL)     /*!<Bit 3 */
#define  I2C_CR2_FREQ_4                      (0x00000010UL)     /*!<Bit 4 */
#define  I2C_CR2_FREQ_5                      (0x00000020UL)     /*!<Bit 5 */

#define  I2C_CR2_ITERREN                     (0x00000100UL)     /*!<Error Interrupt Enable  */
#define  I2C_CR2_ITEVTEN                     (0x00000200UL)     /*!<Event Interrupt Enable  */
#define  I2C_CR2_ITBUFEN                     (0x00000400UL)     /*!<Buffer Interrupt Enable */
#define  I2C_CR2_DMAEN                       (0x00000800UL)     /*!<DMA Requests Enable     */
#define  I2C_CR2_LAST                        (0x00001000UL)     /*!<DMA Last Transfer       */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define  I2C_OAR1_ADD1_7                     (0x000000FEUL)     /*!<Interface Address */
#define  I2C_OAR1_ADD8_9                     (0x00000300UL)     /*!<Interface Address */

#define  I2C_OAR1_ADD0                       (0x00000001UL)     /*!<Bit 0 */
#define  I2C_OAR1_ADD1                       (0x00000002UL)     /*!<Bit 1 */
#define  I2C_OAR1_ADD2                       (0x00000004UL)     /*!<Bit 2 */
#define  I2C_OAR1_ADD3                       (0x00000008UL)     /*!<Bit 3 */
#define  I2C_OAR1_ADD4                       (0x00000010UL)     /*!<Bit 4 */
#define  I2C_OAR1_ADD5                       (0x00000020UL)     /*!<Bit 5 */
#define  I2C_OAR1_ADD6                       (0x00000040UL)     /*!<Bit 6 */
#define  I2C_OAR1_ADD7                       (0x00000080UL)     /*!<Bit 7 */
#define  I2C_OAR1_ADD8                       (0x00000100UL)     /*!<Bit 8 */
#define  I2C_OAR1_ADD9                       (0x00000200UL)     /*!<Bit 9 */

#define  I2C_OAR1_ADDMODE                    (0x00008000UL)     /*!<Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  I2C_OAR2_ENDUAL                     (0x00000001UL)        /*!<Dual addressing mode enable */
#define  I2C_OAR2_ADD2                       (0x000000FEUL)        /*!<Interface address           */

/********************  Bit definition for I2C_DR register  ********************/
#define  I2C_DR_DR                           (0x000000FFUL)        /*!<8-bit Data Register         */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define  I2C_SR1_SB                          (0x00000001UL)     /*!<Start Bit (Master mode)                  */
#define  I2C_SR1_ADDR                        (0x00000002UL)     /*!<Address sent (master mode)/matched (slave mode) */
#define  I2C_SR1_BTF                         (0x00000004UL)     /*!<Byte Transfer Finished                          */
#define  I2C_SR1_ADD10                       (0x00000008UL)     /*!<10-bit header sent (Master mode)         */
#define  I2C_SR1_STOPF                       (0x00000010UL)     /*!<Stop detection (Slave mode)              */
#define  I2C_SR1_RXNE                        (0x00000040UL)     /*!<Data Register not Empty (receivers)      */
#define  I2C_SR1_TXE                         (0x00000080UL)     /*!<Data Register Empty (transmitters)       */
#define  I2C_SR1_BERR                        (0x00000100UL)     /*!<Bus Error                                       */
#define  I2C_SR1_ARLO                        (0x00000200UL)     /*!<Arbitration Lost (master mode)           */
#define  I2C_SR1_AF                          (0x00000400UL)     /*!<Acknowledge Failure                             */
#define  I2C_SR1_OVR                         (0x00000800UL)     /*!<Overrun/Underrun                                */
#define  I2C_SR1_PECERR                      (0x00001000UL)     /*!<PEC Error in reception                          */
#define  I2C_SR1_TIMEOUT                     (0x00004000UL)     /*!<Timeout or Tlow Error                           */
#define  I2C_SR1_SMBALERT                    (0x00008000UL)     /*!<SMBus Alert                                     */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define  I2C_SR2_MSL                         (0x00000001UL)     /*!<Master/Slave                              */
#define  I2C_SR2_BUSY                        (0x00000002UL)     /*!<Bus Busy                                  */
#define  I2C_SR2_TRA                         (0x00000004UL)     /*!<Transmitter/Receiver                      */
#define  I2C_SR2_GENCALL                     (0x00000010UL)     /*!<General Call Address (Slave mode)  */
#define  I2C_SR2_SMBDEFAULT                  (0x00000020UL)     /*!<SMBus Device Default Address (Slave mode) */
#define  I2C_SR2_SMBHOST                     (0x00000040UL)     /*!<SMBus Host Header (Slave mode)     */
#define  I2C_SR2_DUALF                       (0x00000080UL)     /*!<Dual Flag (Slave mode)             */
#define  I2C_SR2_PEC                         (0x0000FF00UL)     /*!<Packet Error Checking Register            */

/*******************  Bit definition for I2C_CCR register  ********************/
#define  I2C_CCR_CCR                         (0x00000FFFUL)     /*!<Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CCR_DUTY                        (0x00004000UL)     /*!<Fast Mode Duty Cycle                                       */
#define  I2C_CCR_FS                          (0x00008000UL)     /*!<I2C Master Mode Selection                                  */

/******************  Bit definition for I2C_TRISE register  *******************/
#define  I2C_TRISE_TRISE                     (0x0000003FUL)     /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************  Bit definition for I2C_FLTR register  *******************/
#define  I2C_FLTR_DNF                        (0x0000000FUL)     /*!<Digital Noise Filter */
#define  I2C_FLTR_ANOFF                      (0x00000010UL)     /*!<Analog Noise Filter OFF */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         (0xFFFFUL)            /*!<Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          (0x07UL)               /*!<PR[2:0] (Prescaler divider)         */
#define  IWDG_PR_PR_0                        (0x01UL)               /*!<Bit 0 */
#define  IWDG_PR_PR_1                        (0x02UL)               /*!<Bit 1 */
#define  IWDG_PR_PR_2                        (0x04UL)               /*!<Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         (0x0FFFUL)            /*!<Watchdog counter reload value        */

/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         (0x01UL)               /*!<Watchdog prescaler value update      */
#define  IWDG_SR_RVU                         (0x02UL)               /*!<Watchdog counter reload value update */


/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
#define  PWR_CR_LPDS                         (0x00000001UL)     /*!< Low-Power Deepsleep                 */
#define  PWR_CR_PDDS                         (0x00000002UL)     /*!< Power Down Deepsleep                */
#define  PWR_CR_CWUF                         (0x00000004UL)     /*!< Clear Wakeup Flag                   */
#define  PWR_CR_CSBF                         (0x00000008UL)     /*!< Clear Standby Flag                  */
#define  PWR_CR_PVDE                         (0x00000010UL)     /*!< Power Voltage Detector Enable       */

#define  PWR_CR_PLS                          (0x000000E0UL)     /*!< PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CR_PLS_0                        (0x00000020UL)     /*!< Bit 0 */
#define  PWR_CR_PLS_1                        (0x00000040UL)     /*!< Bit 1 */
#define  PWR_CR_PLS_2                        (0x00000080UL)     /*!< Bit 2 */

/*!< PVD level configuration */
#define  PWR_CR_PLS_LEV0                     (0x00000000UL)     /*!< PVD level 0 */
#define  PWR_CR_PLS_LEV1                     (0x00000020UL)     /*!< PVD level 1 */
#define  PWR_CR_PLS_LEV2                     (0x00000040UL)     /*!< PVD level 2 */
#define  PWR_CR_PLS_LEV3                     (0x00000060UL)     /*!< PVD level 3 */
#define  PWR_CR_PLS_LEV4                     (0x00000080UL)     /*!< PVD level 4 */
#define  PWR_CR_PLS_LEV5                     (0x000000A0UL)     /*!< PVD level 5 */
#define  PWR_CR_PLS_LEV6                     (0x000000C0UL)     /*!< PVD level 6 */
#define  PWR_CR_PLS_LEV7                     (0x000000E0UL)     /*!< PVD level 7 */

#define  PWR_CR_DBP                          (0x00000100UL)     /*!< Disable Backup Domain write protection                     */
#define  PWR_CR_FPDS                         (0x00000200UL)     /*!< Flash power down in Stop mode                              */
#define  PWR_CR_LPLVDS                       (0x00000400UL)     /*!< Low Power Regulator Low Voltage in Deep Sleep mode         */
#define  PWR_CR_MRLVDS                       (0x00000800UL)     /*!< Main Regulator Low Voltage in Deep Sleep mode              */
#define  PWR_CR_ADCDC1                       (0x00002000UL)     /*!< Refer to AN4073 on how to use this bit                     */

#define  PWR_CR_VOS                          (0x0000C000UL)     /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
#define  PWR_CR_VOS_0                        (0x00004000UL)     /*!< Bit 0 */
#define  PWR_CR_VOS_1                        (0x00008000UL)     /*!< Bit 1 */

#define  PWR_CR_FMSSR                        (0x00100000UL)     /*!< Flash Memory Sleep System Run        */
#define  PWR_CR_FISSR                        (0x00200000UL)     /*!< Flash Interface Stop while System Run */
/* Legacy define */
#define  PWR_CR_PMODE                        PWR_CR_VOS

/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         (0x00000001UL)     /*!< Wakeup Flag                                      */
#define  PWR_CSR_SBF                         (0x00000002UL)     /*!< Standby Flag                                     */
#define  PWR_CSR_PVDO                        (0x00000004UL)     /*!< PVD Output                                       */
#define  PWR_CSR_BRR                         (0x00000008UL)     /*!< Backup regulator ready                           */
#define  PWR_CSR_EWUP                        (0x00000100UL)     /*!< Enable WKUP pin                                  */
#define  PWR_CSR_BRE                         (0x00000200UL)     /*!< Backup regulator enable                          */
#define  PWR_CSR_VOSRDY                      (0x00004000UL)     /*!< Regulator voltage scaling output selection ready */

/* Legacy define */
#define  PWR_CSR_REGRDY                      PWR_CSR_VOSRDY

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        (0x00000001UL)
#define  RCC_CR_HSIRDY                       (0x00000002UL)

#define  RCC_CR_HSITRIM                      (0x000000F8UL)
#define  RCC_CR_HSITRIM_0                    (0x00000008UL)/*!<Bit 0 */
#define  RCC_CR_HSITRIM_1                    (0x00000010UL)/*!<Bit 1 */
#define  RCC_CR_HSITRIM_2                    (0x00000020UL)/*!<Bit 2 */
#define  RCC_CR_HSITRIM_3                    (0x00000040UL)/*!<Bit 3 */
#define  RCC_CR_HSITRIM_4                    (0x00000080UL)/*!<Bit 4 */

#define  RCC_CR_HSICAL                       (0x0000FF00UL)
#define  RCC_CR_HSICAL_0                     (0x00000100UL)/*!<Bit 0 */
#define  RCC_CR_HSICAL_1                     (0x00000200UL)/*!<Bit 1 */
#define  RCC_CR_HSICAL_2                     (0x00000400UL)/*!<Bit 2 */
#define  RCC_CR_HSICAL_3                     (0x00000800UL)/*!<Bit 3 */
#define  RCC_CR_HSICAL_4                     (0x00001000UL)/*!<Bit 4 */
#define  RCC_CR_HSICAL_5                     (0x00002000UL)/*!<Bit 5 */
#define  RCC_CR_HSICAL_6                     (0x00004000UL)/*!<Bit 6 */
#define  RCC_CR_HSICAL_7                     (0x00008000UL)/*!<Bit 7 */

#define  RCC_CR_HSEON                        (0x00010000UL)
#define  RCC_CR_HSERDY                       (0x00020000UL)
#define  RCC_CR_HSEBYP                       (0x00040000UL)
#define  RCC_CR_CSSON                        (0x00080000UL)
#define  RCC_CR_PLLON                        (0x01000000UL)
#define  RCC_CR_PLLRDY                       (0x02000000UL)
#define  RCC_CR_PLLI2SON                     (0x04000000UL)
#define  RCC_CR_PLLI2SRDY                    (0x08000000UL)

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define  RCC_PLLCFGR_PLLM                    (0x0000003FUL)
#define  RCC_PLLCFGR_PLLM_0                  (0x00000001UL)
#define  RCC_PLLCFGR_PLLM_1                  (0x00000002UL)
#define  RCC_PLLCFGR_PLLM_2                  (0x00000004UL)
#define  RCC_PLLCFGR_PLLM_3                  (0x00000008UL)
#define  RCC_PLLCFGR_PLLM_4                  (0x00000010UL)
#define  RCC_PLLCFGR_PLLM_5                  (0x00000020UL)

#define  RCC_PLLCFGR_PLLN                     (0x00007FC0UL)
#define  RCC_PLLCFGR_PLLN_0                   (0x00000040UL)
#define  RCC_PLLCFGR_PLLN_1                   (0x00000080UL)
#define  RCC_PLLCFGR_PLLN_2                   (0x00000100UL)
#define  RCC_PLLCFGR_PLLN_3                   (0x00000200UL)
#define  RCC_PLLCFGR_PLLN_4                   (0x00000400UL)
#define  RCC_PLLCFGR_PLLN_5                   (0x00000800UL)
#define  RCC_PLLCFGR_PLLN_6                   (0x00001000UL)
#define  RCC_PLLCFGR_PLLN_7                   (0x00002000UL)
#define  RCC_PLLCFGR_PLLN_8                   (0x00004000UL)

#define  RCC_PLLCFGR_PLLP                    (0x00030000UL)
#define  RCC_PLLCFGR_PLLP_0                  (0x00010000UL)
#define  RCC_PLLCFGR_PLLP_1                  (0x00020000UL)

#define  RCC_PLLCFGR_PLLSRC                  (0x00400000UL)
#define  RCC_PLLCFGR_PLLSRC_HSE              (0x00400000UL)
#define  RCC_PLLCFGR_PLLSRC_HSI              (0x00000000UL)

#define  RCC_PLLCFGR_PLLQ                    (0x0F000000UL)
#define  RCC_PLLCFGR_PLLQ_0                  (0x01000000UL)
#define  RCC_PLLCFGR_PLLQ_1                  (0x02000000UL)
#define  RCC_PLLCFGR_PLLQ_2                  (0x04000000UL)
#define  RCC_PLLCFGR_PLLQ_3                  (0x08000000UL)

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         (0x00000003UL)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       (0x00000001UL)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       (0x00000002UL)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     (0x00000000UL)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     (0x00000001UL)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     (0x00000002UL)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        (0x0000000CUL)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      (0x00000004UL)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      (0x00000008UL)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    (0x00000000UL)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    (0x00000004UL)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    (0x00000008UL)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       (0x000000F0UL)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     (0x00000010UL)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     (0x00000020UL)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     (0x00000040UL)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     (0x00000080UL)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  (0x00000000UL)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  (0x00000080UL)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  (0x00000090UL)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  (0x000000A0UL)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 (0x000000B0UL)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 (0x000000C0UL)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                (0x000000D0UL)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                (0x000000E0UL)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                (0x000000F0UL)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      (0x00001C00UL)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    (0x00000400UL)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    (0x00000800UL)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    (0x00001000UL)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 (0x00000000UL)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 (0x00001000UL)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 (0x00001400UL)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 (0x00001800UL)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                (0x00001C00UL)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      (0x0000E000UL)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    (0x00002000UL)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    (0x00004000UL)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    (0x00008000UL)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 (0x00000000UL)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 (0x00008000UL)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 (0x0000A000UL)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 (0x0000C000UL)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                (0x0000E000UL)        /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define  RCC_CFGR_RTCPRE                     (0x001F0000UL)
#define  RCC_CFGR_RTCPRE_0                   (0x00010000UL)
#define  RCC_CFGR_RTCPRE_1                   (0x00020000UL)
#define  RCC_CFGR_RTCPRE_2                   (0x00040000UL)
#define  RCC_CFGR_RTCPRE_3                   (0x00080000UL)
#define  RCC_CFGR_RTCPRE_4                   (0x00100000UL)

/*!< MCO1 configuration */
#define  RCC_CFGR_MCO1                       (0x00600000UL)
#define  RCC_CFGR_MCO1_0                     (0x00200000UL)
#define  RCC_CFGR_MCO1_1                     (0x00400000UL)

#define  RCC_CFGR_I2SSRC                     (0x00800000UL)

#define  RCC_CFGR_MCO1PRE                    (0x07000000UL)
#define  RCC_CFGR_MCO1PRE_0                  (0x01000000UL)
#define  RCC_CFGR_MCO1PRE_1                  (0x02000000UL)
#define  RCC_CFGR_MCO1PRE_2                  (0x04000000UL)

#define  RCC_CFGR_MCO2PRE                    (0x38000000UL)
#define  RCC_CFGR_MCO2PRE_0                  (0x08000000UL)
#define  RCC_CFGR_MCO2PRE_1                  (0x10000000UL)
#define  RCC_CFGR_MCO2PRE_2                  (0x20000000UL)

#define  RCC_CFGR_MCO2                       (0xC0000000UL)
#define  RCC_CFGR_MCO2_0                     (0x40000000UL)
#define  RCC_CFGR_MCO2_1                     (0x80000000UL)

/********************  Bit definition for RCC_CIR register  *******************/
#define  RCC_CIR_LSIRDYF                     (0x00000001UL)
#define  RCC_CIR_LSERDYF                     (0x00000002UL)
#define  RCC_CIR_HSIRDYF                     (0x00000004UL)
#define  RCC_CIR_HSERDYF                     (0x00000008UL)
#define  RCC_CIR_PLLRDYF                     (0x00000010UL)
#define  RCC_CIR_PLLI2SRDYF                  (0x00000020UL)

#define  RCC_CIR_CSSF                        (0x00000080UL)
#define  RCC_CIR_LSIRDYIE                    (0x00000100UL)
#define  RCC_CIR_LSERDYIE                    (0x00000200UL)
#define  RCC_CIR_HSIRDYIE                    (0x00000400UL)
#define  RCC_CIR_HSERDYIE                    (0x00000800UL)
#define  RCC_CIR_PLLRDYIE                    (0x00001000UL)
#define  RCC_CIR_PLLI2SRDYIE                 (0x00002000UL)

#define  RCC_CIR_LSIRDYC                     (0x00010000UL)
#define  RCC_CIR_LSERDYC                     (0x00020000UL)
#define  RCC_CIR_HSIRDYC                     (0x00040000UL)
#define  RCC_CIR_HSERDYC                     (0x00080000UL)
#define  RCC_CIR_PLLRDYC                     (0x00100000UL)
#define  RCC_CIR_PLLI2SRDYC                  (0x00200000UL)

#define  RCC_CIR_CSSC                        (0x00800000UL)

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define  RCC_AHB1RSTR_GPIOARST               (0x00000001UL)
#define  RCC_AHB1RSTR_GPIOBRST               (0x00000002UL)
#define  RCC_AHB1RSTR_GPIOCRST               (0x00000004UL)
#define  RCC_AHB1RSTR_GPIODRST               (0x00000008UL)
#define  RCC_AHB1RSTR_GPIOERST               (0x00000010UL)
#define  RCC_AHB1RSTR_GPIOHRST               (0x00000080UL)
#define  RCC_AHB1RSTR_CRCRST                 (0x00001000UL)
#define  RCC_AHB1RSTR_DMA1RST                (0x00200000UL)
#define  RCC_AHB1RSTR_DMA2RST                (0x00400000UL)

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define  RCC_AHB2RSTR_OTGFSRST               (0x00000080UL)

/********************  Bit definition for RCC_AHB3RSTR register  **************/

/********************  Bit definition for RCC_APB1RSTR register  **************/
#define  RCC_APB1RSTR_TIM2RST                (0x00000001UL)
#define  RCC_APB1RSTR_TIM3RST                (0x00000002UL)
#define  RCC_APB1RSTR_TIM4RST                (0x00000004UL)
#define  RCC_APB1RSTR_TIM5RST                (0x00000008UL)
#define  RCC_APB1RSTR_WWDGRST                (0x00000800UL)
#define  RCC_APB1RSTR_SPI2RST                (0x00004000UL)
#define  RCC_APB1RSTR_SPI3RST                (0x00008000UL)
#define  RCC_APB1RSTR_USART2RST              (0x00020000UL)
#define  RCC_APB1RSTR_I2C1RST                (0x00200000UL)
#define  RCC_APB1RSTR_I2C2RST                (0x00400000UL)
#define  RCC_APB1RSTR_I2C3RST                (0x00800000UL)
#define  RCC_APB1RSTR_PWRRST                 (0x10000000UL)

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define  RCC_APB2RSTR_TIM1RST                (0x00000001UL)
#define  RCC_APB2RSTR_USART1RST              (0x00000010UL)
#define  RCC_APB2RSTR_USART6RST              (0x00000020UL)
#define  RCC_APB2RSTR_ADCRST                 (0x00000100UL)
#define  RCC_APB2RSTR_SDIORST                (0x00000800UL)
#define  RCC_APB2RSTR_SPI1RST                (0x00001000UL)
#define  RCC_APB2RSTR_SPI4RST                (0x00002000UL)
#define  RCC_APB2RSTR_SYSCFGRST              (0x00004000UL)
#define  RCC_APB2RSTR_TIM9RST                (0x00010000UL)
#define  RCC_APB2RSTR_TIM10RST               (0x00020000UL)
#define  RCC_APB2RSTR_TIM11RST               (0x00040000UL)
#define  RCC_APB2RSTR_SPI5RST                (0x00100000UL)

/* Old SPI1RST bit definition, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                   RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_GPIOAEN                 (0x00000001UL)
#define  RCC_AHB1ENR_GPIOBEN                 (0x00000002UL)
#define  RCC_AHB1ENR_GPIOCEN                 (0x00000004UL)
#define  RCC_AHB1ENR_GPIODEN                 (0x00000008UL)
#define  RCC_AHB1ENR_GPIOEEN                 (0x00000010UL)
#define  RCC_AHB1ENR_GPIOHEN                 (0x00000080UL)
#define  RCC_AHB1ENR_CRCEN                   (0x00001000UL)
#define  RCC_AHB1ENR_BKPSRAMEN               (0x00040000UL)
#define  RCC_AHB1ENR_CCMDATARAMEN            (0x00100000UL)
#define  RCC_AHB1ENR_DMA1EN                  (0x00200000UL)
#define  RCC_AHB1ENR_DMA2EN                  (0x00400000UL)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_OTGFSEN                 (0x00000080UL)

/********************  Bit definition for RCC_AHB3ENR register  ***************/

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define  RCC_APB1ENR_TIM2EN                  (0x00000001UL)
#define  RCC_APB1ENR_TIM3EN                  (0x00000002UL)
#define  RCC_APB1ENR_TIM4EN                  (0x00000004UL)
#define  RCC_APB1ENR_TIM5EN                  (0x00000008UL)
#define  RCC_APB1ENR_WWDGEN                  (0x00000800UL)
#define  RCC_APB1ENR_SPI2EN                  (0x00004000UL)
#define  RCC_APB1ENR_SPI3EN                  (0x00008000UL)
#define  RCC_APB1ENR_USART2EN                (0x00020000UL)
#define  RCC_APB1ENR_I2C1EN                  (0x00200000UL)
#define  RCC_APB1ENR_I2C2EN                  (0x00400000UL)
#define  RCC_APB1ENR_I2C3EN                  (0x00800000UL)
#define  RCC_APB1ENR_PWREN                   (0x10000000UL)

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define  RCC_APB2ENR_TIM1EN                  (0x00000001UL)
#define  RCC_APB2ENR_USART1EN                (0x00000010UL)
#define  RCC_APB2ENR_USART6EN                (0x00000020UL)
#define  RCC_APB2ENR_ADC1EN                  (0x00000100UL)
#define  RCC_APB2ENR_SDIOEN                  (0x00000800UL)
#define  RCC_APB2ENR_SPI1EN                  (0x00001000UL)
#define  RCC_APB2ENR_SPI4EN                  (0x00002000UL)
#define  RCC_APB2ENR_SYSCFGEN                (0x00004000UL)
#define  RCC_APB2ENR_TIM9EN                  (0x00010000UL)
#define  RCC_APB2ENR_TIM10EN                 (0x00020000UL)
#define  RCC_APB2ENR_TIM11EN                 (0x00040000UL)
#define  RCC_APB2ENR_SPI5EN                  (0x00100000UL)

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define  RCC_AHB1LPENR_GPIOALPEN             (0x00000001UL)
#define  RCC_AHB1LPENR_GPIOBLPEN             (0x00000002UL)
#define  RCC_AHB1LPENR_GPIOCLPEN             (0x00000004UL)
#define  RCC_AHB1LPENR_GPIODLPEN             (0x00000008UL)
#define  RCC_AHB1LPENR_GPIOELPEN             (0x00000010UL)
#define  RCC_AHB1LPENR_GPIOHLPEN             (0x00000080UL)
#define  RCC_AHB1LPENR_CRCLPEN               (0x00001000UL)
#define  RCC_AHB1LPENR_FLITFLPEN             (0x00008000UL)
#define  RCC_AHB1LPENR_SRAM1LPEN             (0x00010000UL)
#define  RCC_AHB1LPENR_SRAM2LPEN             (0x00020000UL)
#define  RCC_AHB1LPENR_BKPSRAMLPEN           (0x00040000UL)
#define  RCC_AHB1LPENR_DMA1LPEN              (0x00200000UL)
#define  RCC_AHB1LPENR_DMA2LPEN              (0x00400000UL)

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define  RCC_AHB2LPENR_OTGFSLPEN             (0x00000080UL)

/********************  Bit definition for RCC_AHB3LPENR register  *************/

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define  RCC_APB1LPENR_TIM2LPEN              (0x00000001UL)
#define  RCC_APB1LPENR_TIM3LPEN              (0x00000002UL)
#define  RCC_APB1LPENR_TIM4LPEN              (0x00000004UL)
#define  RCC_APB1LPENR_TIM5LPEN              (0x00000008UL)
#define  RCC_APB1LPENR_WWDGLPEN              (0x00000800UL)
#define  RCC_APB1LPENR_SPI2LPEN              (0x00004000UL)
#define  RCC_APB1LPENR_SPI3LPEN              (0x00008000UL)
#define  RCC_APB1LPENR_USART2LPEN            (0x00020000UL)
#define  RCC_APB1LPENR_I2C1LPEN              (0x00200000UL)
#define  RCC_APB1LPENR_I2C2LPEN              (0x00400000UL)
#define  RCC_APB1LPENR_I2C3LPEN              (0x00800000UL)
#define  RCC_APB1LPENR_PWRLPEN               (0x10000000UL)
#define  RCC_APB1LPENR_DACLPEN               (0x20000000UL)

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define  RCC_APB2LPENR_TIM1LPEN              (0x00000001UL)
#define  RCC_APB2LPENR_USART1LPEN            (0x00000010UL)
#define  RCC_APB2LPENR_USART6LPEN            (0x00000020UL)
#define  RCC_APB2LPENR_ADC1LPEN              (0x00000100UL)
#define  RCC_APB2LPENR_SDIOLPEN              (0x00000800UL)
#define  RCC_APB2LPENR_SPI1LPEN              (0x00001000UL)
#define  RCC_APB2LPENR_SPI4LPEN              (0x00002000UL)
#define  RCC_APB2LPENR_SYSCFGLPEN            (0x00004000UL)
#define  RCC_APB2LPENR_TIM9LPEN              (0x00010000UL)
#define  RCC_APB2LPENR_TIM10LPEN             (0x00020000UL)
#define  RCC_APB2LPENR_TIM11LPEN             (0x00040000UL)
#define  RCC_APB2LPENR_SPI5LPEN              (0x00100000UL)

/********************  Bit definition for RCC_BDCR register  ******************/
#define  RCC_BDCR_LSEON                      (0x00000001UL)
#define  RCC_BDCR_LSERDY                     (0x00000002UL)
#define  RCC_BDCR_LSEBYP                     (0x00000004UL)
#define  RCC_BDCR_LSEMOD                     (0x00000008UL)

#define  RCC_BDCR_RTCSEL                    (0x00000300UL)
#define  RCC_BDCR_RTCSEL_0                  (0x00000100UL)
#define  RCC_BDCR_RTCSEL_1                  (0x00000200UL)

#define  RCC_BDCR_RTCEN                      (0x00008000UL)
#define  RCC_BDCR_BDRST                      (0x00010000UL)

/********************  Bit definition for RCC_CSR register  *******************/
#define  RCC_CSR_LSION                       (0x00000001UL)
#define  RCC_CSR_LSIRDY                      (0x00000002UL)
#define  RCC_CSR_RMVF                        (0x01000000UL)
#define  RCC_CSR_BORRSTF                     (0x02000000UL)
#define  RCC_CSR_PADRSTF                     (0x04000000UL)
#define  RCC_CSR_PORRSTF                     (0x08000000UL)
#define  RCC_CSR_SFTRSTF                     (0x10000000UL)
#define  RCC_CSR_WDGRSTF                     (0x20000000UL)
#define  RCC_CSR_WWDGRSTF                    (0x40000000UL)
#define  RCC_CSR_LPWRRSTF                    (0x80000000UL)

/********************  Bit definition for RCC_SSCGR register  *****************/
#define  RCC_SSCGR_MODPER                    (0x00001FFFUL)
#define  RCC_SSCGR_INCSTEP                   (0x0FFFE000UL)
#define  RCC_SSCGR_SPREADSEL                 (0x40000000UL)
#define  RCC_SSCGR_SSCGEN                    (0x80000000UL)

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define  RCC_PLLI2SCFGR_PLLI2SM              (0x0000003FUL)
#define  RCC_PLLI2SCFGR_PLLI2SM_0            (0x00000001UL)
#define  RCC_PLLI2SCFGR_PLLI2SM_1            (0x00000002UL)
#define  RCC_PLLI2SCFGR_PLLI2SM_2            (0x00000004UL)
#define  RCC_PLLI2SCFGR_PLLI2SM_3            (0x00000008UL)
#define  RCC_PLLI2SCFGR_PLLI2SM_4            (0x00000010UL)
#define  RCC_PLLI2SCFGR_PLLI2SM_5            (0x00000020UL)

#define  RCC_PLLI2SCFGR_PLLI2SN              (0x00007FC0UL)
#define  RCC_PLLI2SCFGR_PLLI2SN_0            (0x00000040UL)
#define  RCC_PLLI2SCFGR_PLLI2SN_1            (0x00000080UL)
#define  RCC_PLLI2SCFGR_PLLI2SN_2            (0x00000100UL)
#define  RCC_PLLI2SCFGR_PLLI2SN_3            (0x00000200UL)
#define  RCC_PLLI2SCFGR_PLLI2SN_4            (0x00000400UL)
#define  RCC_PLLI2SCFGR_PLLI2SN_5            (0x00000800UL)
#define  RCC_PLLI2SCFGR_PLLI2SN_6            (0x00001000UL)
#define  RCC_PLLI2SCFGR_PLLI2SN_7            (0x00002000UL)
#define  RCC_PLLI2SCFGR_PLLI2SN_8            (0x00004000UL)

#define  RCC_PLLI2SCFGR_PLLI2SR              (0x70000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SR_0            (0x10000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SR_1            (0x20000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SR_2            (0x40000000UL)


/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTCUL)                            */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RTC_TR register  *******************/
#define RTC_TR_PM                            (0x00400000UL)
#define RTC_TR_HT                            (0x00300000UL)
#define RTC_TR_HT_0                          (0x00100000UL)
#define RTC_TR_HT_1                          (0x00200000UL)
#define RTC_TR_HU                            (0x000F0000UL)
#define RTC_TR_HU_0                          (0x00010000UL)
#define RTC_TR_HU_1                          (0x00020000UL)
#define RTC_TR_HU_2                          (0x00040000UL)
#define RTC_TR_HU_3                          (0x00080000UL)
#define RTC_TR_MNT                           (0x00007000UL)
#define RTC_TR_MNT_0                         (0x00001000UL)
#define RTC_TR_MNT_1                         (0x00002000UL)
#define RTC_TR_MNT_2                         (0x00004000UL)
#define RTC_TR_MNU                           (0x00000F00UL)
#define RTC_TR_MNU_0                         (0x00000100UL)
#define RTC_TR_MNU_1                         (0x00000200UL)
#define RTC_TR_MNU_2                         (0x00000400UL)
#define RTC_TR_MNU_3                         (0x00000800UL)
#define RTC_TR_ST                            (0x00000070UL)
#define RTC_TR_ST_0                          (0x00000010UL)
#define RTC_TR_ST_1                          (0x00000020UL)
#define RTC_TR_ST_2                          (0x00000040UL)
#define RTC_TR_SU                            (0x0000000FUL)
#define RTC_TR_SU_0                          (0x00000001UL)
#define RTC_TR_SU_1                          (0x00000002UL)
#define RTC_TR_SU_2                          (0x00000004UL)
#define RTC_TR_SU_3                          (0x00000008UL)

/********************  Bits definition for RTC_DR register  *******************/
#define RTC_DR_YT                            (0x00F00000UL)
#define RTC_DR_YT_0                          (0x00100000UL)
#define RTC_DR_YT_1                          (0x00200000UL)
#define RTC_DR_YT_2                          (0x00400000UL)
#define RTC_DR_YT_3                          (0x00800000UL)
#define RTC_DR_YU                            (0x000F0000UL)
#define RTC_DR_YU_0                          (0x00010000UL)
#define RTC_DR_YU_1                          (0x00020000UL)
#define RTC_DR_YU_2                          (0x00040000UL)
#define RTC_DR_YU_3                          (0x00080000UL)
#define RTC_DR_WDU                           (0x0000E000UL)
#define RTC_DR_WDU_0                         (0x00002000UL)
#define RTC_DR_WDU_1                         (0x00004000UL)
#define RTC_DR_WDU_2                         (0x00008000UL)
#define RTC_DR_MT                            (0x00001000UL)
#define RTC_DR_MU                            (0x00000F00UL)
#define RTC_DR_MU_0                          (0x00000100UL)
#define RTC_DR_MU_1                          (0x00000200UL)
#define RTC_DR_MU_2                          (0x00000400UL)
#define RTC_DR_MU_3                          (0x00000800UL)
#define RTC_DR_DT                            (0x00000030UL)
#define RTC_DR_DT_0                          (0x00000010UL)
#define RTC_DR_DT_1                          (0x00000020UL)
#define RTC_DR_DU                            (0x0000000FUL)
#define RTC_DR_DU_0                          (0x00000001UL)
#define RTC_DR_DU_1                          (0x00000002UL)
#define RTC_DR_DU_2                          (0x00000004UL)
#define RTC_DR_DU_3                          (0x00000008UL)

/********************  Bits definition for RTC_CR register  *******************/
#define RTC_CR_COE                           (0x00800000UL)
#define RTC_CR_OSEL                          (0x00600000UL)
#define RTC_CR_OSEL_0                        (0x00200000UL)
#define RTC_CR_OSEL_1                        (0x00400000UL)
#define RTC_CR_POL                           (0x00100000UL)
#define RTC_CR_COSEL                         (0x00080000UL)
#define RTC_CR_BCK                           (0x00040000UL)
#define RTC_CR_SUB1H                         (0x00020000UL)
#define RTC_CR_ADD1H                         (0x00010000UL)
#define RTC_CR_TSIE                          (0x00008000UL)
#define RTC_CR_WUTIE                         (0x00004000UL)
#define RTC_CR_ALRBIE                        (0x00002000UL)
#define RTC_CR_ALRAIE                        (0x00001000UL)
#define RTC_CR_TSE                           (0x00000800UL)
#define RTC_CR_WUTE                          (0x00000400UL)
#define RTC_CR_ALRBE                         (0x00000200UL)
#define RTC_CR_ALRAE                         (0x00000100UL)
#define RTC_CR_DCE                           (0x00000080UL)
#define RTC_CR_FMT                           (0x00000040UL)
#define RTC_CR_BYPSHAD                       (0x00000020UL)
#define RTC_CR_REFCKON                       (0x00000010UL)
#define RTC_CR_TSEDGE                        (0x00000008UL)
#define RTC_CR_WUCKSEL                       (0x00000007UL)
#define RTC_CR_WUCKSEL_0                     (0x00000001UL)
#define RTC_CR_WUCKSEL_1                     (0x00000002UL)
#define RTC_CR_WUCKSEL_2                     (0x00000004UL)

/********************  Bits definition for RTC_ISR register  ******************/
#define RTC_ISR_RECALPF                      (0x00010000UL)
#define RTC_ISR_TAMP1F                       (0x00002000UL)
#define RTC_ISR_TAMP2F                       (0x00004000UL)
#define RTC_ISR_TSOVF                        (0x00001000UL)
#define RTC_ISR_TSF                          (0x00000800UL)
#define RTC_ISR_WUTF                         (0x00000400UL)
#define RTC_ISR_ALRBF                        (0x00000200UL)
#define RTC_ISR_ALRAF                        (0x00000100UL)
#define RTC_ISR_INIT                         (0x00000080UL)
#define RTC_ISR_INITF                        (0x00000040UL)
#define RTC_ISR_RSF                          (0x00000020UL)
#define RTC_ISR_INITS                        (0x00000010UL)
#define RTC_ISR_SHPF                         (0x00000008UL)
#define RTC_ISR_WUTWF                        (0x00000004UL)
#define RTC_ISR_ALRBWF                       (0x00000002UL)
#define RTC_ISR_ALRAWF                       (0x00000001UL)

/********************  Bits definition for RTC_PRER register  *****************/
#define RTC_PRER_PREDIV_A                    (0x007F0000UL)
#define RTC_PRER_PREDIV_S                    (0x00007FFFUL)

/********************  Bits definition for RTC_WUTR register  *****************/
#define RTC_WUTR_WUT                         (0x0000FFFFUL)

/********************  Bits definition for RTC_CALIBR register  ***************/
#define RTC_CALIBR_DCS                       (0x00000080UL)
#define RTC_CALIBR_DC                        (0x0000001FUL)

/********************  Bits definition for RTC_ALRMAR register  ***************/
#define RTC_ALRMAR_MSK4                      (0x80000000UL)
#define RTC_ALRMAR_WDSEL                     (0x40000000UL)
#define RTC_ALRMAR_DT                        (0x30000000UL)
#define RTC_ALRMAR_DT_0                      (0x10000000UL)
#define RTC_ALRMAR_DT_1                      (0x20000000UL)
#define RTC_ALRMAR_DU                        (0x0F000000UL)
#define RTC_ALRMAR_DU_0                      (0x01000000UL)
#define RTC_ALRMAR_DU_1                      (0x02000000UL)
#define RTC_ALRMAR_DU_2                      (0x04000000UL)
#define RTC_ALRMAR_DU_3                      (0x08000000UL)
#define RTC_ALRMAR_MSK3                      (0x00800000UL)
#define RTC_ALRMAR_PM                        (0x00400000UL)
#define RTC_ALRMAR_HT                        (0x00300000UL)
#define RTC_ALRMAR_HT_0                      (0x00100000UL)
#define RTC_ALRMAR_HT_1                      (0x00200000UL)
#define RTC_ALRMAR_HU                        (0x000F0000UL)
#define RTC_ALRMAR_HU_0                      (0x00010000UL)
#define RTC_ALRMAR_HU_1                      (0x00020000UL)
#define RTC_ALRMAR_HU_2                      (0x00040000UL)
#define RTC_ALRMAR_HU_3                      (0x00080000UL)
#define RTC_ALRMAR_MSK2                      (0x00008000UL)
#define RTC_ALRMAR_MNT                       (0x00007000UL)
#define RTC_ALRMAR_MNT_0                     (0x00001000UL)
#define RTC_ALRMAR_MNT_1                     (0x00002000UL)
#define RTC_ALRMAR_MNT_2                     (0x00004000UL)
#define RTC_ALRMAR_MNU                       (0x00000F00UL)
#define RTC_ALRMAR_MNU_0                     (0x00000100UL)
#define RTC_ALRMAR_MNU_1                     (0x00000200UL)
#define RTC_ALRMAR_MNU_2                     (0x00000400UL)
#define RTC_ALRMAR_MNU_3                     (0x00000800UL)
#define RTC_ALRMAR_MSK1                      (0x00000080UL)
#define RTC_ALRMAR_ST                        (0x00000070UL)
#define RTC_ALRMAR_ST_0                      (0x00000010UL)
#define RTC_ALRMAR_ST_1                      (0x00000020UL)
#define RTC_ALRMAR_ST_2                      (0x00000040UL)
#define RTC_ALRMAR_SU                        (0x0000000FUL)
#define RTC_ALRMAR_SU_0                      (0x00000001UL)
#define RTC_ALRMAR_SU_1                      (0x00000002UL)
#define RTC_ALRMAR_SU_2                      (0x00000004UL)
#define RTC_ALRMAR_SU_3                      (0x00000008UL)

/********************  Bits definition for RTC_ALRMBR register  ***************/
#define RTC_ALRMBR_MSK4                      (0x80000000UL)
#define RTC_ALRMBR_WDSEL                     (0x40000000UL)
#define RTC_ALRMBR_DT                        (0x30000000UL)
#define RTC_ALRMBR_DT_0                      (0x10000000UL)
#define RTC_ALRMBR_DT_1                      (0x20000000UL)
#define RTC_ALRMBR_DU                        (0x0F000000UL)
#define RTC_ALRMBR_DU_0                      (0x01000000UL)
#define RTC_ALRMBR_DU_1                      (0x02000000UL)
#define RTC_ALRMBR_DU_2                      (0x04000000UL)
#define RTC_ALRMBR_DU_3                      (0x08000000UL)
#define RTC_ALRMBR_MSK3                      (0x00800000UL)
#define RTC_ALRMBR_PM                        (0x00400000UL)
#define RTC_ALRMBR_HT                        (0x00300000UL)
#define RTC_ALRMBR_HT_0                      (0x00100000UL)
#define RTC_ALRMBR_HT_1                      (0x00200000UL)
#define RTC_ALRMBR_HU                        (0x000F0000UL)
#define RTC_ALRMBR_HU_0                      (0x00010000UL)
#define RTC_ALRMBR_HU_1                      (0x00020000UL)
#define RTC_ALRMBR_HU_2                      (0x00040000UL)
#define RTC_ALRMBR_HU_3                      (0x00080000UL)
#define RTC_ALRMBR_MSK2                      (0x00008000UL)
#define RTC_ALRMBR_MNT                       (0x00007000UL)
#define RTC_ALRMBR_MNT_0                     (0x00001000UL)
#define RTC_ALRMBR_MNT_1                     (0x00002000UL)
#define RTC_ALRMBR_MNT_2                     (0x00004000UL)
#define RTC_ALRMBR_MNU                       (0x00000F00UL)
#define RTC_ALRMBR_MNU_0                     (0x00000100UL)
#define RTC_ALRMBR_MNU_1                     (0x00000200UL)
#define RTC_ALRMBR_MNU_2                     (0x00000400UL)
#define RTC_ALRMBR_MNU_3                     (0x00000800UL)
#define RTC_ALRMBR_MSK1                      (0x00000080UL)
#define RTC_ALRMBR_ST                        (0x00000070UL)
#define RTC_ALRMBR_ST_0                      (0x00000010UL)
#define RTC_ALRMBR_ST_1                      (0x00000020UL)
#define RTC_ALRMBR_ST_2                      (0x00000040UL)
#define RTC_ALRMBR_SU                        (0x0000000FUL)
#define RTC_ALRMBR_SU_0                      (0x00000001UL)
#define RTC_ALRMBR_SU_1                      (0x00000002UL)
#define RTC_ALRMBR_SU_2                      (0x00000004UL)
#define RTC_ALRMBR_SU_3                      (0x00000008UL)

/********************  Bits definition for RTC_WPR register  ******************/
#define RTC_WPR_KEY                          (0x000000FFUL)

/********************  Bits definition for RTC_SSR register  ******************/
#define RTC_SSR_SS                           (0x0000FFFFUL)

/********************  Bits definition for RTC_SHIFTR register  ***************/
#define RTC_SHIFTR_SUBFS                     (0x00007FFFUL)
#define RTC_SHIFTR_ADD1S                     (0x80000000UL)

/********************  Bits definition for RTC_TSTR register  *****************/
#define RTC_TSTR_PM                          (0x00400000UL)
#define RTC_TSTR_HT                          (0x00300000UL)
#define RTC_TSTR_HT_0                        (0x00100000UL)
#define RTC_TSTR_HT_1                        (0x00200000UL)
#define RTC_TSTR_HU                          (0x000F0000UL)
#define RTC_TSTR_HU_0                        (0x00010000UL)
#define RTC_TSTR_HU_1                        (0x00020000UL)
#define RTC_TSTR_HU_2                        (0x00040000UL)
#define RTC_TSTR_HU_3                        (0x00080000UL)
#define RTC_TSTR_MNT                         (0x00007000UL)
#define RTC_TSTR_MNT_0                       (0x00001000UL)
#define RTC_TSTR_MNT_1                       (0x00002000UL)
#define RTC_TSTR_MNT_2                       (0x00004000UL)
#define RTC_TSTR_MNU                         (0x00000F00UL)
#define RTC_TSTR_MNU_0                       (0x00000100UL)
#define RTC_TSTR_MNU_1                       (0x00000200UL)
#define RTC_TSTR_MNU_2                       (0x00000400UL)
#define RTC_TSTR_MNU_3                       (0x00000800UL)
#define RTC_TSTR_ST                          (0x00000070UL)
#define RTC_TSTR_ST_0                        (0x00000010UL)
#define RTC_TSTR_ST_1                        (0x00000020UL)
#define RTC_TSTR_ST_2                        (0x00000040UL)
#define RTC_TSTR_SU                          (0x0000000FUL)
#define RTC_TSTR_SU_0                        (0x00000001UL)
#define RTC_TSTR_SU_1                        (0x00000002UL)
#define RTC_TSTR_SU_2                        (0x00000004UL)
#define RTC_TSTR_SU_3                        (0x00000008UL)

/********************  Bits definition for RTC_TSDR register  *****************/
#define RTC_TSDR_WDU                         (0x0000E000UL)
#define RTC_TSDR_WDU_0                       (0x00002000UL)
#define RTC_TSDR_WDU_1                       (0x00004000UL)
#define RTC_TSDR_WDU_2                       (0x00008000UL)
#define RTC_TSDR_MT                          (0x00001000UL)
#define RTC_TSDR_MU                          (0x00000F00UL)
#define RTC_TSDR_MU_0                        (0x00000100UL)
#define RTC_TSDR_MU_1                        (0x00000200UL)
#define RTC_TSDR_MU_2                        (0x00000400UL)
#define RTC_TSDR_MU_3                        (0x00000800UL)
#define RTC_TSDR_DT                          (0x00000030UL)
#define RTC_TSDR_DT_0                        (0x00000010UL)
#define RTC_TSDR_DT_1                        (0x00000020UL)
#define RTC_TSDR_DU                          (0x0000000FUL)
#define RTC_TSDR_DU_0                        (0x00000001UL)
#define RTC_TSDR_DU_1                        (0x00000002UL)
#define RTC_TSDR_DU_2                        (0x00000004UL)
#define RTC_TSDR_DU_3                        (0x00000008UL)

/********************  Bits definition for RTC_TSSSR register  ****************/
#define RTC_TSSSR_SS                         (0x0000FFFFUL)

/********************  Bits definition for RTC_CAL register  *****************/
#define RTC_CALR_CALP                        (0x00008000UL)
#define RTC_CALR_CALW8                       (0x00004000UL)
#define RTC_CALR_CALW16                      (0x00002000UL)
#define RTC_CALR_CALM                        (0x000001FFUL)
#define RTC_CALR_CALM_0                      (0x00000001UL)
#define RTC_CALR_CALM_1                      (0x00000002UL)
#define RTC_CALR_CALM_2                      (0x00000004UL)
#define RTC_CALR_CALM_3                      (0x00000008UL)
#define RTC_CALR_CALM_4                      (0x00000010UL)
#define RTC_CALR_CALM_5                      (0x00000020UL)
#define RTC_CALR_CALM_6                      (0x00000040UL)
#define RTC_CALR_CALM_7                      (0x00000080UL)
#define RTC_CALR_CALM_8                      (0x00000100UL)

/********************  Bits definition for RTC_TAFCR register  ****************/
#define RTC_TAFCR_ALARMOUTTYPE               (0x00040000UL)
#define RTC_TAFCR_TSINSEL                    (0x00020000UL)
#define RTC_TAFCR_TAMPINSEL                  (0x00010000UL)
#define RTC_TAFCR_TAMPPUDIS                  (0x00008000UL)
#define RTC_TAFCR_TAMPPRCH                   (0x00006000UL)
#define RTC_TAFCR_TAMPPRCH_0                 (0x00002000UL)
#define RTC_TAFCR_TAMPPRCH_1                 (0x00004000UL)
#define RTC_TAFCR_TAMPFLT                    (0x00001800UL)
#define RTC_TAFCR_TAMPFLT_0                  (0x00000800UL)
#define RTC_TAFCR_TAMPFLT_1                  (0x00001000UL)
#define RTC_TAFCR_TAMPFREQ                   (0x00000700UL)
#define RTC_TAFCR_TAMPFREQ_0                 (0x00000100UL)
#define RTC_TAFCR_TAMPFREQ_1                 (0x00000200UL)
#define RTC_TAFCR_TAMPFREQ_2                 (0x00000400UL)
#define RTC_TAFCR_TAMPTS                     (0x00000080UL)
#define RTC_TAFCR_TAMP2TRG                   (0x00000010UL)
#define RTC_TAFCR_TAMP2E                     (0x00000008UL)
#define RTC_TAFCR_TAMPIE                     (0x00000004UL)
#define RTC_TAFCR_TAMP1TRG                   (0x00000002UL)
#define RTC_TAFCR_TAMP1E                     (0x00000001UL)

/********************  Bits definition for RTC_ALRMASSR register  *************/
#define RTC_ALRMASSR_MASKSS                  (0x0F000000UL)
#define RTC_ALRMASSR_MASKSS_0                (0x01000000UL)
#define RTC_ALRMASSR_MASKSS_1                (0x02000000UL)
#define RTC_ALRMASSR_MASKSS_2                (0x04000000UL)
#define RTC_ALRMASSR_MASKSS_3                (0x08000000UL)
#define RTC_ALRMASSR_SS                      (0x00007FFFUL)

/********************  Bits definition for RTC_ALRMBSSR register  *************/
#define RTC_ALRMBSSR_MASKSS                  (0x0F000000UL)
#define RTC_ALRMBSSR_MASKSS_0                (0x01000000UL)
#define RTC_ALRMBSSR_MASKSS_1                (0x02000000UL)
#define RTC_ALRMBSSR_MASKSS_2                (0x04000000UL)
#define RTC_ALRMBSSR_MASKSS_3                (0x08000000UL)
#define RTC_ALRMBSSR_SS                      (0x00007FFFUL)

/********************  Bits definition for RTC_BKP0R register  ****************/
#define RTC_BKP0R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP1R register  ****************/
#define RTC_BKP1R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP2R register  ****************/
#define RTC_BKP2R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP3R register  ****************/
#define RTC_BKP3R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP4R register  ****************/
#define RTC_BKP4R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP5R register  ****************/
#define RTC_BKP5R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP6R register  ****************/
#define RTC_BKP6R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP7R register  ****************/
#define RTC_BKP7R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP8R register  ****************/
#define RTC_BKP8R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP9R register  ****************/
#define RTC_BKP9R                            (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP10R register  ***************/
#define RTC_BKP10R                           (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP11R register  ***************/
#define RTC_BKP11R                           (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP12R register  ***************/
#define RTC_BKP12R                           (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP13R register  ***************/
#define RTC_BKP13R                           (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP14R register  ***************/
#define RTC_BKP14R                           (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP15R register  ***************/
#define RTC_BKP15R                           (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP16R register  ***************/
#define RTC_BKP16R                           (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP17R register  ***************/
#define RTC_BKP17R                           (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP18R register  ***************/
#define RTC_BKP18R                           (0xFFFFFFFFUL)

/********************  Bits definition for RTC_BKP19R register  ***************/
#define RTC_BKP19R                           (0xFFFFFFFFUL)



/******************************************************************************/
/*                                                                            */
/*                          SD host Interface                                 */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SDIO_POWER register  ******************/
#define  SDIO_POWER_PWRCTRL                  (0x03UL)               /*!<PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDIO_POWER_PWRCTRL_0                (0x01UL)               /*!<Bit 0 */
#define  SDIO_POWER_PWRCTRL_1                (0x02UL)               /*!<Bit 1 */

/******************  Bit definition for SDIO_CLKCR register  ******************/
#define  SDIO_CLKCR_CLKDIV                   (0x00FFUL)            /*!<Clock divide factor             */
#define  SDIO_CLKCR_CLKEN                    (0x0100UL)            /*!<Clock enable bit                */
#define  SDIO_CLKCR_PWRSAV                   (0x0200UL)            /*!<Power saving configuration bit  */
#define  SDIO_CLKCR_BYPASS                   (0x0400UL)            /*!<Clock divider bypass enable bit */

#define  SDIO_CLKCR_WIDBUS                   (0x1800UL)            /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDIO_CLKCR_WIDBUS_0                 (0x0800UL)            /*!<Bit 0 */
#define  SDIO_CLKCR_WIDBUS_1                 (0x1000UL)            /*!<Bit 1 */

#define  SDIO_CLKCR_NEGEDGE                  (0x2000UL)            /*!<SDIO_CK dephasing selection bit */
#define  SDIO_CLKCR_HWFC_EN                  (0x4000UL)            /*!<HW Flow Control enable          */

/*******************  Bit definition for SDIO_ARG register  *******************/
#define  SDIO_ARG_CMDARG                     (0xFFFFFFFFUL)            /*!<Command argument */

/*******************  Bit definition for SDIO_CMD register  *******************/
#define  SDIO_CMD_CMDINDEX                   (0x003FUL)            /*!<Command Index                               */

#define  SDIO_CMD_WAITRESP                   (0x00C0UL)            /*!<WAITRESP[1:0] bits (Wait for response bits) */
#define  SDIO_CMD_WAITRESP_0                 (0x0040UL)            /*!< Bit 0 */
#define  SDIO_CMD_WAITRESP_1                 (0x0080UL)            /*!< Bit 1 */

#define  SDIO_CMD_WAITINT                    (0x0100UL)            /*!<CPSM Waits for Interrupt Request                               */
#define  SDIO_CMD_WAITPEND                   (0x0200UL)            /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDIO_CMD_CPSMEN                     (0x0400UL)            /*!<Command path state machine (CPSM) Enable bit                   */
#define  SDIO_CMD_SDIOSUSPEND                (0x0800UL)            /*!<SD I/O suspend command                                         */
#define  SDIO_CMD_ENCMDCOMPL                 (0x1000UL)            /*!<Enable CMD completion                                          */
#define  SDIO_CMD_NIEN                       (0x2000UL)            /*!<Not Interrupt Enable */
#define  SDIO_CMD_CEATACMD                   (0x4000UL)            /*!<CE-ATA command       */

/*****************  Bit definition for SDIO_RESPCMD register  *****************/
#define  SDIO_RESPCMD_RESPCMD                (0x3FUL)               /*!<Response command index */

/******************  Bit definition for SDIO_RESP0 register  ******************/
#define  SDIO_RESP0_CARDSTATUS0              (0xFFFFFFFFUL)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP1 register  ******************/
#define  SDIO_RESP1_CARDSTATUS1              (0xFFFFFFFFUL)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP2 register  ******************/
#define  SDIO_RESP2_CARDSTATUS2              (0xFFFFFFFFUL)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP3 register  ******************/
#define  SDIO_RESP3_CARDSTATUS3              (0xFFFFFFFFUL)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP4 register  ******************/
#define  SDIO_RESP4_CARDSTATUS4              (0xFFFFFFFFUL)        /*!<Card Status */

/******************  Bit definition for SDIO_DTIMER register  *****************/
#define  SDIO_DTIMER_DATATIME                (0xFFFFFFFFUL)        /*!<Data timeout period. */

/******************  Bit definition for SDIO_DLEN register  *******************/
#define  SDIO_DLEN_DATALENGTH                (0x01FFFFFFUL)        /*!<Data length value    */

/******************  Bit definition for SDIO_DCTRL register  ******************/
#define  SDIO_DCTRL_DTEN                     (0x0001UL)            /*!<Data transfer enabled bit         */
#define  SDIO_DCTRL_DTDIR                    (0x0002UL)            /*!<Data transfer direction selection */
#define  SDIO_DCTRL_DTMODE                   (0x0004UL)            /*!<Data transfer mode selection      */
#define  SDIO_DCTRL_DMAEN                    (0x0008UL)            /*!<DMA enabled bit                   */

#define  SDIO_DCTRL_DBLOCKSIZE               (0x00F0UL)            /*!<DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDIO_DCTRL_DBLOCKSIZE_0             (0x0010UL)            /*!<Bit 0 */
#define  SDIO_DCTRL_DBLOCKSIZE_1             (0x0020UL)            /*!<Bit 1 */
#define  SDIO_DCTRL_DBLOCKSIZE_2             (0x0040UL)            /*!<Bit 2 */
#define  SDIO_DCTRL_DBLOCKSIZE_3             (0x0080UL)            /*!<Bit 3 */

#define  SDIO_DCTRL_RWSTART                  (0x0100UL)            /*!<Read wait start         */
#define  SDIO_DCTRL_RWSTOP                   (0x0200UL)            /*!<Read wait stop          */
#define  SDIO_DCTRL_RWMOD                    (0x0400UL)            /*!<Read wait mode          */
#define  SDIO_DCTRL_SDIOEN                   (0x0800UL)            /*!<SD I/O enable functions */

/******************  Bit definition for SDIO_DCOUNT register  *****************/
#define  SDIO_DCOUNT_DATACOUNT               (0x01FFFFFFUL)        /*!<Data count value */

/******************  Bit definition for SDIO_STA register  ********************/
#define  SDIO_STA_CCRCFAIL                   (0x00000001UL)        /*!<Command response received (CRC check failed)  */
#define  SDIO_STA_DCRCFAIL                   (0x00000002UL)        /*!<Data block sent/received (CRC check failed)   */
#define  SDIO_STA_CTIMEOUT                   (0x00000004UL)        /*!<Command response timeout                      */
#define  SDIO_STA_DTIMEOUT                   (0x00000008UL)        /*!<Data timeout                                  */
#define  SDIO_STA_TXUNDERR                   (0x00000010UL)        /*!<Transmit FIFO underrun error                  */
#define  SDIO_STA_RXOVERR                    (0x00000020UL)        /*!<Received FIFO overrun error                   */
#define  SDIO_STA_CMDREND                    (0x00000040UL)        /*!<Command response received (CRC check passed)  */
#define  SDIO_STA_CMDSENT                    (0x00000080UL)        /*!<Command sent (no response required)           */
#define  SDIO_STA_DATAEND                    (0x00000100UL)        /*!<Data end (data counter, SDIDCOUNT, is zero)   */
#define  SDIO_STA_STBITERR                   (0x00000200UL)        /*!<Start bit not detected on all data signals in wide bus mode */
#define  SDIO_STA_DBCKEND                    (0x00000400UL)        /*!<Data block sent/received (CRC check passed)   */
#define  SDIO_STA_CMDACT                     (0x00000800UL)        /*!<Command transfer in progress                  */
#define  SDIO_STA_TXACT                      (0x00001000UL)        /*!<Data transmit in progress                     */
#define  SDIO_STA_RXACT                      (0x00002000UL)        /*!<Data receive in progress                      */
#define  SDIO_STA_TXFIFOHE                   (0x00004000UL)        /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  SDIO_STA_RXFIFOHF                   (0x00008000UL)        /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  SDIO_STA_TXFIFOF                    (0x00010000UL)        /*!<Transmit FIFO full                            */
#define  SDIO_STA_RXFIFOF                    (0x00020000UL)        /*!<Receive FIFO full                             */
#define  SDIO_STA_TXFIFOE                    (0x00040000UL)        /*!<Transmit FIFO empty                           */
#define  SDIO_STA_RXFIFOE                    (0x00080000UL)        /*!<Receive FIFO empty                            */
#define  SDIO_STA_TXDAVL                     (0x00100000UL)        /*!<Data available in transmit FIFO               */
#define  SDIO_STA_RXDAVL                     (0x00200000UL)        /*!<Data available in receive FIFO                */
#define  SDIO_STA_SDIOIT                     (0x00400000UL)        /*!<SDIO interrupt received                       */
#define  SDIO_STA_CEATAEND                   (0x00800000UL)        /*!<CE-ATA command completion signal received for CMD61 */

/*******************  Bit definition for SDIO_ICR register  *******************/
#define  SDIO_ICR_CCRCFAILC                  (0x00000001UL)        /*!<CCRCFAIL flag clear bit */
#define  SDIO_ICR_DCRCFAILC                  (0x00000002UL)        /*!<DCRCFAIL flag clear bit */
#define  SDIO_ICR_CTIMEOUTC                  (0x00000004UL)        /*!<CTIMEOUT flag clear bit */
#define  SDIO_ICR_DTIMEOUTC                  (0x00000008UL)        /*!<DTIMEOUT flag clear bit */
#define  SDIO_ICR_TXUNDERRC                  (0x00000010UL)        /*!<TXUNDERR flag clear bit */
#define  SDIO_ICR_RXOVERRC                   (0x00000020UL)        /*!<RXOVERR flag clear bit  */
#define  SDIO_ICR_CMDRENDC                   (0x00000040UL)        /*!<CMDREND flag clear bit  */
#define  SDIO_ICR_CMDSENTC                   (0x00000080UL)        /*!<CMDSENT flag clear bit  */
#define  SDIO_ICR_DATAENDC                   (0x00000100UL)        /*!<DATAEND flag clear bit  */
#define  SDIO_ICR_STBITERRC                  (0x00000200UL)        /*!<STBITERR flag clear bit */
#define  SDIO_ICR_DBCKENDC                   (0x00000400UL)        /*!<DBCKEND flag clear bit  */
#define  SDIO_ICR_SDIOITC                    (0x00400000UL)        /*!<SDIOIT flag clear bit   */
#define  SDIO_ICR_CEATAENDC                  (0x00800000UL)        /*!<CEATAEND flag clear bit */

/******************  Bit definition for SDIO_MASK register  *******************/
#define  SDIO_MASK_CCRCFAILIE                (0x00000001UL)        /*!<Command CRC Fail Interrupt Enable          */
#define  SDIO_MASK_DCRCFAILIE                (0x00000002UL)        /*!<Data CRC Fail Interrupt Enable             */
#define  SDIO_MASK_CTIMEOUTIE                (0x00000004UL)        /*!<Command TimeOut Interrupt Enable           */
#define  SDIO_MASK_DTIMEOUTIE                (0x00000008UL)        /*!<Data TimeOut Interrupt Enable              */
#define  SDIO_MASK_TXUNDERRIE                (0x00000010UL)        /*!<Tx FIFO UnderRun Error Interrupt Enable    */
#define  SDIO_MASK_RXOVERRIE                 (0x00000020UL)        /*!<Rx FIFO OverRun Error Interrupt Enable     */
#define  SDIO_MASK_CMDRENDIE                 (0x00000040UL)        /*!<Command Response Received Interrupt Enable */
#define  SDIO_MASK_CMDSENTIE                 (0x00000080UL)        /*!<Command Sent Interrupt Enable              */
#define  SDIO_MASK_DATAENDIE                 (0x00000100UL)        /*!<Data End Interrupt Enable                  */
#define  SDIO_MASK_STBITERRIE                (0x00000200UL)        /*!<Start Bit Error Interrupt Enable           */
#define  SDIO_MASK_DBCKENDIE                 (0x00000400UL)        /*!<Data Block End Interrupt Enable            */
#define  SDIO_MASK_CMDACTIE                  (0x00000800UL)        /*!<CCommand Acting Interrupt Enable           */
#define  SDIO_MASK_TXACTIE                   (0x00001000UL)        /*!<Data Transmit Acting Interrupt Enable      */
#define  SDIO_MASK_RXACTIE                   (0x00002000UL)        /*!<Data receive acting interrupt enabled      */
#define  SDIO_MASK_TXFIFOHEIE                (0x00004000UL)        /*!<Tx FIFO Half Empty interrupt Enable        */
#define  SDIO_MASK_RXFIFOHFIE                (0x00008000UL)        /*!<Rx FIFO Half Full interrupt Enable         */
#define  SDIO_MASK_TXFIFOFIE                 (0x00010000UL)        /*!<Tx FIFO Full interrupt Enable              */
#define  SDIO_MASK_RXFIFOFIE                 (0x00020000UL)        /*!<Rx FIFO Full interrupt Enable              */
#define  SDIO_MASK_TXFIFOEIE                 (0x00040000UL)        /*!<Tx FIFO Empty interrupt Enable             */
#define  SDIO_MASK_RXFIFOEIE                 (0x00080000UL)        /*!<Rx FIFO Empty interrupt Enable             */
#define  SDIO_MASK_TXDAVLIE                  (0x00100000UL)        /*!<Data available in Tx FIFO interrupt Enable */
#define  SDIO_MASK_RXDAVLIE                  (0x00200000UL)        /*!<Data available in Rx FIFO interrupt Enable */
#define  SDIO_MASK_SDIOITIE                  (0x00400000UL)        /*!<SDIO Mode Interrupt Received interrupt Enable */
#define  SDIO_MASK_CEATAENDIE                (0x00800000UL)        /*!<CE-ATA command completion signal received Interrupt Enable */

/*****************  Bit definition for SDIO_FIFOCNT register  *****************/
#define  SDIO_FIFOCNT_FIFOCOUNT              (0x00FFFFFFUL)        /*!<Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDIO_FIFO register  *******************/
#define  SDIO_FIFO_FIFODATA                  (0xFFFFFFFFUL)        /*!<Receive and transmit FIFO data */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        (0x00000001UL)            /*!<Clock Phase      */
#define  SPI_CR1_CPOL                        (0x00000002UL)            /*!<Clock Polarity   */
#define  SPI_CR1_MSTR                        (0x00000004UL)            /*!<Master Selection */

#define  SPI_CR1_BR                          (0x00000038UL)            /*!<BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        (0x00000008UL)            /*!<Bit 0 */
#define  SPI_CR1_BR_1                        (0x00000010UL)            /*!<Bit 1 */
#define  SPI_CR1_BR_2                        (0x00000020UL)            /*!<Bit 2 */

#define  SPI_CR1_SPE                         (0x00000040UL)            /*!<SPI Enable                          */
#define  SPI_CR1_LSBFIRST                    (0x00000080UL)            /*!<Frame Format                        */
#define  SPI_CR1_SSI                         (0x00000100UL)            /*!<Internal slave select               */
#define  SPI_CR1_SSM                         (0x00000200UL)            /*!<Software slave management           */
#define  SPI_CR1_RXONLY                      (0x00000400UL)            /*!<Receive only                        */
#define  SPI_CR1_DFF                         (0x00000800UL)            /*!<Data Frame Format                   */
#define  SPI_CR1_CRCNEXT                     (0x00001000UL)            /*!<Transmit CRC next                   */
#define  SPI_CR1_CRCEN                       (0x00002000UL)            /*!<Hardware CRC calculation enable     */
#define  SPI_CR1_BIDIOE                      (0x00004000UL)            /*!<Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    (0x00008000UL)            /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     (0x00000001UL)               /*!<Rx Buffer DMA Enable                 */
#define  SPI_CR2_TXDMAEN                     (0x00000002UL)               /*!<Tx Buffer DMA Enable                 */
#define  SPI_CR2_SSOE                        (0x00000004UL)               /*!<SS Output Enable                     */
#define  SPI_CR2_FRF                         (0x00000010UL)               /*!<Frame Format                         */
#define  SPI_CR2_ERRIE                       (0x00000020UL)               /*!<Error Interrupt Enable               */
#define  SPI_CR2_RXNEIE                      (0x00000040UL)               /*!<RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       (0x00000080UL)               /*!<Tx buffer Empty Interrupt Enable     */

/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         (0x00000001UL)               /*!<Receive buffer Not Empty */
#define  SPI_SR_TXE                          (0x00000002UL)               /*!<Transmit buffer Empty    */
#define  SPI_SR_CHSIDE                       (0x00000004UL)               /*!<Channel side             */
#define  SPI_SR_UDR                          (0x00000008UL)               /*!<Underrun flag            */
#define  SPI_SR_CRCERR                       (0x00000010UL)               /*!<CRC Error flag           */
#define  SPI_SR_MODF                         (0x00000020UL)               /*!<Mode fault               */
#define  SPI_SR_OVR                          (0x00000040UL)               /*!<Overrun flag             */
#define  SPI_SR_BSY                          (0x00000080UL)               /*!<Busy flag                */
#define  SPI_SR_FRE                          (0x00000100UL)               /*!<Frame format error flag  */

/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           (0x0000FFFFUL)            /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   (0x0000FFFFUL)            /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    (0x0000FFFFUL)            /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    (0x0000FFFFUL)            /*!<Tx CRC Register         */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   (0x00000001UL)            /*!<Channel length (number of bits per audio channel) */

#define  SPI_I2SCFGR_DATLEN                  (0x00000006UL)            /*!<DATLEN[1:0] bits (Data length to be transferred)  */
#define  SPI_I2SCFGR_DATLEN_0                (0x00000002UL)            /*!<Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                (0x00000004UL)            /*!<Bit 1 */

#define  SPI_I2SCFGR_CKPOL                   (0x00000008UL)            /*!<steady state clock polarity               */

#define  SPI_I2SCFGR_I2SSTD                  (0x00000030UL)            /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                (0x00000010UL)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                (0x00000020UL)            /*!<Bit 1 */

#define  SPI_I2SCFGR_PCMSYNC                 (0x00000080UL)            /*!<PCM frame synchronization                 */

#define  SPI_I2SCFGR_I2SCFG                  (0x00000300UL)            /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                (0x00000100UL)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                (0x00000200UL)            /*!<Bit 1 */

#define  SPI_I2SCFGR_I2SE                    (0x00000400UL)            /*!<I2S Enable         */
#define  SPI_I2SCFGR_I2SMOD                  (0x00000800UL)            /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    (0x000000FFUL)            /*!<I2S Linear prescaler         */
#define  SPI_I2SPR_ODD                       (0x00000100UL)            /*!<Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     (0x00000200UL)            /*!<Master Clock Output Enable   */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/
#define SYSCFG_MEMRMP_MEM_MODE          (0x00000007UL) /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0        (0x00000001UL)
#define SYSCFG_MEMRMP_MEM_MODE_1        (0x00000002UL)
#define SYSCFG_MEMRMP_MEM_MODE_2        (0x00000004UL)

/******************  Bit definition for SYSCFG_PMC register  ******************/
#define SYSCFG_PMC_ADC1DC2              (0x00010000UL) /*!< Refer to AN4073 on how to use this bit  */

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0            (0x000FUL) /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1            (0x00F0UL) /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2            (0x0F00UL) /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3            (0xF000UL) /*!<EXTI 3 configuration */
/**
  * @brief   EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA         (0x0000UL) /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB         (0x0001UL) /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC         (0x0002UL) /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD         (0x0003UL) /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE         (0x0004UL) /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH         (0x0007UL) /*!<PH[0] pin */

/**
  * @brief   EXTI1 configuration
  */
#define SYSCFG_EXTICR1_EXTI1_PA         (0x0000UL) /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB         (0x0010UL) /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC         (0x0020UL) /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD         (0x0030UL) /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE         (0x0040UL) /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH         (0x0070UL) /*!<PH[1] pin */

/**
  * @brief   EXTI2 configuration
  */
#define SYSCFG_EXTICR1_EXTI2_PA         (0x0000UL) /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB         (0x0100UL) /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC         (0x0200UL) /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD         (0x0300UL) /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE         (0x0400UL) /*!<PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PH         (0x0700UL) /*!<PH[2] pin */

/**
  * @brief   EXTI3 configuration
  */
#define SYSCFG_EXTICR1_EXTI3_PA         (0x0000UL) /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB         (0x1000UL) /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC         (0x2000UL) /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD         (0x3000UL) /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE         (0x4000UL) /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PH         (0x7000UL) /*!<PH[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4            (0x000FUL) /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5            (0x00F0UL) /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6            (0x0F00UL) /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7            (0xF000UL) /*!<EXTI 7 configuration */
/**
  * @brief   EXTI4 configuration
  */
#define SYSCFG_EXTICR2_EXTI4_PA         (0x0000UL) /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB         (0x0001UL) /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC         (0x0002UL) /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD         (0x0003UL) /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE         (0x0004UL) /*!<PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PH         (0x0007UL) /*!<PH[4] pin */

/**
  * @brief   EXTI5 configuration
  */
#define SYSCFG_EXTICR2_EXTI5_PA         (0x0000UL) /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB         (0x0010UL) /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC         (0x0020UL) /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD         (0x0030UL) /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE         (0x0040UL) /*!<PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PH         (0x0070UL) /*!<PH[5] pin */

/**
  * @brief   EXTI6 configuration
  */
#define SYSCFG_EXTICR2_EXTI6_PA         (0x0000UL) /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB         (0x0100UL) /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC         (0x0200UL) /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD         (0x0300UL) /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE         (0x0400UL) /*!<PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PH         (0x0700UL) /*!<PH[6] pin */

/**
  * @brief   EXTI7 configuration
  */
#define SYSCFG_EXTICR2_EXTI7_PA         (0x0000UL) /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB         (0x1000UL) /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC         (0x2000UL) /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD         (0x3000UL) /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE         (0x4000UL) /*!<PE[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PH         (0x7000UL) /*!<PH[7] pin */


/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8            (0x000FUL) /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9            (0x00F0UL) /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10           (0x0F00UL) /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11           (0xF000UL) /*!<EXTI 11 configuration */

/**
  * @brief   EXTI8 configuration
  */
#define SYSCFG_EXTICR3_EXTI8_PA         (0x0000UL) /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB         (0x0001UL) /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC         (0x0002UL) /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD         (0x0003UL) /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE         (0x0004UL) /*!<PE[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PH         (0x0007UL) /*!<PH[8] pin */

/**
  * @brief   EXTI9 configuration
  */
#define SYSCFG_EXTICR3_EXTI9_PA         (0x0000UL) /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB         (0x0010UL) /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC         (0x0020UL) /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD         (0x0030UL) /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE         (0x0040UL) /*!<PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PH         (0x0070UL) /*!<PH[9] pin */

/**
  * @brief   EXTI10 configuration
  */
#define SYSCFG_EXTICR3_EXTI10_PA        (0x0000UL) /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB        (0x0100UL) /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC        (0x0200UL) /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD        (0x0300UL) /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE        (0x0400UL) /*!<PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PH        (0x0700UL) /*!<PH[10] pin */

/**
  * @brief   EXTI11 configuration
  */
#define SYSCFG_EXTICR3_EXTI11_PA        (0x0000UL) /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB        (0x1000UL) /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC        (0x2000UL) /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD        (0x3000UL) /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE        (0x4000UL) /*!<PE[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PH        (0x7000UL) /*!<PH[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12           (0x000FUL) /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13           (0x00F0UL) /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14           (0x0F00UL) /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15           (0xF000UL) /*!<EXTI 15 configuration */
/**
  * @brief   EXTI12 configuration
  */
#define SYSCFG_EXTICR4_EXTI12_PA        (0x0000UL) /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB        (0x0001UL) /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC        (0x0002UL) /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD        (0x0003UL) /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE        (0x0004UL) /*!<PE[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PH        (0x0007UL) /*!<PH[12] pin */

/**
  * @brief   EXTI13 configuration
  */
#define SYSCFG_EXTICR4_EXTI13_PA        (0x0000UL) /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB        (0x0010UL) /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC        (0x0020UL) /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD        (0x0030UL) /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE        (0x0040UL) /*!<PE[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PH        (0x0070UL) /*!<PH[13] pin */

/**
  * @brief   EXTI14 configuration
  */
#define SYSCFG_EXTICR4_EXTI14_PA        (0x0000UL) /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB        (0x0100UL) /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC        (0x0200UL) /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD        (0x0300UL) /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE        (0x0400UL) /*!<PE[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PH        (0x0700UL) /*!<PH[14] pin */

/**
  * @brief   EXTI15 configuration
  */
#define SYSCFG_EXTICR4_EXTI15_PA        (0x0000UL) /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB        (0x1000UL) /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC        (0x2000UL) /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD        (0x3000UL) /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE        (0x4000UL) /*!<PE[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PH        (0x7000UL) /*!<PH[15] pin */

/******************  Bit definition for SYSCFG_CMPCR register  ****************/
#define SYSCFG_CMPCR_CMP_PD             (0x00000001UL) /*!<Compensation cell ready flag */
#define SYSCFG_CMPCR_READY              (0x00000100UL) /*!<Compensation cell power-down */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN                         (0x0001UL)            /*!<Counter enable        */
#define  TIM_CR1_UDIS                        (0x0002UL)            /*!<Update disable        */
#define  TIM_CR1_URS                         (0x0004UL)            /*!<Update request source */
#define  TIM_CR1_OPM                         (0x0008UL)            /*!<One pulse mode        */
#define  TIM_CR1_DIR                         (0x0010UL)            /*!<Direction             */

#define  TIM_CR1_CMS                         (0x0060UL)            /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0                       (0x0020UL)            /*!<Bit 0 */
#define  TIM_CR1_CMS_1                       (0x0040UL)            /*!<Bit 1 */

#define  TIM_CR1_ARPE                        (0x0080UL)            /*!<Auto-reload preload enable     */

#define  TIM_CR1_CKD                         (0x0300UL)            /*!<CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0                       (0x0100UL)            /*!<Bit 0 */
#define  TIM_CR1_CKD_1                       (0x0200UL)            /*!<Bit 1 */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC                        (0x0001UL)            /*!<Capture/Compare Preloaded Control        */
#define  TIM_CR2_CCUS                        (0x0004UL)            /*!<Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS                        (0x0008UL)            /*!<Capture/Compare DMA Selection            */

#define  TIM_CR2_MMS                         (0x0070UL)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0                       (0x0010UL)            /*!<Bit 0 */
#define  TIM_CR2_MMS_1                       (0x0020UL)            /*!<Bit 1 */
#define  TIM_CR2_MMS_2                       (0x0040UL)            /*!<Bit 2 */

#define  TIM_CR2_TI1S                        (0x0080UL)            /*!<TI1 Selection */
#define  TIM_CR2_OIS1                        (0x0100UL)            /*!<Output Idle state 1 (OC1 output)  */
#define  TIM_CR2_OIS1N                       (0x0200UL)            /*!<Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2                        (0x0400UL)            /*!<Output Idle state 2 (OC2 output)  */
#define  TIM_CR2_OIS2N                       (0x0800UL)            /*!<Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3                        (0x1000UL)            /*!<Output Idle state 3 (OC3 output)  */
#define  TIM_CR2_OIS3N                       (0x2000UL)            /*!<Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4                        (0x4000UL)            /*!<Output Idle state 4 (OC4 output)  */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS                        (0x0007UL)            /*!<SMS[2:0] bits (Slave mode selection)    */
#define  TIM_SMCR_SMS_0                      (0x0001UL)            /*!<Bit 0 */
#define  TIM_SMCR_SMS_1                      (0x0002UL)            /*!<Bit 1 */
#define  TIM_SMCR_SMS_2                      (0x0004UL)            /*!<Bit 2 */

#define  TIM_SMCR_TS                         (0x0070UL)            /*!<TS[2:0] bits (Trigger selection)        */
#define  TIM_SMCR_TS_0                       (0x0010UL)            /*!<Bit 0 */
#define  TIM_SMCR_TS_1                       (0x0020UL)            /*!<Bit 1 */
#define  TIM_SMCR_TS_2                       (0x0040UL)            /*!<Bit 2 */

#define  TIM_SMCR_MSM                        (0x0080UL)            /*!<Master/slave mode                       */

#define  TIM_SMCR_ETF                        (0x0F00UL)            /*!<ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0                      (0x0100UL)            /*!<Bit 0 */
#define  TIM_SMCR_ETF_1                      (0x0200UL)            /*!<Bit 1 */
#define  TIM_SMCR_ETF_2                      (0x0400UL)            /*!<Bit 2 */
#define  TIM_SMCR_ETF_3                      (0x0800UL)            /*!<Bit 3 */

#define  TIM_SMCR_ETPS                       (0x3000UL)            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0                     (0x1000UL)            /*!<Bit 0 */
#define  TIM_SMCR_ETPS_1                     (0x2000UL)            /*!<Bit 1 */

#define  TIM_SMCR_ECE                        (0x4000UL)            /*!<External clock enable     */
#define  TIM_SMCR_ETP                        (0x8000UL)            /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE                        (0x0001UL)            /*!<Update interrupt enable */
#define  TIM_DIER_CC1IE                      (0x0002UL)            /*!<Capture/Compare 1 interrupt enable   */
#define  TIM_DIER_CC2IE                      (0x0004UL)            /*!<Capture/Compare 2 interrupt enable   */
#define  TIM_DIER_CC3IE                      (0x0008UL)            /*!<Capture/Compare 3 interrupt enable   */
#define  TIM_DIER_CC4IE                      (0x0010UL)            /*!<Capture/Compare 4 interrupt enable   */
#define  TIM_DIER_COMIE                      (0x0020UL)            /*!<COM interrupt enable                 */
#define  TIM_DIER_TIE                        (0x0040UL)            /*!<Trigger interrupt enable             */
#define  TIM_DIER_BIE                        (0x0080UL)            /*!<Break interrupt enable               */
#define  TIM_DIER_UDE                        (0x0100UL)            /*!<Update DMA request enable            */
#define  TIM_DIER_CC1DE                      (0x0200UL)            /*!<Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE                      (0x0400UL)            /*!<Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE                      (0x0800UL)            /*!<Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE                      (0x1000UL)            /*!<Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE                      (0x2000UL)            /*!<COM DMA request enable               */
#define  TIM_DIER_TDE                        (0x4000UL)            /*!<Trigger DMA request enable           */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                          (0x0001UL)            /*!<Update interrupt Flag              */
#define  TIM_SR_CC1IF                        (0x0002UL)            /*!<Capture/Compare 1 interrupt Flag   */
#define  TIM_SR_CC2IF                        (0x0004UL)            /*!<Capture/Compare 2 interrupt Flag   */
#define  TIM_SR_CC3IF                        (0x0008UL)            /*!<Capture/Compare 3 interrupt Flag   */
#define  TIM_SR_CC4IF                        (0x0010UL)            /*!<Capture/Compare 4 interrupt Flag   */
#define  TIM_SR_COMIF                        (0x0020UL)            /*!<COM interrupt Flag                 */
#define  TIM_SR_TIF                          (0x0040UL)            /*!<Trigger interrupt Flag             */
#define  TIM_SR_BIF                          (0x0080UL)            /*!<Break interrupt Flag               */
#define  TIM_SR_CC1OF                        (0x0200UL)            /*!<Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF                        (0x0400UL)            /*!<Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF                        (0x0800UL)            /*!<Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF                        (0x1000UL)            /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                          (0x01UL)               /*!<Update Generation                         */
#define  TIM_EGR_CC1G                        (0x02UL)               /*!<Capture/Compare 1 Generation              */
#define  TIM_EGR_CC2G                        (0x04UL)               /*!<Capture/Compare 2 Generation              */
#define  TIM_EGR_CC3G                        (0x08UL)               /*!<Capture/Compare 3 Generation              */
#define  TIM_EGR_CC4G                        (0x10UL)               /*!<Capture/Compare 4 Generation              */
#define  TIM_EGR_COMG                        (0x20UL)               /*!<Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                          (0x40UL)               /*!<Trigger Generation                        */
#define  TIM_EGR_BG                          (0x80UL)               /*!<Break Generation                          */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S                      (0x0003UL)            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0                    (0x0001UL)            /*!<Bit 0 */
#define  TIM_CCMR1_CC1S_1                    (0x0002UL)            /*!<Bit 1 */

#define  TIM_CCMR1_OC1FE                     (0x0004UL)            /*!<Output Compare 1 Fast enable                 */
#define  TIM_CCMR1_OC1PE                     (0x0008UL)            /*!<Output Compare 1 Preload enable              */

#define  TIM_CCMR1_OC1M                      (0x0070UL)            /*!<OC1M[2:0] bits (Output Compare 1 Mode)       */
#define  TIM_CCMR1_OC1M_0                    (0x0010UL)            /*!<Bit 0 */
#define  TIM_CCMR1_OC1M_1                    (0x0020UL)            /*!<Bit 1 */
#define  TIM_CCMR1_OC1M_2                    (0x0040UL)            /*!<Bit 2 */

#define  TIM_CCMR1_OC1CE                     (0x0080UL)            /*!<Output Compare 1Clear Enable                 */

#define  TIM_CCMR1_CC2S                      (0x0300UL)            /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0                    (0x0100UL)            /*!<Bit 0 */
#define  TIM_CCMR1_CC2S_1                    (0x0200UL)            /*!<Bit 1 */

#define  TIM_CCMR1_OC2FE                     (0x0400UL)            /*!<Output Compare 2 Fast enable                 */
#define  TIM_CCMR1_OC2PE                     (0x0800UL)            /*!<Output Compare 2 Preload enable              */

#define  TIM_CCMR1_OC2M                      (0x7000UL)            /*!<OC2M[2:0] bits (Output Compare 2 Mode3       */
#define  TIM_CCMR1_OC2M_0                    (0x1000UL)            /*!<Bit 0 */
#define  TIM_CCMR1_OC2M_1                    (0x2000UL)            /*!<Bit 1 */
#define  TIM_CCMR1_OC2M_2                    (0x4000UL)            /*!<Bit 2 */

#define  TIM_CCMR1_OC2CE                     (0x8000UL)            /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR1_IC1PSC                    (0x000CUL)            /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0                  (0x0004UL)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1PSC_1                  (0x0008UL)            /*!<Bit 1 */

#define  TIM_CCMR1_IC1F                      (0x00F0UL)            /*!<IC1F[3:0] bits (Input Capture 1 Filter)      */
#define  TIM_CCMR1_IC1F_0                    (0x0010UL)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1F_1                    (0x0020UL)            /*!<Bit 1 */
#define  TIM_CCMR1_IC1F_2                    (0x0040UL)            /*!<Bit 2 */
#define  TIM_CCMR1_IC1F_3                    (0x0080UL)            /*!<Bit 3 */

#define  TIM_CCMR1_IC2PSC                    (0x0C00UL)            /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler)  */
#define  TIM_CCMR1_IC2PSC_0                  (0x0400UL)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2PSC_1                  (0x0800UL)            /*!<Bit 1 */

#define  TIM_CCMR1_IC2F                      (0xF000UL)            /*!<IC2F[3:0] bits (Input Capture 2 Filter)       */
#define  TIM_CCMR1_IC2F_0                    (0x1000UL)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2F_1                    (0x2000UL)            /*!<Bit 1 */
#define  TIM_CCMR1_IC2F_2                    (0x4000UL)            /*!<Bit 2 */
#define  TIM_CCMR1_IC2F_3                    (0x8000UL)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S                      (0x0003UL)            /*!<CC3S[1:0] bits (Capture/Compare 3 Selection)  */
#define  TIM_CCMR2_CC3S_0                    (0x0001UL)            /*!<Bit 0 */
#define  TIM_CCMR2_CC3S_1                    (0x0002UL)            /*!<Bit 1 */

#define  TIM_CCMR2_OC3FE                     (0x0004UL)            /*!<Output Compare 3 Fast enable           */
#define  TIM_CCMR2_OC3PE                     (0x0008UL)            /*!<Output Compare 3 Preload enable        */

#define  TIM_CCMR2_OC3M                      (0x0070UL)            /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0                    (0x0010UL)            /*!<Bit 0 */
#define  TIM_CCMR2_OC3M_1                    (0x0020UL)            /*!<Bit 1 */
#define  TIM_CCMR2_OC3M_2                    (0x0040UL)            /*!<Bit 2 */

#define  TIM_CCMR2_OC3CE                     (0x0080UL)            /*!<Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S                      (0x0300UL)            /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0                    (0x0100UL)            /*!<Bit 0 */
#define  TIM_CCMR2_CC4S_1                    (0x0200UL)            /*!<Bit 1 */

#define  TIM_CCMR2_OC4FE                     (0x0400UL)            /*!<Output Compare 4 Fast enable    */
#define  TIM_CCMR2_OC4PE                     (0x0800UL)            /*!<Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M                      (0x7000UL)            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0                    (0x1000UL)            /*!<Bit 0 */
#define  TIM_CCMR2_OC4M_1                    (0x2000UL)            /*!<Bit 1 */
#define  TIM_CCMR2_OC4M_2                    (0x4000UL)            /*!<Bit 2 */

#define  TIM_CCMR2_OC4CE                     (0x8000UL)            /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR2_IC3PSC                    (0x000CUL)            /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0                  (0x0004UL)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3PSC_1                  (0x0008UL)            /*!<Bit 1 */

#define  TIM_CCMR2_IC3F                      (0x00F0UL)            /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0                    (0x0010UL)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3F_1                    (0x0020UL)            /*!<Bit 1 */
#define  TIM_CCMR2_IC3F_2                    (0x0040UL)            /*!<Bit 2 */
#define  TIM_CCMR2_IC3F_3                    (0x0080UL)            /*!<Bit 3 */

#define  TIM_CCMR2_IC4PSC                    (0x0C00UL)            /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0                  (0x0400UL)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4PSC_1                  (0x0800UL)            /*!<Bit 1 */

#define  TIM_CCMR2_IC4F                      (0xF000UL)            /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0                    (0x1000UL)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4F_1                    (0x2000UL)            /*!<Bit 1 */
#define  TIM_CCMR2_IC4F_2                    (0x4000UL)            /*!<Bit 2 */
#define  TIM_CCMR2_IC4F_3                    (0x8000UL)            /*!<Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E                       (0x0001UL)            /*!<Capture/Compare 1 output enable                 */
#define  TIM_CCER_CC1P                       (0x0002UL)            /*!<Capture/Compare 1 output Polarity               */
#define  TIM_CCER_CC1NE                      (0x0004UL)            /*!<Capture/Compare 1 Complementary output enable   */
#define  TIM_CCER_CC1NP                      (0x0008UL)            /*!<Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E                       (0x0010UL)            /*!<Capture/Compare 2 output enable                 */
#define  TIM_CCER_CC2P                       (0x0020UL)            /*!<Capture/Compare 2 output Polarity               */
#define  TIM_CCER_CC2NE                      (0x0040UL)            /*!<Capture/Compare 2 Complementary output enable   */
#define  TIM_CCER_CC2NP                      (0x0080UL)            /*!<Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E                       (0x0100UL)            /*!<Capture/Compare 3 output enable                 */
#define  TIM_CCER_CC3P                       (0x0200UL)            /*!<Capture/Compare 3 output Polarity               */
#define  TIM_CCER_CC3NE                      (0x0400UL)            /*!<Capture/Compare 3 Complementary output enable   */
#define  TIM_CCER_CC3NP                      (0x0800UL)            /*!<Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E                       (0x1000UL)            /*!<Capture/Compare 4 output enable                 */
#define  TIM_CCER_CC4P                       (0x2000UL)            /*!<Capture/Compare 4 output Polarity               */
#define  TIM_CCER_CC4NP                      (0x8000UL)            /*!<Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT                         (0xFFFFUL)            /*!<Counter Value            */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC                         (0xFFFFUL)            /*!<Prescaler Value          */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR                         (0xFFFFUL)            /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP                         (0xFFUL)               /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1                       (0xFFFFUL)            /*!<Capture/Compare 1 Value  */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2                       (0xFFFFUL)            /*!<Capture/Compare 2 Value  */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3                       (0xFFFFUL)            /*!<Capture/Compare 3 Value  */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4                       (0xFFFFUL)            /*!<Capture/Compare 4 Value  */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG                        (0x00FFUL)            /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0                      (0x0001UL)            /*!<Bit 0 */
#define  TIM_BDTR_DTG_1                      (0x0002UL)            /*!<Bit 1 */
#define  TIM_BDTR_DTG_2                      (0x0004UL)            /*!<Bit 2 */
#define  TIM_BDTR_DTG_3                      (0x0008UL)            /*!<Bit 3 */
#define  TIM_BDTR_DTG_4                      (0x0010UL)            /*!<Bit 4 */
#define  TIM_BDTR_DTG_5                      (0x0020UL)            /*!<Bit 5 */
#define  TIM_BDTR_DTG_6                      (0x0040UL)            /*!<Bit 6 */
#define  TIM_BDTR_DTG_7                      (0x0080UL)            /*!<Bit 7 */

#define  TIM_BDTR_LOCK                       (0x0300UL)            /*!<LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0                     (0x0100UL)            /*!<Bit 0 */
#define  TIM_BDTR_LOCK_1                     (0x0200UL)            /*!<Bit 1 */

#define  TIM_BDTR_OSSI                       (0x0400UL)            /*!<Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR                       (0x0800UL)            /*!<Off-State Selection for Run mode  */
#define  TIM_BDTR_BKE                        (0x1000UL)            /*!<Break enable                      */
#define  TIM_BDTR_BKP                        (0x2000UL)            /*!<Break Polarity                    */
#define  TIM_BDTR_AOE                        (0x4000UL)            /*!<Automatic Output enable           */
#define  TIM_BDTR_MOE                        (0x8000UL)            /*!<Main Output enable                */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA                         (0x001FUL)            /*!<DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0                       (0x0001UL)            /*!<Bit 0 */
#define  TIM_DCR_DBA_1                       (0x0002UL)            /*!<Bit 1 */
#define  TIM_DCR_DBA_2                       (0x0004UL)            /*!<Bit 2 */
#define  TIM_DCR_DBA_3                       (0x0008UL)            /*!<Bit 3 */
#define  TIM_DCR_DBA_4                       (0x0010UL)            /*!<Bit 4 */

#define  TIM_DCR_DBL                         (0x1F00UL)            /*!<DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0                       (0x0100UL)            /*!<Bit 0 */
#define  TIM_DCR_DBL_1                       (0x0200UL)            /*!<Bit 1 */
#define  TIM_DCR_DBL_2                       (0x0400UL)            /*!<Bit 2 */
#define  TIM_DCR_DBL_3                       (0x0800UL)            /*!<Bit 3 */
#define  TIM_DCR_DBL_4                       (0x1000UL)            /*!<Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB                       (0xFFFFUL)            /*!<DMA register for burst accesses                    */

/*******************  Bit definition for TIM_OR register  *********************/
#define TIM_OR_TI4_RMP                       (0x00C0UL)            /*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)             */
#define TIM_OR_TI4_RMP_0                     (0x0040UL)            /*!<Bit 0 */
#define TIM_OR_TI4_RMP_1                     (0x0080UL)            /*!<Bit 1 */
#define TIM_OR_ITR1_RMP                      (0x0C00UL)            /*!<ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap) */
#define TIM_OR_ITR1_RMP_0                    (0x0400UL)            /*!<Bit 0 */
#define TIM_OR_ITR1_RMP_1                    (0x0800UL)            /*!<Bit 1 */


/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define  USART_SR_PE                         (0x0001UL)            /*!<Parity Error                 */
#define  USART_SR_FE                         (0x0002UL)            /*!<Framing Error                */
#define  USART_SR_NE                         (0x0004UL)            /*!<Noise Error Flag             */
#define  USART_SR_ORE                        (0x0008UL)            /*!<OverRun Error                */
#define  USART_SR_IDLE                       (0x0010UL)            /*!<IDLE line detected           */
#define  USART_SR_RXNE                       (0x0020UL)            /*!<Read Data Register Not Empty */
#define  USART_SR_TC                         (0x0040UL)            /*!<Transmission Complete        */
#define  USART_SR_TXE                        (0x0080UL)            /*!<Transmit Data Register Empty */
#define  USART_SR_LBD                        (0x0100UL)            /*!<LIN Break Detection Flag     */
#define  USART_SR_CTS                        (0x0200UL)            /*!<CTS Flag                     */

/*******************  Bit definition for USART_DR register  *******************/
#define  USART_DR_DR                         (0x01FFUL)            /*!<Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_Fraction              (0x000FUL)            /*!<Fraction of USARTDIV */
#define  USART_BRR_DIV_Mantissa              (0xFFF0UL)            /*!<Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_SBK                       (0x0001UL)            /*!<Send Break                             */
#define  USART_CR1_RWU                       (0x0002UL)            /*!<Receiver wakeup                        */
#define  USART_CR1_RE                        (0x0004UL)            /*!<Receiver Enable                        */
#define  USART_CR1_TE                        (0x0008UL)            /*!<Transmitter Enable                     */
#define  USART_CR1_IDLEIE                    (0x0010UL)            /*!<IDLE Interrupt Enable                  */
#define  USART_CR1_RXNEIE                    (0x0020UL)            /*!<RXNE Interrupt Enable                  */
#define  USART_CR1_TCIE                      (0x0040UL)            /*!<Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     (0x0080UL)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PEIE                      (0x0100UL)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PS                        (0x0200UL)            /*!<Parity Selection                       */
#define  USART_CR1_PCE                       (0x0400UL)            /*!<Parity Control Enable                  */
#define  USART_CR1_WAKE                      (0x0800UL)            /*!<Wakeup method                          */
#define  USART_CR1_M                         (0x1000UL)            /*!<Word length                            */
#define  USART_CR1_UE                        (0x2000UL)            /*!<USART Enable                           */
#define  USART_CR1_OVER8                     (0x8000UL)            /*!<USART Oversampling by 8 enable         */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADD                       (0x000FUL)            /*!<Address of the USART node            */
#define  USART_CR2_LBDL                      (0x0020UL)            /*!<LIN Break Detection Length           */
#define  USART_CR2_LBDIE                     (0x0040UL)            /*!<LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      (0x0100UL)            /*!<Last Bit Clock pulse                 */
#define  USART_CR2_CPHA                      (0x0200UL)            /*!<Clock Phase                          */
#define  USART_CR2_CPOL                      (0x0400UL)            /*!<Clock Polarity                       */
#define  USART_CR2_CLKEN                     (0x0800UL)            /*!<Clock Enable                         */

#define  USART_CR2_STOP                      (0x3000UL)            /*!<STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    (0x1000UL)            /*!<Bit 0 */
#define  USART_CR2_STOP_1                    (0x2000UL)            /*!<Bit 1 */

#define  USART_CR2_LINEN                     (0x4000UL)            /*!<LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       (0x0001UL)            /*!<Error Interrupt Enable      */
#define  USART_CR3_IREN                      (0x0002UL)            /*!<IrDA mode Enable            */
#define  USART_CR3_IRLP                      (0x0004UL)            /*!<IrDA Low-Power              */
#define  USART_CR3_HDSEL                     (0x0008UL)            /*!<Half-Duplex Selection       */
#define  USART_CR3_NACK                      (0x0010UL)            /*!<Smartcard NACK enable       */
#define  USART_CR3_SCEN                      (0x0020UL)            /*!<Smartcard mode enable       */
#define  USART_CR3_DMAR                      (0x0040UL)            /*!<DMA Enable Receiver         */
#define  USART_CR3_DMAT                      (0x0080UL)            /*!<DMA Enable Transmitter      */
#define  USART_CR3_RTSE                      (0x0100UL)            /*!<RTS Enable                  */
#define  USART_CR3_CTSE                      (0x0200UL)            /*!<CTS Enable                  */
#define  USART_CR3_CTSIE                     (0x0400UL)            /*!<CTS Interrupt Enable        */
#define  USART_CR3_ONEBIT                    (0x0800UL)            /*!<USART One bit method enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      (0x00FFUL)            /*!<PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_PSC_0                    (0x0001UL)            /*!<Bit 0 */
#define  USART_GTPR_PSC_1                    (0x0002UL)            /*!<Bit 1 */
#define  USART_GTPR_PSC_2                    (0x0004UL)            /*!<Bit 2 */
#define  USART_GTPR_PSC_3                    (0x0008UL)            /*!<Bit 3 */
#define  USART_GTPR_PSC_4                    (0x0010UL)            /*!<Bit 4 */
#define  USART_GTPR_PSC_5                    (0x0020UL)            /*!<Bit 5 */
#define  USART_GTPR_PSC_6                    (0x0040UL)            /*!<Bit 6 */
#define  USART_GTPR_PSC_7                    (0x0080UL)            /*!<Bit 7 */

#define  USART_GTPR_GT                       (0xFF00UL)            /*!<Guard time value */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           (0x7FUL)               /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T0                          (0x01UL)               /*!<Bit 0 */
#define  WWDG_CR_T1                          (0x02UL)               /*!<Bit 1 */
#define  WWDG_CR_T2                          (0x04UL)               /*!<Bit 2 */
#define  WWDG_CR_T3                          (0x08UL)               /*!<Bit 3 */
#define  WWDG_CR_T4                          (0x10UL)               /*!<Bit 4 */
#define  WWDG_CR_T5                          (0x20UL)               /*!<Bit 5 */
#define  WWDG_CR_T6                          (0x40UL)               /*!<Bit 6 */

#define  WWDG_CR_WDGA                        (0x80UL)               /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          (0x007FUL)            /*!<W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W0                         (0x0001UL)            /*!<Bit 0 */
#define  WWDG_CFR_W1                         (0x0002UL)            /*!<Bit 1 */
#define  WWDG_CFR_W2                         (0x0004UL)            /*!<Bit 2 */
#define  WWDG_CFR_W3                         (0x0008UL)            /*!<Bit 3 */
#define  WWDG_CFR_W4                         (0x0010UL)            /*!<Bit 4 */
#define  WWDG_CFR_W5                         (0x0020UL)            /*!<Bit 5 */
#define  WWDG_CFR_W6                         (0x0040UL)            /*!<Bit 6 */

#define  WWDG_CFR_WDGTB                      (0x0180UL)            /*!<WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB0                     (0x0080UL)            /*!<Bit 0 */
#define  WWDG_CFR_WDGTB1                     (0x0100UL)            /*!<Bit 1 */

#define  WWDG_CFR_EWI                        (0x0200UL)            /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        (0x01UL)               /*!<Early Wakeup Interrupt Flag */


/******************************************************************************/
/*                                                                            */
/*                                DBG                                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBGMCU_IDCODE register  *************/
#define  DBGMCU_IDCODE_DEV_ID                (0x00000FFFUL)
#define  DBGMCU_IDCODE_REV_ID                (0xFFFF0000UL)

/********************  Bit definition for DBGMCU_CR register  *****************/
#define  DBGMCU_CR_DBG_SLEEP                 (0x00000001UL)
#define  DBGMCU_CR_DBG_STOP                  (0x00000002UL)
#define  DBGMCU_CR_DBG_STANDBY               (0x00000004UL)
#define  DBGMCU_CR_TRACE_IOEN                (0x00000020UL)

#define  DBGMCU_CR_TRACE_MODE                (0x000000C0UL)
#define  DBGMCU_CR_TRACE_MODE_0              (0x00000040UL)/*!<Bit 0 */
#define  DBGMCU_CR_TRACE_MODE_1              (0x00000080UL)/*!<Bit 1 */

/********************  Bit definition for DBGMCU_APB1_FZ register  ************/
#define  DBGMCU_APB1_FZ_DBG_TIM2_STOP            (0x00000001UL)
#define  DBGMCU_APB1_FZ_DBG_TIM3_STOP            (0x00000002UL)
#define  DBGMCU_APB1_FZ_DBG_TIM4_STOP            (0x00000004UL)
#define  DBGMCU_APB1_FZ_DBG_TIM5_STOP            (0x00000008UL)
#define  DBGMCU_APB1_FZ_DBG_TIM6_STOP            (0x00000010UL)
#define  DBGMCU_APB1_FZ_DBG_TIM7_STOP            (0x00000020UL)
#define  DBGMCU_APB1_FZ_DBG_TIM12_STOP           (0x00000040UL)
#define  DBGMCU_APB1_FZ_DBG_TIM13_STOP           (0x00000080UL)
#define  DBGMCU_APB1_FZ_DBG_TIM14_STOP           (0x00000100UL)
#define  DBGMCU_APB1_FZ_DBG_RTC_STOP             (0x00000400UL)
#define  DBGMCU_APB1_FZ_DBG_WWDG_STOP            (0x00000800UL)
#define  DBGMCU_APB1_FZ_DBG_IWDG_STOP            (0x00001000UL)
#define  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT   (0x00200000UL)
#define  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT   (0x00400000UL)
#define  DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT   (0x00800000UL)
#define  DBGMCU_APB1_FZ_DBG_CAN1_STOP            (0x02000000UL)
#define  DBGMCU_APB1_FZ_DBG_CAN2_STOP            (0x04000000UL)
/* Old IWDGSTOP bit definition, maintained for legacy purpose */
#define  DBGMCU_APB1_FZ_DBG_IWDEG_STOP           DBGMCU_APB1_FZ_DBG_IWDG_STOP

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
#define  DBGMCU_APB2_FZ_DBG_TIM1_STOP        (0x00000001UL)
#define  DBGMCU_APB2_FZ_DBG_TIM8_STOP        (0x00000002UL)
#define  DBGMCU_APB2_FZ_DBG_TIM9_STOP        (0x00010000UL)
#define  DBGMCU_APB2_FZ_DBG_TIM10_STOP       (0x00020000UL)
#define  DBGMCU_APB2_FZ_DBG_TIM11_STOP       (0x00040000UL)

/******************************************************************************/
/*                                                                            */
/*                                       USB_OTG                              */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition forUSB_OTG_GOTGCTL register  ********************/
#define USB_OTG_GOTGCTL_SRQSCS                  (0x00000001UL)            /*!< Session request success */
#define USB_OTG_GOTGCTL_SRQ                     (0x00000002UL)            /*!< Session request */
#define USB_OTG_GOTGCTL_HNGSCS                  (0x00000100UL)            /*!< Host negotiation success */
#define USB_OTG_GOTGCTL_HNPRQ                   (0x00000200UL)            /*!< HNP request */
#define USB_OTG_GOTGCTL_HSHNPEN                 (0x00000400UL)            /*!< Host set HNP enable */
#define USB_OTG_GOTGCTL_DHNPEN                  (0x00000800UL)            /*!< Device HNP enabled */
#define USB_OTG_GOTGCTL_CIDSTS                  (0x00010000UL)            /*!< Connector ID status */
#define USB_OTG_GOTGCTL_DBCT                    (0x00020000UL)            /*!< Long/short debounce time */
#define USB_OTG_GOTGCTL_ASVLD                   (0x00040000UL)            /*!< A-session valid */
#define USB_OTG_GOTGCTL_BSVLD                   (0x00080000UL)            /*!< B-session valid */

/********************  Bit definition forUSB_OTG_HCFG register  ********************/

#define USB_OTG_HCFG_FSLSPCS                 (0x00000003UL)            /*!< FS/LS PHY clock select */
#define USB_OTG_HCFG_FSLSPCS_0               (0x00000001UL)            /*!<Bit 0 */
#define USB_OTG_HCFG_FSLSPCS_1               (0x00000002UL)            /*!<Bit 1 */
#define USB_OTG_HCFG_FSLSS                   (0x00000004UL)            /*!< FS- and LS-only support */

/********************  Bit definition forUSB_OTG_DCFG register  ********************/

#define USB_OTG_DCFG_DSPD                    (0x00000003UL)            /*!< Device speed */
#define USB_OTG_DCFG_DSPD_0                  (0x00000001UL)            /*!<Bit 0 */
#define USB_OTG_DCFG_DSPD_1                  (0x00000002UL)            /*!<Bit 1 */
#define USB_OTG_DCFG_NZLSOHSK                (0x00000004UL)            /*!< Nonzero-length status OUT handshake */

#define USB_OTG_DCFG_DAD                     (0x000007F0UL)            /*!< Device address */
#define USB_OTG_DCFG_DAD_0                   (0x00000010UL)            /*!<Bit 0 */
#define USB_OTG_DCFG_DAD_1                   (0x00000020UL)            /*!<Bit 1 */
#define USB_OTG_DCFG_DAD_2                   (0x00000040UL)            /*!<Bit 2 */
#define USB_OTG_DCFG_DAD_3                   (0x00000080UL)            /*!<Bit 3 */
#define USB_OTG_DCFG_DAD_4                   (0x00000100UL)            /*!<Bit 4 */
#define USB_OTG_DCFG_DAD_5                   (0x00000200UL)            /*!<Bit 5 */
#define USB_OTG_DCFG_DAD_6                   (0x00000400UL)            /*!<Bit 6 */

#define USB_OTG_DCFG_PFIVL                   (0x00001800UL)            /*!< Periodic (micro)frame interval */
#define USB_OTG_DCFG_PFIVL_0                 (0x00000800UL)            /*!<Bit 0 */
#define USB_OTG_DCFG_PFIVL_1                 (0x00001000UL)            /*!<Bit 1 */

#define USB_OTG_DCFG_PERSCHIVL               (0x03000000UL)            /*!< Periodic scheduling interval */
#define USB_OTG_DCFG_PERSCHIVL_0             (0x01000000UL)            /*!<Bit 0 */
#define USB_OTG_DCFG_PERSCHIVL_1             (0x02000000UL)            /*!<Bit 1 */

/********************  Bit definition forUSB_OTG_PCGCR register  ********************/
#define USB_OTG_PCGCR_STPPCLK                 (0x00000001UL)            /*!< Stop PHY clock */
#define USB_OTG_PCGCR_GATEHCLK                (0x00000002UL)            /*!< Gate HCLK */
#define USB_OTG_PCGCR_PHYSUSP                 (0x00000010UL)            /*!< PHY suspended */

/********************  Bit definition forUSB_OTG_GOTGINT register  ********************/
#define USB_OTG_GOTGINT_SEDET                   (0x00000004UL)            /*!< Session end detected */
#define USB_OTG_GOTGINT_SRSSCHG                 (0x00000100UL)            /*!< Session request success status change */
#define USB_OTG_GOTGINT_HNSSCHG                 (0x00000200UL)            /*!< Host negotiation success status change */
#define USB_OTG_GOTGINT_HNGDET                  (0x00020000UL)            /*!< Host negotiation detected */
#define USB_OTG_GOTGINT_ADTOCHG                 (0x00040000UL)            /*!< A-device timeout change */
#define USB_OTG_GOTGINT_DBCDNE                  (0x00080000UL)            /*!< Debounce done */

/********************  Bit definition forUSB_OTG_DCTL register  ********************/
#define USB_OTG_DCTL_RWUSIG                  (0x00000001UL)            /*!< Remote wakeup signaling */
#define USB_OTG_DCTL_SDIS                    (0x00000002UL)            /*!< Soft disconnect */
#define USB_OTG_DCTL_GINSTS                  (0x00000004UL)            /*!< Global IN NAK status */
#define USB_OTG_DCTL_GONSTS                  (0x00000008UL)            /*!< Global OUT NAK status */

#define USB_OTG_DCTL_TCTL                    (0x00000070UL)            /*!< Test control */
#define USB_OTG_DCTL_TCTL_0                  (0x00000010UL)            /*!<Bit 0 */
#define USB_OTG_DCTL_TCTL_1                  (0x00000020UL)            /*!<Bit 1 */
#define USB_OTG_DCTL_TCTL_2                  (0x00000040UL)            /*!<Bit 2 */
#define USB_OTG_DCTL_SGINAK                  (0x00000080UL)            /*!< Set global IN NAK */
#define USB_OTG_DCTL_CGINAK                  (0x00000100UL)            /*!< Clear global IN NAK */
#define USB_OTG_DCTL_SGONAK                  (0x00000200UL)            /*!< Set global OUT NAK */
#define USB_OTG_DCTL_CGONAK                  (0x00000400UL)            /*!< Clear global OUT NAK */
#define USB_OTG_DCTL_POPRGDNE                (0x00000800UL)            /*!< Power-on programming done */

/********************  Bit definition forUSB_OTG_HFIR register  ********************/
#define USB_OTG_HFIR_FRIVL                   (0x0000FFFFUL)            /*!< Frame interval */

/********************  Bit definition forUSB_OTG_HFNUM register  ********************/
#define USB_OTG_HFNUM_FRNUM                   (0x0000FFFFUL)            /*!< Frame number */
#define USB_OTG_HFNUM_FTREM                   (0xFFFF0000UL)            /*!< Frame time remaining */

/********************  Bit definition forUSB_OTG_DSTS register  ********************/
#define USB_OTG_DSTS_SUSPSTS                 (0x00000001UL)            /*!< Suspend status */

#define USB_OTG_DSTS_ENUMSPD                 (0x00000006UL)            /*!< Enumerated speed */
#define USB_OTG_DSTS_ENUMSPD_0               (0x00000002UL)            /*!<Bit 0 */
#define USB_OTG_DSTS_ENUMSPD_1               (0x00000004UL)            /*!<Bit 1 */
#define USB_OTG_DSTS_EERR                    (0x00000008UL)            /*!< Erratic error */
#define USB_OTG_DSTS_FNSOF                   (0x003FFF00UL)            /*!< Frame number of the received SOF */

/********************  Bit definition forUSB_OTG_GAHBCFG register  ********************/
#define USB_OTG_GAHBCFG_GINT                    (0x00000001UL)            /*!< Global interrupt mask */

#define USB_OTG_GAHBCFG_HBSTLEN                 (0x0000001EUL)            /*!< Burst length/type */
#define USB_OTG_GAHBCFG_HBSTLEN_0               (0x00000002UL)            /*!<Bit 0 */
#define USB_OTG_GAHBCFG_HBSTLEN_1               (0x00000004UL)            /*!<Bit 1 */
#define USB_OTG_GAHBCFG_HBSTLEN_2               (0x00000008UL)            /*!<Bit 2 */
#define USB_OTG_GAHBCFG_HBSTLEN_3               (0x00000010UL)            /*!<Bit 3 */
#define USB_OTG_GAHBCFG_DMAEN                   (0x00000020UL)            /*!< DMA enable */
#define USB_OTG_GAHBCFG_TXFELVL                 (0x00000080UL)            /*!< TxFIFO empty level */
#define USB_OTG_GAHBCFG_PTXFELVL                (0x00000100UL)            /*!< Periodic TxFIFO empty level */

/********************  Bit definition forUSB_OTG_GUSBCFG register  ********************/

#define USB_OTG_GUSBCFG_TOCAL                   (0x00000007UL)            /*!< FS timeout calibration */
#define USB_OTG_GUSBCFG_TOCAL_0                 (0x00000001UL)            /*!<Bit 0 */
#define USB_OTG_GUSBCFG_TOCAL_1                 (0x00000002UL)            /*!<Bit 1 */
#define USB_OTG_GUSBCFG_TOCAL_2                 (0x00000004UL)            /*!<Bit 2 */
#define USB_OTG_GUSBCFG_PHYSEL                  (0x00000040UL)            /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
#define USB_OTG_GUSBCFG_SRPCAP                  (0x00000100UL)            /*!< SRP-capable */
#define USB_OTG_GUSBCFG_HNPCAP                  (0x00000200UL)            /*!< HNP-capable */

#define USB_OTG_GUSBCFG_TRDT                    (0x00003C00UL)            /*!< USB turnaround time */
#define USB_OTG_GUSBCFG_TRDT_0                  (0x00000400UL)            /*!<Bit 0 */
#define USB_OTG_GUSBCFG_TRDT_1                  (0x00000800UL)            /*!<Bit 1 */
#define USB_OTG_GUSBCFG_TRDT_2                  (0x00001000UL)            /*!<Bit 2 */
#define USB_OTG_GUSBCFG_TRDT_3                  (0x00002000UL)            /*!<Bit 3 */
#define USB_OTG_GUSBCFG_PHYLPCS                 (0x00008000UL)            /*!< PHY Low-power clock select */
#define USB_OTG_GUSBCFG_ULPIFSLS                (0x00020000UL)            /*!< ULPI FS/LS select */
#define USB_OTG_GUSBCFG_ULPIAR                  (0x00040000UL)            /*!< ULPI Auto-resume */
#define USB_OTG_GUSBCFG_ULPICSM                 (0x00080000UL)            /*!< ULPI Clock SuspendM */
#define USB_OTG_GUSBCFG_ULPIEVBUSD              (0x00100000UL)            /*!< ULPI External VBUS Drive */
#define USB_OTG_GUSBCFG_ULPIEVBUSI              (0x00200000UL)            /*!< ULPI external VBUS indicator */
#define USB_OTG_GUSBCFG_TSDPS                   (0x00400000UL)            /*!< TermSel DLine pulsing selection */
#define USB_OTG_GUSBCFG_PCCI                    (0x00800000UL)            /*!< Indicator complement */
#define USB_OTG_GUSBCFG_PTCI                    (0x01000000UL)            /*!< Indicator pass through */
#define USB_OTG_GUSBCFG_ULPIIPD                 (0x02000000UL)            /*!< ULPI interface protect disable */
#define USB_OTG_GUSBCFG_FHMOD                   (0x20000000UL)            /*!< Forced host mode */
#define USB_OTG_GUSBCFG_FDMOD                   (0x40000000UL)            /*!< Forced peripheral mode */
#define USB_OTG_GUSBCFG_CTXPKT                  (0x80000000UL)            /*!< Corrupt Tx packet */

/********************  Bit definition forUSB_OTG_GRSTCTL register  ********************/
#define USB_OTG_GRSTCTL_CSRST                   (0x00000001UL)            /*!< Core soft reset */
#define USB_OTG_GRSTCTL_HSRST                   (0x00000002UL)            /*!< HCLK soft reset */
#define USB_OTG_GRSTCTL_FCRST                   (0x00000004UL)            /*!< Host frame counter reset */
#define USB_OTG_GRSTCTL_RXFFLSH                 (0x00000010UL)            /*!< RxFIFO flush */
#define USB_OTG_GRSTCTL_TXFFLSH                 (0x00000020UL)            /*!< TxFIFO flush */

#define USB_OTG_GRSTCTL_TXFNUM                  (0x000007C0UL)            /*!< TxFIFO number */
#define USB_OTG_GRSTCTL_TXFNUM_0                (0x00000040UL)            /*!<Bit 0 */
#define USB_OTG_GRSTCTL_TXFNUM_1                (0x00000080UL)            /*!<Bit 1 */
#define USB_OTG_GRSTCTL_TXFNUM_2                (0x00000100UL)            /*!<Bit 2 */
#define USB_OTG_GRSTCTL_TXFNUM_3                (0x00000200UL)            /*!<Bit 3 */
#define USB_OTG_GRSTCTL_TXFNUM_4                (0x00000400UL)            /*!<Bit 4 */
#define USB_OTG_GRSTCTL_DMAREQ                  (0x40000000UL)            /*!< DMA request signal */
#define USB_OTG_GRSTCTL_AHBIDL                  (0x80000000UL)            /*!< AHB master idle */

/********************  Bit definition forUSB_OTG_DIEPMSK register  ********************/
#define USB_OTG_DIEPMSK_XFRCM                   (0x00000001UL)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DIEPMSK_EPDM                    (0x00000002UL)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DIEPMSK_TOM                     (0x00000008UL)            /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPMSK_ITTXFEMSK               (0x00000010UL)            /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DIEPMSK_INEPNMM                 (0x00000020UL)            /*!< IN token received with EP mismatch mask */
#define USB_OTG_DIEPMSK_INEPNEM                 (0x00000040UL)            /*!< IN endpoint NAK effective mask */
#define USB_OTG_DIEPMSK_TXFURM                  (0x00000100UL)            /*!< FIFO underrun mask */
#define USB_OTG_DIEPMSK_BIM                     (0x00000200UL)            /*!< BNA interrupt mask */

/********************  Bit definition forUSB_OTG_HPTXSTS register  ********************/
#define USB_OTG_HPTXSTS_PTXFSAVL                (0x0000FFFFUL)            /*!< Periodic transmit data FIFO space available */

#define USB_OTG_HPTXSTS_PTXQSAV                 (0x00FF0000UL)            /*!< Periodic transmit request queue space available */
#define USB_OTG_HPTXSTS_PTXQSAV_0               (0x00010000UL)            /*!<Bit 0 */
#define USB_OTG_HPTXSTS_PTXQSAV_1               (0x00020000UL)            /*!<Bit 1 */
#define USB_OTG_HPTXSTS_PTXQSAV_2               (0x00040000UL)            /*!<Bit 2 */
#define USB_OTG_HPTXSTS_PTXQSAV_3               (0x00080000UL)            /*!<Bit 3 */
#define USB_OTG_HPTXSTS_PTXQSAV_4               (0x00100000UL)            /*!<Bit 4 */
#define USB_OTG_HPTXSTS_PTXQSAV_5               (0x00200000UL)            /*!<Bit 5 */
#define USB_OTG_HPTXSTS_PTXQSAV_6               (0x00400000UL)            /*!<Bit 6 */
#define USB_OTG_HPTXSTS_PTXQSAV_7               (0x00800000UL)            /*!<Bit 7 */

#define USB_OTG_HPTXSTS_PTXQTOP                 (0xFF000000UL)            /*!< Top of the periodic transmit request queue */
#define USB_OTG_HPTXSTS_PTXQTOP_0               (0x01000000UL)            /*!<Bit 0 */
#define USB_OTG_HPTXSTS_PTXQTOP_1               (0x02000000UL)            /*!<Bit 1 */
#define USB_OTG_HPTXSTS_PTXQTOP_2               (0x04000000UL)            /*!<Bit 2 */
#define USB_OTG_HPTXSTS_PTXQTOP_3               (0x08000000UL)            /*!<Bit 3 */
#define USB_OTG_HPTXSTS_PTXQTOP_4               (0x10000000UL)            /*!<Bit 4 */
#define USB_OTG_HPTXSTS_PTXQTOP_5               (0x20000000UL)            /*!<Bit 5 */
#define USB_OTG_HPTXSTS_PTXQTOP_6               (0x40000000UL)            /*!<Bit 6 */
#define USB_OTG_HPTXSTS_PTXQTOP_7               (0x80000000UL)            /*!<Bit 7 */

/********************  Bit definition forUSB_OTG_HAINT register  ********************/
#define USB_OTG_HAINT_HAINT                   (0x0000FFFFUL)            /*!< Channel interrupts */

/********************  Bit definition forUSB_OTG_DOEPMSK register  ********************/
#define USB_OTG_DOEPMSK_XFRCM                   (0x00000001UL)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DOEPMSK_EPDM                    (0x00000002UL)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DOEPMSK_STUPM                   (0x00000008UL)            /*!< SETUP phase done mask */
#define USB_OTG_DOEPMSK_OTEPDM                  (0x00000010UL)            /*!< OUT token received when endpoint disabled mask */
#define USB_OTG_DOEPMSK_B2BSTUP                 (0x00000040UL)            /*!< Back-to-back SETUP packets received mask */
#define USB_OTG_DOEPMSK_OPEM                    (0x00000100UL)            /*!< OUT packet error mask */
#define USB_OTG_DOEPMSK_BOIM                    (0x00000200UL)            /*!< BNA interrupt mask */

/********************  Bit definition forUSB_OTG_GINTSTS register  ********************/
#define USB_OTG_GINTSTS_CMOD                    (0x00000001UL)            /*!< Current mode of operation */
#define USB_OTG_GINTSTS_MMIS                    (0x00000002UL)            /*!< Mode mismatch interrupt */
#define USB_OTG_GINTSTS_OTGINT                  (0x00000004UL)            /*!< OTG interrupt */
#define USB_OTG_GINTSTS_SOF                     (0x00000008UL)            /*!< Start of frame */
#define USB_OTG_GINTSTS_RXFLVL                  (0x00000010UL)            /*!< RxFIFO nonempty */
#define USB_OTG_GINTSTS_NPTXFE                  (0x00000020UL)            /*!< Nonperiodic TxFIFO empty */
#define USB_OTG_GINTSTS_GINAKEFF                (0x00000040UL)            /*!< Global IN nonperiodic NAK effective */
#define USB_OTG_GINTSTS_BOUTNAKEFF              (0x00000080UL)            /*!< Global OUT NAK effective */
#define USB_OTG_GINTSTS_ESUSP                   (0x00000400UL)            /*!< Early suspend */
#define USB_OTG_GINTSTS_USBSUSP                 (0x00000800UL)            /*!< USB suspend */
#define USB_OTG_GINTSTS_USBRST                  (0x00001000UL)            /*!< USB reset */
#define USB_OTG_GINTSTS_ENUMDNE                 (0x00002000UL)            /*!< Enumeration done */
#define USB_OTG_GINTSTS_ISOODRP                 (0x00004000UL)            /*!< Isochronous OUT packet dropped interrupt */
#define USB_OTG_GINTSTS_EOPF                    (0x00008000UL)            /*!< End of periodic frame interrupt */
#define USB_OTG_GINTSTS_IEPINT                  (0x00040000UL)            /*!< IN endpoint interrupt */
#define USB_OTG_GINTSTS_OEPINT                  (0x00080000UL)            /*!< OUT endpoint interrupt */
#define USB_OTG_GINTSTS_IISOIXFR                (0x00100000UL)            /*!< Incomplete isochronous IN transfer */
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT       (0x00200000UL)            /*!< Incomplete periodic transfer */
#define USB_OTG_GINTSTS_DATAFSUSP               (0x00400000UL)            /*!< Data fetch suspended */
#define USB_OTG_GINTSTS_HPRTINT                 (0x01000000UL)            /*!< Host port interrupt */
#define USB_OTG_GINTSTS_HCINT                   (0x02000000UL)            /*!< Host channels interrupt */
#define USB_OTG_GINTSTS_PTXFE                   (0x04000000UL)            /*!< Periodic TxFIFO empty */
#define USB_OTG_GINTSTS_CIDSCHG                 (0x10000000UL)            /*!< Connector ID status change */
#define USB_OTG_GINTSTS_DISCINT                 (0x20000000UL)            /*!< Disconnect detected interrupt */
#define USB_OTG_GINTSTS_SRQINT                  (0x40000000UL)            /*!< Session request/new session detected interrupt */
#define USB_OTG_GINTSTS_WKUINT                  (0x80000000UL)            /*!< Resume/remote wakeup detected interrupt */

/********************  Bit definition forUSB_OTG_GINTMSK register  ********************/
#define USB_OTG_GINTMSK_MMISM                   (0x00000002UL)            /*!< Mode mismatch interrupt mask */
#define USB_OTG_GINTMSK_OTGINT                  (0x00000004UL)            /*!< OTG interrupt mask */
#define USB_OTG_GINTMSK_SOFM                    (0x00000008UL)            /*!< Start of frame mask */
#define USB_OTG_GINTMSK_RXFLVLM                 (0x00000010UL)            /*!< Receive FIFO nonempty mask */
#define USB_OTG_GINTMSK_NPTXFEM                 (0x00000020UL)            /*!< Nonperiodic TxFIFO empty mask */
#define USB_OTG_GINTMSK_GINAKEFFM               (0x00000040UL)            /*!< Global nonperiodic IN NAK effective mask */
#define USB_OTG_GINTMSK_GONAKEFFM               (0x00000080UL)            /*!< Global OUT NAK effective mask */
#define USB_OTG_GINTMSK_ESUSPM                  (0x00000400UL)            /*!< Early suspend mask */
#define USB_OTG_GINTMSK_USBSUSPM                (0x00000800UL)            /*!< USB suspend mask */
#define USB_OTG_GINTMSK_USBRST                  (0x00001000UL)            /*!< USB reset mask */
#define USB_OTG_GINTMSK_ENUMDNEM                (0x00002000UL)            /*!< Enumeration done mask */
#define USB_OTG_GINTMSK_ISOODRPM                (0x00004000UL)            /*!< Isochronous OUT packet dropped interrupt mask */
#define USB_OTG_GINTMSK_EOPFM                   (0x00008000UL)            /*!< End of periodic frame interrupt mask */
#define USB_OTG_GINTMSK_EPMISM                  (0x00020000UL)            /*!< Endpoint mismatch interrupt mask */
#define USB_OTG_GINTMSK_IEPINT                  (0x00040000UL)            /*!< IN endpoints interrupt mask */
#define USB_OTG_GINTMSK_OEPINT                  (0x00080000UL)            /*!< OUT endpoints interrupt mask */
#define USB_OTG_GINTMSK_IISOIXFRM               (0x00100000UL)            /*!< Incomplete isochronous IN transfer mask */
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM         (0x00200000UL)            /*!< Incomplete periodic transfer mask */
#define USB_OTG_GINTMSK_FSUSPM                  (0x00400000UL)            /*!< Data fetch suspended mask */
#define USB_OTG_GINTMSK_PRTIM                   (0x01000000UL)            /*!< Host port interrupt mask */
#define USB_OTG_GINTMSK_HCIM                    (0x02000000UL)            /*!< Host channels interrupt mask */
#define USB_OTG_GINTMSK_PTXFEM                  (0x04000000UL)            /*!< Periodic TxFIFO empty mask */
#define USB_OTG_GINTMSK_CIDSCHGM                (0x10000000UL)            /*!< Connector ID status change mask */
#define USB_OTG_GINTMSK_DISCINT                 (0x20000000UL)            /*!< Disconnect detected interrupt mask */
#define USB_OTG_GINTMSK_SRQIM                   (0x40000000UL)            /*!< Session request/new session detected interrupt mask */
#define USB_OTG_GINTMSK_WUIM                    (0x80000000UL)            /*!< Resume/remote wakeup detected interrupt mask */

/********************  Bit definition forUSB_OTG_DAINT register  ********************/
#define USB_OTG_DAINT_IEPINT                  (0x0000FFFFUL)            /*!< IN endpoint interrupt bits */
#define USB_OTG_DAINT_OEPINT                  (0xFFFF0000UL)            /*!< OUT endpoint interrupt bits */

/********************  Bit definition forUSB_OTG_HAINTMSK register  ********************/
#define USB_OTG_HAINTMSK_HAINTM                  (0x0000FFFFUL)            /*!< Channel interrupt mask */

/********************  Bit definition for USB_OTG_GRXSTSP register  ********************/
#define USB_OTG_GRXSTSP_EPNUM                    (0x0000000FUL)            /*!< IN EP interrupt mask bits */
#define USB_OTG_GRXSTSP_BCNT                     (0x00007FF0UL)            /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_DPID                     (0x00018000UL)            /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_PKTSTS                   (0x001E0000UL)            /*!< OUT EP interrupt mask bits */

/********************  Bit definition forUSB_OTG_DAINTMSK register  ********************/
#define USB_OTG_DAINTMSK_IEPM                    (0x0000FFFFUL)            /*!< IN EP interrupt mask bits */
#define USB_OTG_DAINTMSK_OEPM                    (0xFFFF0000UL)            /*!< OUT EP interrupt mask bits */

/********************  Bit definition for OTG register  ********************/

#define USB_OTG_CHNUM                   (0x0000000FUL)            /*!< Channel number */
#define USB_OTG_CHNUM_0                 (0x00000001UL)            /*!<Bit 0 */
#define USB_OTG_CHNUM_1                 (0x00000002UL)            /*!<Bit 1 */
#define USB_OTG_CHNUM_2                 (0x00000004UL)            /*!<Bit 2 */
#define USB_OTG_CHNUM_3                 (0x00000008UL)            /*!<Bit 3 */
#define USB_OTG_BCNT                    (0x00007FF0UL)            /*!< Byte count */

#define USB_OTG_DPID                    (0x00018000UL)            /*!< Data PID */
#define USB_OTG_DPID_0                  (0x00008000UL)            /*!<Bit 0 */
#define USB_OTG_DPID_1                  (0x00010000UL)            /*!<Bit 1 */

#define USB_OTG_PKTSTS                  (0x001E0000UL)            /*!< Packet status */
#define USB_OTG_PKTSTS_0                (0x00020000UL)            /*!<Bit 0 */
#define USB_OTG_PKTSTS_1                (0x00040000UL)            /*!<Bit 1 */
#define USB_OTG_PKTSTS_2                (0x00080000UL)            /*!<Bit 2 */
#define USB_OTG_PKTSTS_3                (0x00100000UL)            /*!<Bit 3 */

#define USB_OTG_EPNUM                   (0x0000000FUL)            /*!< Endpoint number */
#define USB_OTG_EPNUM_0                 (0x00000001UL)            /*!<Bit 0 */
#define USB_OTG_EPNUM_1                 (0x00000002UL)            /*!<Bit 1 */
#define USB_OTG_EPNUM_2                 (0x00000004UL)            /*!<Bit 2 */
#define USB_OTG_EPNUM_3                 (0x00000008UL)            /*!<Bit 3 */

#define USB_OTG_FRMNUM                  (0x01E00000UL)            /*!< Frame number */
#define USB_OTG_FRMNUM_0                (0x00200000UL)            /*!<Bit 0 */
#define USB_OTG_FRMNUM_1                (0x00400000UL)            /*!<Bit 1 */
#define USB_OTG_FRMNUM_2                (0x00800000UL)            /*!<Bit 2 */
#define USB_OTG_FRMNUM_3                (0x01000000UL)            /*!<Bit 3 */

/********************  Bit definition for OTG register  ********************/

#define USB_OTG_CHNUM                   (0x0000000FUL)            /*!< Channel number */
#define USB_OTG_CHNUM_0                 (0x00000001UL)            /*!<Bit 0 */
#define USB_OTG_CHNUM_1                 (0x00000002UL)            /*!<Bit 1 */
#define USB_OTG_CHNUM_2                 (0x00000004UL)            /*!<Bit 2 */
#define USB_OTG_CHNUM_3                 (0x00000008UL)            /*!<Bit 3 */
#define USB_OTG_BCNT                    (0x00007FF0UL)            /*!< Byte count */

#define USB_OTG_DPID                    (0x00018000UL)            /*!< Data PID */
#define USB_OTG_DPID_0                  (0x00008000UL)            /*!<Bit 0 */
#define USB_OTG_DPID_1                  (0x00010000UL)            /*!<Bit 1 */

#define USB_OTG_PKTSTS                  (0x001E0000UL)            /*!< Packet status */
#define USB_OTG_PKTSTS_0                (0x00020000UL)            /*!<Bit 0 */
#define USB_OTG_PKTSTS_1                (0x00040000UL)            /*!<Bit 1 */
#define USB_OTG_PKTSTS_2                (0x00080000UL)            /*!<Bit 2 */
#define USB_OTG_PKTSTS_3                (0x00100000UL)            /*!<Bit 3 */

#define USB_OTG_EPNUM                   (0x0000000FUL)            /*!< Endpoint number */
#define USB_OTG_EPNUM_0                 (0x00000001UL)            /*!<Bit 0 */
#define USB_OTG_EPNUM_1                 (0x00000002UL)            /*!<Bit 1 */
#define USB_OTG_EPNUM_2                 (0x00000004UL)            /*!<Bit 2 */
#define USB_OTG_EPNUM_3                 (0x00000008UL)            /*!<Bit 3 */

#define USB_OTG_FRMNUM                  (0x01E00000UL)            /*!< Frame number */
#define USB_OTG_FRMNUM_0                (0x00200000UL)            /*!<Bit 0 */
#define USB_OTG_FRMNUM_1                (0x00400000UL)            /*!<Bit 1 */
#define USB_OTG_FRMNUM_2                (0x00800000UL)            /*!<Bit 2 */
#define USB_OTG_FRMNUM_3                (0x01000000UL)            /*!<Bit 3 */

/********************  Bit definition forUSB_OTG_GRXFSIZ register  ********************/
#define USB_OTG_GRXFSIZ_RXFD                    (0x0000FFFFUL)            /*!< RxFIFO depth */

/********************  Bit definition forUSB_OTG_DVBUSDIS register  ********************/
#define USB_OTG_DVBUSDIS_VBUSDT                  (0x0000FFFFUL)            /*!< Device VBUS discharge time */

/********************  Bit definition for OTG register  ********************/
#define USB_OTG_NPTXFSA                 (0x0000FFFFUL)            /*!< Nonperiodic transmit RAM start address */
#define USB_OTG_NPTXFD                  (0xFFFF0000UL)            /*!< Nonperiodic TxFIFO depth */
#define USB_OTG_TX0FSA                  (0x0000FFFFUL)            /*!< Endpoint 0 transmit RAM start address */
#define USB_OTG_TX0FD                   (0xFFFF0000UL)            /*!< Endpoint 0 TxFIFO depth */

/********************  Bit definition forUSB_OTG_DVBUSPULSE register  ********************/
#define USB_OTG_DVBUSPULSE_DVBUSP                  (0x00000FFFUL)            /*!< Device VBUS pulsing time */

/********************  Bit definition forUSB_OTG_GNPTXSTS register  ********************/
#define USB_OTG_GNPTXSTS_NPTXFSAV                (0x0000FFFFUL)            /*!< Nonperiodic TxFIFO space available */

#define USB_OTG_GNPTXSTS_NPTQXSAV                (0x00FF0000UL)            /*!< Nonperiodic transmit request queue space available */
#define USB_OTG_GNPTXSTS_NPTQXSAV_0              (0x00010000UL)            /*!<Bit 0 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_1              (0x00020000UL)            /*!<Bit 1 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_2              (0x00040000UL)            /*!<Bit 2 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_3              (0x00080000UL)            /*!<Bit 3 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_4              (0x00100000UL)            /*!<Bit 4 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_5              (0x00200000UL)            /*!<Bit 5 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_6              (0x00400000UL)            /*!<Bit 6 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_7              (0x00800000UL)            /*!<Bit 7 */

#define USB_OTG_GNPTXSTS_NPTXQTOP                (0x7F000000UL)            /*!< Top of the nonperiodic transmit request queue */
#define USB_OTG_GNPTXSTS_NPTXQTOP_0              (0x01000000UL)            /*!<Bit 0 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_1              (0x02000000UL)            /*!<Bit 1 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_2              (0x04000000UL)            /*!<Bit 2 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_3              (0x08000000UL)            /*!<Bit 3 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_4              (0x10000000UL)            /*!<Bit 4 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_5              (0x20000000UL)            /*!<Bit 5 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_6              (0x40000000UL)            /*!<Bit 6 */

/********************  Bit definition forUSB_OTG_DTHRCTL register  ********************/
#define USB_OTG_DTHRCTL_NONISOTHREN             (0x00000001UL)            /*!< Nonisochronous IN endpoints threshold enable */
#define USB_OTG_DTHRCTL_ISOTHREN                (0x00000002UL)            /*!< ISO IN endpoint threshold enable */

#define USB_OTG_DTHRCTL_TXTHRLEN                (0x000007FCUL)            /*!< Transmit threshold length */
#define USB_OTG_DTHRCTL_TXTHRLEN_0              (0x00000004UL)            /*!<Bit 0 */
#define USB_OTG_DTHRCTL_TXTHRLEN_1              (0x00000008UL)            /*!<Bit 1 */
#define USB_OTG_DTHRCTL_TXTHRLEN_2              (0x00000010UL)            /*!<Bit 2 */
#define USB_OTG_DTHRCTL_TXTHRLEN_3              (0x00000020UL)            /*!<Bit 3 */
#define USB_OTG_DTHRCTL_TXTHRLEN_4              (0x00000040UL)            /*!<Bit 4 */
#define USB_OTG_DTHRCTL_TXTHRLEN_5              (0x00000080UL)            /*!<Bit 5 */
#define USB_OTG_DTHRCTL_TXTHRLEN_6              (0x00000100UL)            /*!<Bit 6 */
#define USB_OTG_DTHRCTL_TXTHRLEN_7              (0x00000200UL)            /*!<Bit 7 */
#define USB_OTG_DTHRCTL_TXTHRLEN_8              (0x00000400UL)            /*!<Bit 8 */
#define USB_OTG_DTHRCTL_RXTHREN                 (0x00010000UL)            /*!< Receive threshold enable */

#define USB_OTG_DTHRCTL_RXTHRLEN                (0x03FE0000UL)            /*!< Receive threshold length */
#define USB_OTG_DTHRCTL_RXTHRLEN_0              (0x00020000UL)            /*!<Bit 0 */
#define USB_OTG_DTHRCTL_RXTHRLEN_1              (0x00040000UL)            /*!<Bit 1 */
#define USB_OTG_DTHRCTL_RXTHRLEN_2              (0x00080000UL)            /*!<Bit 2 */
#define USB_OTG_DTHRCTL_RXTHRLEN_3              (0x00100000UL)            /*!<Bit 3 */
#define USB_OTG_DTHRCTL_RXTHRLEN_4              (0x00200000UL)            /*!<Bit 4 */
#define USB_OTG_DTHRCTL_RXTHRLEN_5              (0x00400000UL)            /*!<Bit 5 */
#define USB_OTG_DTHRCTL_RXTHRLEN_6              (0x00800000UL)            /*!<Bit 6 */
#define USB_OTG_DTHRCTL_RXTHRLEN_7              (0x01000000UL)            /*!<Bit 7 */
#define USB_OTG_DTHRCTL_RXTHRLEN_8              (0x02000000UL)            /*!<Bit 8 */
#define USB_OTG_DTHRCTL_ARPEN                   (0x08000000UL)            /*!< Arbiter parking enable */

/********************  Bit definition forUSB_OTG_DIEPEMPMSK register  ********************/
#define USB_OTG_DIEPEMPMSK_INEPTXFEM               (0x0000FFFFUL)            /*!< IN EP Tx FIFO empty interrupt mask bits */

/********************  Bit definition forUSB_OTG_DEACHINT register  ********************/
#define USB_OTG_DEACHINT_IEP1INT                 (0x00000002UL)            /*!< IN endpoint 1interrupt bit */
#define USB_OTG_DEACHINT_OEP1INT                 (0x00020000UL)            /*!< OUT endpoint 1 interrupt bit */

/********************  Bit definition forUSB_OTG_GCCFG register  ********************/
#define USB_OTG_GCCFG_PWRDWN                  (0x00010000UL)            /*!< Power down */
#define USB_OTG_GCCFG_I2CPADEN                (0x00020000UL)            /*!< Enable I2C bus connection for the external I2C PHY interface */
#define USB_OTG_GCCFG_VBUSASEN                (0x00040000UL)            /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_VBUSBSEN                (0x00080000UL)            /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_SOFOUTEN                (0x00100000UL)            /*!< SOF output enable */
#define USB_OTG_GCCFG_NOVBUSSENS              (0x00200000UL)            /*!< VBUS sensing disable option */

/********************  Bit definition forUSB_OTG_DEACHINTMSK register  ********************/
#define USB_OTG_DEACHINTMSK_IEP1INTM                (0x00000002UL)            /*!< IN Endpoint 1 interrupt mask bit */
#define USB_OTG_DEACHINTMSK_OEP1INTM                (0x00020000UL)            /*!< OUT Endpoint 1 interrupt mask bit */

/********************  Bit definition forUSB_OTG_CID register  ********************/
#define USB_OTG_CID_PRODUCT_ID              (0xFFFFFFFFUL)            /*!< Product ID field */

/********************  Bit definition forUSB_OTG_DIEPEACHMSK1 register  ********************/
#define USB_OTG_DIEPEACHMSK1_XFRCM                   (0x00000001UL)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DIEPEACHMSK1_EPDM                    (0x00000002UL)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DIEPEACHMSK1_TOM                     (0x00000008UL)            /*!< Timeout condition mask (nonisochronous endpointsUL) */
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK               (0x00000010UL)            /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DIEPEACHMSK1_INEPNMM                 (0x00000020UL)            /*!< IN token received with EP mismatch mask */
#define USB_OTG_DIEPEACHMSK1_INEPNEM                 (0x00000040UL)            /*!< IN endpoint NAK effective mask */
#define USB_OTG_DIEPEACHMSK1_TXFURM                  (0x00000100UL)            /*!< FIFO underrun mask */
#define USB_OTG_DIEPEACHMSK1_BIM                     (0x00000200UL)            /*!< BNA interrupt mask */
#define USB_OTG_DIEPEACHMSK1_NAKM                    (0x00002000UL)            /*!< NAK interrupt mask */

/********************  Bit definition forUSB_OTG_HPRT register  ********************/
#define USB_OTG_HPRT_PCSTS                   (0x00000001UL)            /*!< Port connect status */
#define USB_OTG_HPRT_PCDET                   (0x00000002UL)            /*!< Port connect detected */
#define USB_OTG_HPRT_PENA                    (0x00000004UL)            /*!< Port enable */
#define USB_OTG_HPRT_PENCHNG                 (0x00000008UL)            /*!< Port enable/disable change */
#define USB_OTG_HPRT_POCA                    (0x00000010UL)            /*!< Port overcurrent active */
#define USB_OTG_HPRT_POCCHNG                 (0x00000020UL)            /*!< Port overcurrent change */
#define USB_OTG_HPRT_PRES                    (0x00000040UL)            /*!< Port resume */
#define USB_OTG_HPRT_PSUSP                   (0x00000080UL)            /*!< Port suspend */
#define USB_OTG_HPRT_PRST                    (0x00000100UL)            /*!< Port reset */

#define USB_OTG_HPRT_PLSTS                   (0x00000C00UL)            /*!< Port line status */
#define USB_OTG_HPRT_PLSTS_0                 (0x00000400UL)            /*!<Bit 0 */
#define USB_OTG_HPRT_PLSTS_1                 (0x00000800UL)            /*!<Bit 1 */
#define USB_OTG_HPRT_PPWR                    (0x00001000UL)            /*!< Port power */

#define USB_OTG_HPRT_PTCTL                   (0x0001E000UL)            /*!< Port test control */
#define USB_OTG_HPRT_PTCTL_0                 (0x00002000UL)            /*!<Bit 0 */
#define USB_OTG_HPRT_PTCTL_1                 (0x00004000UL)            /*!<Bit 1 */
#define USB_OTG_HPRT_PTCTL_2                 (0x00008000UL)            /*!<Bit 2 */
#define USB_OTG_HPRT_PTCTL_3                 (0x00010000UL)            /*!<Bit 3 */

#define USB_OTG_HPRT_PSPD                    (0x00060000UL)            /*!< Port speed */
#define USB_OTG_HPRT_PSPD_0                  (0x00020000UL)            /*!<Bit 0 */
#define USB_OTG_HPRT_PSPD_1                  (0x00040000UL)            /*!<Bit 1 */

/********************  Bit definition forUSB_OTG_DOEPEACHMSK1 register  ********************/
#define USB_OTG_DOEPEACHMSK1_XFRCM                   (0x00000001UL)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DOEPEACHMSK1_EPDM                    (0x00000002UL)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DOEPEACHMSK1_TOM                     (0x00000008UL)            /*!< Timeout condition mask */
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK               (0x00000010UL)            /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DOEPEACHMSK1_INEPNMM                 (0x00000020UL)            /*!< IN token received with EP mismatch mask */
#define USB_OTG_DOEPEACHMSK1_INEPNEM                 (0x00000040UL)            /*!< IN endpoint NAK effective mask */
#define USB_OTG_DOEPEACHMSK1_TXFURM                  (0x00000100UL)            /*!< OUT packet error mask */
#define USB_OTG_DOEPEACHMSK1_BIM                     (0x00000200UL)            /*!< BNA interrupt mask */
#define USB_OTG_DOEPEACHMSK1_BERRM                   (0x00001000UL)            /*!< Bubble error interrupt mask */
#define USB_OTG_DOEPEACHMSK1_NAKM                    (0x00002000UL)            /*!< NAK interrupt mask */
#define USB_OTG_DOEPEACHMSK1_NYETM                   (0x00004000UL)            /*!< NYET interrupt mask */

/********************  Bit definition forUSB_OTG_HPTXFSIZ register  ********************/
#define USB_OTG_HPTXFSIZ_PTXSA                   (0x0000FFFFUL)            /*!< Host periodic TxFIFO start address */
#define USB_OTG_HPTXFSIZ_PTXFD                   (0xFFFF0000UL)            /*!< Host periodic TxFIFO depth */

/********************  Bit definition forUSB_OTG_DIEPCTL register  ********************/
#define USB_OTG_DIEPCTL_MPSIZ                   (0x000007FFUL)            /*!< Maximum packet size */
#define USB_OTG_DIEPCTL_USBAEP                  (0x00008000UL)            /*!< USB active endpoint */
#define USB_OTG_DIEPCTL_EONUM_DPID              (0x00010000UL)            /*!< Even/odd frame */
#define USB_OTG_DIEPCTL_NAKSTS                  (0x00020000UL)            /*!< NAK status */

#define USB_OTG_DIEPCTL_EPTYP                   (0x000C0000UL)            /*!< Endpoint type */
#define USB_OTG_DIEPCTL_EPTYP_0                 (0x00040000UL)            /*!<Bit 0 */
#define USB_OTG_DIEPCTL_EPTYP_1                 (0x00080000UL)            /*!<Bit 1 */
#define USB_OTG_DIEPCTL_STALL                   (0x00200000UL)            /*!< STALL handshake */

#define USB_OTG_DIEPCTL_TXFNUM                  (0x03C00000UL)            /*!< TxFIFO number */
#define USB_OTG_DIEPCTL_TXFNUM_0                (0x00400000UL)            /*!<Bit 0 */
#define USB_OTG_DIEPCTL_TXFNUM_1                (0x00800000UL)            /*!<Bit 1 */
#define USB_OTG_DIEPCTL_TXFNUM_2                (0x01000000UL)            /*!<Bit 2 */
#define USB_OTG_DIEPCTL_TXFNUM_3                (0x02000000UL)            /*!<Bit 3 */
#define USB_OTG_DIEPCTL_CNAK                    (0x04000000UL)            /*!< Clear NAK */
#define USB_OTG_DIEPCTL_SNAK                    (0x08000000UL)            /*!< Set NAK */
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM          (0x10000000UL)            /*!< Set DATA0 PID */
#define USB_OTG_DIEPCTL_SODDFRM                 (0x20000000UL)            /*!< Set odd frame */
#define USB_OTG_DIEPCTL_EPDIS                   (0x40000000UL)            /*!< Endpoint disable */
#define USB_OTG_DIEPCTL_EPENA                   (0x80000000UL)            /*!< Endpoint enable */

/********************  Bit definition forUSB_OTG_HCCHAR register  ********************/
#define USB_OTG_HCCHAR_MPSIZ                   (0x000007FFUL)            /*!< Maximum packet size */

#define USB_OTG_HCCHAR_EPNUM                   (0x00007800UL)            /*!< Endpoint number */
#define USB_OTG_HCCHAR_EPNUM_0                 (0x00000800UL)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_EPNUM_1                 (0x00001000UL)            /*!<Bit 1 */
#define USB_OTG_HCCHAR_EPNUM_2                 (0x00002000UL)            /*!<Bit 2 */
#define USB_OTG_HCCHAR_EPNUM_3                 (0x00004000UL)            /*!<Bit 3 */
#define USB_OTG_HCCHAR_EPDIR                   (0x00008000UL)            /*!< Endpoint direction */
#define USB_OTG_HCCHAR_LSDEV                   (0x00020000UL)            /*!< Low-speed device */

#define USB_OTG_HCCHAR_EPTYP                   (0x000C0000UL)            /*!< Endpoint type */
#define USB_OTG_HCCHAR_EPTYP_0                 (0x00040000UL)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_EPTYP_1                 (0x00080000UL)            /*!<Bit 1 */

#define USB_OTG_HCCHAR_MC                      (0x00300000UL)            /*!< Multi Count (MC) / Error Count (EC) */
#define USB_OTG_HCCHAR_MC_0                    (0x00100000UL)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_MC_1                    (0x00200000UL)            /*!<Bit 1 */

#define USB_OTG_HCCHAR_DAD                     (0x1FC00000UL)            /*!< Device address */
#define USB_OTG_HCCHAR_DAD_0                   (0x00400000UL)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_DAD_1                   (0x00800000UL)            /*!<Bit 1 */
#define USB_OTG_HCCHAR_DAD_2                   (0x01000000UL)            /*!<Bit 2 */
#define USB_OTG_HCCHAR_DAD_3                   (0x02000000UL)            /*!<Bit 3 */
#define USB_OTG_HCCHAR_DAD_4                   (0x04000000UL)            /*!<Bit 4 */
#define USB_OTG_HCCHAR_DAD_5                   (0x08000000UL)            /*!<Bit 5 */
#define USB_OTG_HCCHAR_DAD_6                   (0x10000000UL)            /*!<Bit 6 */
#define USB_OTG_HCCHAR_ODDFRM                  (0x20000000UL)            /*!< Odd frame */
#define USB_OTG_HCCHAR_CHDIS                   (0x40000000UL)            /*!< Channel disable */
#define USB_OTG_HCCHAR_CHENA                   (0x80000000UL)            /*!< Channel enable */

/********************  Bit definition forUSB_OTG_HCSPLT register  ********************/

#define USB_OTG_HCSPLT_PRTADDR                 (0x0000007FUL)            /*!< Port address */
#define USB_OTG_HCSPLT_PRTADDR_0               (0x00000001UL)            /*!<Bit 0 */
#define USB_OTG_HCSPLT_PRTADDR_1               (0x00000002UL)            /*!<Bit 1 */
#define USB_OTG_HCSPLT_PRTADDR_2               (0x00000004UL)            /*!<Bit 2 */
#define USB_OTG_HCSPLT_PRTADDR_3               (0x00000008UL)            /*!<Bit 3 */
#define USB_OTG_HCSPLT_PRTADDR_4               (0x00000010UL)            /*!<Bit 4 */
#define USB_OTG_HCSPLT_PRTADDR_5               (0x00000020UL)            /*!<Bit 5 */
#define USB_OTG_HCSPLT_PRTADDR_6               (0x00000040UL)            /*!<Bit 6 */

#define USB_OTG_HCSPLT_HUBADDR                 (0x00003F80UL)            /*!< Hub address */
#define USB_OTG_HCSPLT_HUBADDR_0               (0x00000080UL)            /*!<Bit 0 */
#define USB_OTG_HCSPLT_HUBADDR_1               (0x00000100UL)            /*!<Bit 1 */
#define USB_OTG_HCSPLT_HUBADDR_2               (0x00000200UL)            /*!<Bit 2 */
#define USB_OTG_HCSPLT_HUBADDR_3               (0x00000400UL)            /*!<Bit 3 */
#define USB_OTG_HCSPLT_HUBADDR_4               (0x00000800UL)            /*!<Bit 4 */
#define USB_OTG_HCSPLT_HUBADDR_5               (0x00001000UL)            /*!<Bit 5 */
#define USB_OTG_HCSPLT_HUBADDR_6               (0x00002000UL)            /*!<Bit 6 */

#define USB_OTG_HCSPLT_XACTPOS                 (0x0000C000UL)            /*!< XACTPOS */
#define USB_OTG_HCSPLT_XACTPOS_0               (0x00004000UL)            /*!<Bit 0 */
#define USB_OTG_HCSPLT_XACTPOS_1               (0x00008000UL)            /*!<Bit 1 */
#define USB_OTG_HCSPLT_COMPLSPLT               (0x00010000UL)            /*!< Do complete split */
#define USB_OTG_HCSPLT_SPLITEN                 (0x80000000UL)            /*!< Split enable */

/********************  Bit definition forUSB_OTG_HCINT register  ********************/
#define USB_OTG_HCINT_XFRC                    (0x00000001UL)            /*!< Transfer completed */
#define USB_OTG_HCINT_CHH                     (0x00000002UL)            /*!< Channel halted */
#define USB_OTG_HCINT_AHBERR                  (0x00000004UL)            /*!< AHB error */
#define USB_OTG_HCINT_STALL                   (0x00000008UL)            /*!< STALL response received interrupt */
#define USB_OTG_HCINT_NAK                     (0x00000010UL)            /*!< NAK response received interrupt */
#define USB_OTG_HCINT_ACK                     (0x00000020UL)            /*!< ACK response received/transmitted interrupt */
#define USB_OTG_HCINT_NYET                    (0x00000040UL)            /*!< Response received interrupt */
#define USB_OTG_HCINT_TXERR                   (0x00000080UL)            /*!< Transaction error */
#define USB_OTG_HCINT_BBERR                   (0x00000100UL)            /*!< Babble error */
#define USB_OTG_HCINT_FRMOR                   (0x00000200UL)            /*!< Frame overrun */
#define USB_OTG_HCINT_DTERR                   (0x00000400UL)            /*!< Data toggle error */

/********************  Bit definition forUSB_OTG_DIEPINT register  ********************/
#define USB_OTG_DIEPINT_XFRC                    (0x00000001UL)            /*!< Transfer completed interrupt */
#define USB_OTG_DIEPINT_EPDISD                  (0x00000002UL)            /*!< Endpoint disabled interrupt */
#define USB_OTG_DIEPINT_TOC                     (0x00000008UL)            /*!< Timeout condition */
#define USB_OTG_DIEPINT_ITTXFE                  (0x00000010UL)            /*!< IN token received when TxFIFO is empty */
#define USB_OTG_DIEPINT_INEPNE                  (0x00000040UL)            /*!< IN endpoint NAK effective */
#define USB_OTG_DIEPINT_TXFE                    (0x00000080UL)            /*!< Transmit FIFO empty */
#define USB_OTG_DIEPINT_TXFIFOUDRN              (0x00000100UL)            /*!< Transmit Fifo Underrun */
#define USB_OTG_DIEPINT_BNA                     (0x00000200UL)            /*!< Buffer not available interrupt */
#define USB_OTG_DIEPINT_PKTDRPSTS               (0x00000800UL)            /*!< Packet dropped status */
#define USB_OTG_DIEPINT_BERR                    (0x00001000UL)            /*!< Babble error interrupt */
#define USB_OTG_DIEPINT_NAK                     (0x00002000UL)            /*!< NAK interrupt */

/********************  Bit definition forUSB_OTG_HCINTMSK register  ********************/
#define USB_OTG_HCINTMSK_XFRCM                   (0x00000001UL)            /*!< Transfer completed mask */
#define USB_OTG_HCINTMSK_CHHM                    (0x00000002UL)            /*!< Channel halted mask */
#define USB_OTG_HCINTMSK_AHBERR                  (0x00000004UL)            /*!< AHB error */
#define USB_OTG_HCINTMSK_STALLM                  (0x00000008UL)            /*!< STALL response received interrupt mask */
#define USB_OTG_HCINTMSK_NAKM                    (0x00000010UL)            /*!< NAK response received interrupt mask */
#define USB_OTG_HCINTMSK_ACKM                    (0x00000020UL)            /*!< ACK response received/transmitted interrupt mask */
#define USB_OTG_HCINTMSK_NYET                    (0x00000040UL)            /*!< response received interrupt mask */
#define USB_OTG_HCINTMSK_TXERRM                  (0x00000080UL)            /*!< Transaction error mask */
#define USB_OTG_HCINTMSK_BBERRM                  (0x00000100UL)            /*!< Babble error mask */
#define USB_OTG_HCINTMSK_FRMORM                  (0x00000200UL)            /*!< Frame overrun mask */
#define USB_OTG_HCINTMSK_DTERRM                  (0x00000400UL)            /*!< Data toggle error mask */

/********************  Bit definition for USB_OTG_DIEPTSIZ register  ********************/

#define USB_OTG_DIEPTSIZ_XFRSIZ                  (0x0007FFFFUL)            /*!< Transfer size */
#define USB_OTG_DIEPTSIZ_PKTCNT                  (0x1FF80000UL)            /*!< Packet count */
#define USB_OTG_DIEPTSIZ_MULCNT                  (0x60000000UL)            /*!< Packet count */
/********************  Bit definition forUSB_OTG_HCTSIZ register  ********************/
#define USB_OTG_HCTSIZ_XFRSIZ                    (0x0007FFFFUL)            /*!< Transfer size */
#define USB_OTG_HCTSIZ_PKTCNT                    (0x1FF80000UL)            /*!< Packet count */
#define USB_OTG_HCTSIZ_DOPING                    (0x80000000UL)            /*!< Do PING */
#define USB_OTG_HCTSIZ_DPID                      (0x60000000UL)            /*!< Data PID */
#define USB_OTG_HCTSIZ_DPID_0                    (0x20000000UL)            /*!<Bit 0 */
#define USB_OTG_HCTSIZ_DPID_1                    (0x40000000UL)            /*!<Bit 1 */

/********************  Bit definition forUSB_OTG_DIEPDMA register  ********************/
#define USB_OTG_DIEPDMA_DMAADDR                  (0xFFFFFFFFUL)            /*!< DMA address */

/********************  Bit definition forUSB_OTG_HCDMA register  ********************/
#define USB_OTG_HCDMA_DMAADDR                    (0xFFFFFFFFUL)            /*!< DMA address */

/********************  Bit definition forUSB_OTG_DTXFSTS register  ********************/
#define USB_OTG_DTXFSTS_INEPTFSAV                (0x0000FFFFUL)            /*!< IN endpoint TxFIFO space avail */

/********************  Bit definition forUSB_OTG_DIEPTXF register  ********************/
#define USB_OTG_DIEPTXF_INEPTXSA                 (0x0000FFFFUL)            /*!< IN endpoint FIFOx transmit RAM start address */
#define USB_OTG_DIEPTXF_INEPTXFD                 (0xFFFF0000UL)            /*!< IN endpoint TxFIFO depth */

/********************  Bit definition forUSB_OTG_DOEPCTL register  ********************/

#define USB_OTG_DOEPCTL_MPSIZ                     (0x000007FFUL)            /*!< Maximum packet size */          /*!<Bit 1 */
#define USB_OTG_DOEPCTL_USBAEP                    (0x00008000UL)            /*!< USB active endpoint */
#define USB_OTG_DOEPCTL_NAKSTS                    (0x00020000UL)            /*!< NAK status */
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM            (0x10000000UL)            /*!< Set DATA0 PID */
#define USB_OTG_DOEPCTL_SODDFRM                   (0x20000000UL)            /*!< Set odd frame */
#define USB_OTG_DOEPCTL_EPTYP                     (0x000C0000UL)            /*!< Endpoint type */
#define USB_OTG_DOEPCTL_EPTYP_0                   (0x00040000UL)            /*!<Bit 0 */
#define USB_OTG_DOEPCTL_EPTYP_1                   (0x00080000UL)            /*!<Bit 1 */
#define USB_OTG_DOEPCTL_SNPM                      (0x00100000UL)            /*!< Snoop mode */
#define USB_OTG_DOEPCTL_STALL                     (0x00200000UL)            /*!< STALL handshake */
#define USB_OTG_DOEPCTL_CNAK                      (0x04000000UL)            /*!< Clear NAK */
#define USB_OTG_DOEPCTL_SNAK                      (0x08000000UL)            /*!< Set NAK */
#define USB_OTG_DOEPCTL_EPDIS                     (0x40000000UL)            /*!< Endpoint disable */
#define USB_OTG_DOEPCTL_EPENA                     (0x80000000UL)            /*!< Endpoint enable */

/********************  Bit definition forUSB_OTG_DOEPINT register  ********************/
#define USB_OTG_DOEPINT_XFRC                    (0x00000001UL)            /*!< Transfer completed interrupt */
#define USB_OTG_DOEPINT_EPDISD                  (0x00000002UL)            /*!< Endpoint disabled interrupt */
#define USB_OTG_DOEPINT_STUP                    (0x00000008UL)            /*!< SETUP phase done */
#define USB_OTG_DOEPINT_OTEPDIS                 (0x00000010UL)            /*!< OUT token received when endpoint disabled */
#define USB_OTG_DOEPINT_B2BSTUP                 (0x00000040UL)            /*!< Back-to-back SETUP packets received */
#define USB_OTG_DOEPINT_NYET                    (0x00004000UL)            /*!< NYET interrupt */

/********************  Bit definition forUSB_OTG_DOEPTSIZ register  ********************/

#define USB_OTG_DOEPTSIZ_XFRSIZ                  (0x0007FFFFUL)            /*!< Transfer size */
#define USB_OTG_DOEPTSIZ_PKTCNT                  (0x1FF80000UL)            /*!< Packet count */

#define USB_OTG_DOEPTSIZ_STUPCNT                 (0x60000000UL)            /*!< SETUP packet count */
#define USB_OTG_DOEPTSIZ_STUPCNT_0               (0x20000000UL)            /*!<Bit 0 */
#define USB_OTG_DOEPTSIZ_STUPCNT_1               (0x40000000UL)            /*!<Bit 1 */

/********************  Bit definition for PCGCCTL register  ********************/
#define USB_OTG_PCGCCTL_STOPCLK                 (0x00000001UL)            /*!< SETUP packet count */
#define USB_OTG_PCGCCTL_GATECLK                 (0x00000002UL)            /*!<Bit 0 */
#define USB_OTG_PCGCCTL_PHYSUSP                 (0x00000010UL)            /*!<Bit 1 */

/****************************** USB Exported Constants ************************/
#define USB_OTG_FS_HOST_MAX_CHANNEL_NBR                8
#define USB_OTG_FS_MAX_IN_ENDPOINTS                    4    /* Including EP0 */
#define USB_OTG_FS_MAX_OUT_ENDPOINTS                   4    /* Including EP0 */
#define USB_OTG_FS_TOTAL_FIFO_SIZE                     1280 /* in Bytes */

#endif /* __STM32F4xx_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
