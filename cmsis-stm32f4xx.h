/**
  ******************************************************************************
  * @file    stm32f4xx.h
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    09-November-2016
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer Header File.
  *          This file contains all the peripheral register's definitions, bits
  *          definitions and memory mapping for STM32F4xx devices.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The device used in the target application
  *              - To use or not the peripheral’s drivers in application code(i.e.
  *                code will be based on direct access to peripheral’s registers
  *                rather than drivers API), this option is controlled by
  *                "#define USE_STDPERIPH_DRIVER"
  *              - To change few application-specific parameters such as the HSE
  *                crystal frequency
  *           - Data structures and the address mapping for all peripherals
  *           - Peripherals registers declarations and bits definition
  *           - Macros to access peripheral’s registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* changes for Teacup
    - replace type-casts in defines with 'UL', so we can use this in precompiler macros
*/

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx
  * @{
  */

#ifndef __STM32F4xx_H
#define __STM32F4xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/* Uncomment the line below according to the target STM32 device used in your
   application
  */

#if !defined(STM32F40_41xxx) && !defined(STM32F427_437xx) && !defined(STM32F429_439xx) && !defined(STM32F401xx) && !defined(STM32F410xx) && \
    !defined(STM32F411xE) && !defined(STM32F412xG) && !defined(STM32F413_423xx) && !defined(STM32F446xx) && !defined(STM32F469_479xx)
  /* #define STM32F40_41xxx */   /*!< STM32F405RG, STM32F405VG, STM32F405ZG, STM32F415RG, STM32F415VG, STM32F415ZG,
                                      STM32F407VG, STM32F407VE, STM32F407ZG, STM32F407ZE, STM32F407IG, STM32F407IE,
                                      STM32F417VG, STM32F417VE, STM32F417ZG, STM32F417ZE, STM32F417IG and STM32F417IE Devices */

  /* #define STM32F427_437xx */  /*!< STM32F427VG, STM32F427VI, STM32F427ZG, STM32F427ZI, STM32F427IG, STM32F427II,
                                      STM32F437VG, STM32F437VI, STM32F437ZG, STM32F437ZI, STM32F437IG, STM32F437II Devices */

  /* #define STM32F429_439xx */  /*!< STM32F429VG, STM32F429VI, STM32F429ZG, STM32F429ZI, STM32F429BG, STM32F429BI,
                                      STM32F429NG, STM32F439NI, STM32F429IG, STM32F429II, STM32F439VG, STM32F439VI,
                                      STM32F439ZG, STM32F439ZI, STM32F439BG, STM32F439BI, STM32F439NG, STM32F439NI,
                                      STM32F439IG and STM32F439II Devices */

  /* #define STM32F401xx */      /*!< STM32F401CB, STM32F401CC,  STM32F401RB, STM32F401RC, STM32F401VB, STM32F401VC,
                                      STM32F401CD, STM32F401RD, STM32F401VD, STM32F401CExx, STM32F401RE and STM32F401VE Devices */

  /* #define STM32F410xx */      /*!< STM32F410Tx, STM32F410Cx and STM32F410Rx */

  /* #define STM32F411xE */      /*!< STM32F411CC, STM32F411RC, STM32F411VC, STM32F411CE, STM32F411RE and STM32F411VE Devices */

  /* #define STM32F412xG */      /*!< STM32F412CEU, STM32F412CGU, STM32F412ZET, STM32F412ZGT, STM32F412ZEJ, STM32F412ZGJ,
                                      STM32F412VET, STM32F412VGT, STM32F412VEH, STM32F412VGH, STM32F412RET, STM32F412RGT,
                                      STM32F412REY and STM32F412RGY Devices */

  /* #define STM32F413_423xx */  /*!< STM32F413CGU, STM32F413CHU, STM32F413MGY, STM32F413MHY, STM32F413RGT, STM32F413VGT,
                                      STM32F413ZGT, STM32F413RHT, STM32F413VHT, STM32F413ZHT, STM32F413VGH, STM32F413ZGJ,
                                      STM32F413VHH, STM32F413ZHJ, STM32F423CHU, STM32F423RHT, STM32F423VHT, STM32F423ZHT,
                                      STM32F423VHH and STM32F423ZHJ devices */

  /* #define STM32F446xx */      /*!< STM32F446MC, STM32F446ME, STM32F446RC, STM32F446RE, STM32F446VC, STM32F446VE, STM32F446ZC
                                      and STM32F446ZE Devices */

  /* #define STM32F469_479xx */  /*!< STM32F479AI, STM32F479II, STM32F479BI, STM32F479NI, STM32F479AG, STM32F479IG, STM32F479BG,
                                      STM32F479NG, STM32F479AE, STM32F479IE, STM32F479BE, STM32F479NE Devices */

#endif /* STM32F40_41xxx && STM32F427_437xx && STM32F429_439xx && STM32F401xx && STM32F410xx && STM32F411xE && STM32F412xG && STM32F413_423xx && STM32F446xx && STM32F469_479xx */

/* Old STM32F40XX definition, maintained for legacy purpose */
#ifdef STM32F40XX
  #define STM32F40_41xxx
#endif /* STM32F40XX */

/* Old STM32F427X definition, maintained for legacy purpose */
#ifdef STM32F427X
  #define STM32F427_437xx
#endif /* STM32F427X */

/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */

#if !defined(STM32F40_41xxx) && !defined(STM32F427_437xx) && !defined(STM32F429_439xx) && !defined(STM32F401xx) && !defined(STM32F410xx) && \
    !defined(STM32F411xE) && !defined(STM32F412xG) && !defined(STM32F413_423xx) && !defined(STM32F446xx) && !defined(STM32F469_479xx)
 #error "Please select first the target STM32F4xx device used in your application (in stm32f4xx.h file)"
#endif /* STM32F40_41xxx && STM32F427_437xx && STM32F429_439xx && STM32F401xx && STM32F410xx && STM32F411xE && STM32F412xG && STM32F413_23xx && STM32F446xx && STM32F469_479xx */

#if !defined  (USE_STDPERIPH_DRIVER)
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will
   be based on direct access to peripherals registers
   */
  /*#define USE_STDPERIPH_DRIVER */
#endif /* USE_STDPERIPH_DRIVER */

/**
 * @brief In the following line adjust the value of External High Speed oscillator (HSE)
   used in your application

   Tip: To avoid modifying this file each time you need to use different HSE, you
        can define the HSE value in your toolchain compiler preprocessor.
  */
#if defined(STM32F40_41xxx) || defined(STM32F427_437xx)  || defined(STM32F429_439xx) || defined(STM32F401xx) || \
    defined(STM32F410xx) || defined(STM32F411xE) || defined(STM32F469_479xx)
 #if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)25000000) /*!< Value of the External oscillator in Hz */
 #endif /* HSE_VALUE */
#elif defined (STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
 #if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)8000000) /*!< Value of the External oscillator in Hz */
 #endif /* HSE_VALUE */
#endif /* STM32F40_41xxx || STM32F427_437xx || STM32F429_439xx || STM32F401xx || STM32F411xE || STM32F469_479xx */
/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup
   Timeout value
   */
#if !defined  (HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    (0x05000U)   /*!< Time out for HSE start up */
#endif /* HSE_STARTUP_TIMEOUT */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
 * @brief STM32F4XX Standard Peripherals Library version number V1.8.0
   */
#define __STM32F4XX_STDPERIPH_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32F4XX_STDPERIPH_VERSION_SUB1   (0x08) /*!< [23:16] sub1 version */
#define __STM32F4XX_STDPERIPH_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __STM32F4XX_STDPERIPH_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32F4XX_STDPERIPH_VERSION        ((__STM32F4XX_STDPERIPH_VERSION_MAIN << 24)\
                                             |(__STM32F4XX_STDPERIPH_VERSION_SUB1 << 16)\
                                             |(__STM32F4XX_STDPERIPH_VERSION_SUB2 << 8)\
                                             |(__STM32F4XX_STDPERIPH_VERSION_RC))

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
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum IRQn
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

#if defined(STM32F40_41xxx)
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
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
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                             */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  CRYP_IRQn                   = 79,     /*!< CRYP crypto global interrupt                                      */
  HASH_RNG_IRQn               = 80,     /*!< Hash and Rng global interrupt                                     */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                              */
#endif /* STM32F40_41xxx */

#if defined(STM32F427_437xx)
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
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
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  CRYP_IRQn                   = 79,     /*!< CRYP crypto global interrupt                                      */
  HASH_RNG_IRQn               = 80,     /*!< Hash and Rng global interrupt                                     */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  DMA2D_IRQn                  = 90      /*!< DMA2D global Interrupt                                            */
#endif /* STM32F427_437xx */

#if defined(STM32F429_439xx)
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
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
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  CRYP_IRQn                   = 79,     /*!< CRYP crypto global interrupt                                      */
  HASH_RNG_IRQn               = 80,     /*!< Hash and Rng global interrupt                                     */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  LTDC_IRQn                   = 88,     /*!< LTDC global Interrupt                                             */
  LTDC_ER_IRQn                = 89,     /*!< LTDC Error global Interrupt                                       */
  DMA2D_IRQn                  = 90      /*!< DMA2D global Interrupt                                            */
#endif /* STM32F429_439xx */

#if defined(STM32F410xx)
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                             */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
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
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global Interrupt and DAC Global Interrupt                    */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  RNG_IRQn                    = 80,     /*!< RNG global Interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96,     /*!< FMPI2C1 Error Interrupt                                           */
  LPTIM1_IRQn                 = 97      /*!< LPTIM1 interrupt                                                  */
#endif /* STM32F410xx */

#if defined(STM32F401xx) || defined(STM32F411xE)
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
  FPU_IRQn                    = 81,      /*!< FPU global interrupt                                             */
#if defined(STM32F401xx)
  SPI4_IRQn                   = 84       /*!< SPI4 global Interrupt                                            */
#endif /* STM32F411xE */
#if defined(STM32F411xE)
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85      /*!< SPI5 global Interrupt                                             */
#endif /* STM32F411xE */
#endif /* STM32F401xx || STM32F411xE */

#if defined(STM32F469_479xx)
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
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
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  CRYP_IRQn                   = 79,     /*!< CRYP crypto global interrupt                                      */
  HASH_RNG_IRQn               = 80,     /*!< Hash and Rng global interrupt                                     */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  LTDC_IRQn                   = 88,     /*!< LTDC global Interrupt                                             */
  LTDC_ER_IRQn                = 89,     /*!< LTDC Error global Interrupt                                       */
  DMA2D_IRQn                  = 90,     /*!< DMA2D global Interrupt                                            */
  QUADSPI_IRQn                = 91,     /*!< QUADSPI global Interrupt                                          */
  DSI_IRQn                    = 92      /*!< DSI global Interrupt                                              */
#endif /* STM32F469_479xx */

#if defined(STM32F446xx)
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
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
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                              */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
  CEC_IRQn                    = 93,     /*!< QuadSPI global Interrupt                                          */
  SPDIF_RX_IRQn               = 94,     /*!< QuadSPI global Interrupt                                          */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C Event Interrupt                                            */
  FMPI2C1_ER_IRQn             = 96      /*!< FMPCI2C Error Interrupt                                           */
#endif /* STM32F446xx */

#if defined(STM32F412xG)
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
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
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  TIM6_IRQn                   = 54,     /*!< TIM6 global interrupt                                             */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  DFSDM1_FLT0_IRQn            = 61,     /*!< DFSDM1 Filter 0 global Interrupt                                  */
  DFSDM1_FLT1_IRQn            = 62,     /*!< DFSDM1 Filter 1 global Interrupt                                  */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  RNG_IRQn                    = 80,     /*!< RNG global Interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,      /*!< SPI5 global Interrupt                                            */
  QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96      /*!< FMPI2C1 Error Interrupt                                           */
#endif /* STM32F412xG */

#if defined(STM32F413_423xx)
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
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
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 and DAC1&2 global Interrupt                                  */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  DFSDM1_FLT0_IRQn            = 61,     /*!< DFSDM1 Filter 0 global Interrupt                                  */
  DFSDM1_FLT1_IRQn            = 62,     /*!< DFSDM1 Filter 1 global Interrupt                                  */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  CAN3_TX_IRQn                = 74,     /*!< CAN3 TX Interrupt                                                 */
  CAN3_RX0_IRQn               = 75,     /*!< CAN3 RX0 Interrupt                                                */
  CAN3_RX1_IRQn               = 76,     /*!< CAN3 RX1 Interrupt                                                */
  CAN3_SCE_IRQn               = 77,     /*!< CAN3 SCE Interrupt                                                */
  RNG_IRQn                    = 80,     /*!< RNG global Interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< Serial Audio Interface 1 global interrupt                         */
  UART9_IRQn                  = 88,     /*!< UART9 global Interrupt                                            */
  UART10_IRQn                 = 89,     /*!< UART10 global Interrupt                                           */
  QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96,     /*!< FMPI2C1 Error Interrupt                                           */
  LPTIM1_IRQn                 = 97,     /*!< LP TIM1 interrupt                                                 */
  DFSDM2_FLT0_IRQn            = 98,     /*!< DFSDM2 Filter 0 global Interrupt                                  */
  DFSDM2_FLT1_IRQn            = 99,     /*!< DFSDM2 Filter 1 global Interrupt                                  */
  DFSDM2_FLT2_IRQn            = 100,    /*!< DFSDM2 Filter 2 global Interrupt                                  */
  DFSDM2_FLT3_IRQn            = 101     /*!< DFSDM2 Filter 3 global Interrupt                                  */
#endif /* STM32F413_423xx */
} IRQn_Type;

/**
  * @}
  */

#include "cmsis-core_cm4.h"             /* Cortex-M4 processor and core peripherals */
#include "cmsis-system_stm32f4xx.h"
#include <stdint.h>

/** @addtogroup Exported_types
  * @{
  */
/*!< STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/**
  * @}
  */

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
  __IO uint32_t JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38 */
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
  * @brief Controller Area Network TxMailBox
  */

typedef struct
{
  __IO uint32_t TIR;  /*!< CAN TX mailbox identifier register */
  __IO uint32_t TDTR; /*!< CAN mailbox data length control and time stamp register */
  __IO uint32_t TDLR; /*!< CAN mailbox data low register */
  __IO uint32_t TDHR; /*!< CAN mailbox data high register */
} CAN_TxMailBox_TypeDef;

/**
  * @brief Controller Area Network FIFOMailBox
  */

typedef struct
{
  __IO uint32_t RIR;  /*!< CAN receive FIFO mailbox identifier register */
  __IO uint32_t RDTR; /*!< CAN receive FIFO mailbox data length control and time stamp register */
  __IO uint32_t RDLR; /*!< CAN receive FIFO mailbox data low register */
  __IO uint32_t RDHR; /*!< CAN receive FIFO mailbox data high register */
} CAN_FIFOMailBox_TypeDef;

/**
  * @brief Controller Area Network FilterRegister
  */

typedef struct
{
  __IO uint32_t FR1; /*!< CAN Filter bank register 1 */
  __IO uint32_t FR2; /*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef;

/**
  * @brief Controller Area Network
  */

typedef struct
{
  __IO uint32_t              MCR;                 /*!< CAN master control register,         Address offset: 0x00          */
  __IO uint32_t              MSR;                 /*!< CAN master status register,          Address offset: 0x04          */
  __IO uint32_t              TSR;                 /*!< CAN transmit status register,        Address offset: 0x08          */
  __IO uint32_t              RF0R;                /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
  __IO uint32_t              RF1R;                /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
  __IO uint32_t              IER;                 /*!< CAN interrupt enable register,       Address offset: 0x14          */
  __IO uint32_t              ESR;                 /*!< CAN error status register,           Address offset: 0x18          */
  __IO uint32_t              BTR;                 /*!< CAN bit timing register,             Address offset: 0x1C          */
  uint32_t                   RESERVED0[88];       /*!< Reserved, 0x020 - 0x17F                                            */
  CAN_TxMailBox_TypeDef      sTxMailBox[3];       /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];     /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
  uint32_t                   RESERVED1[12];       /*!< Reserved, 0x1D0 - 0x1FF                                            */
  __IO uint32_t              FMR;                 /*!< CAN filter master register,          Address offset: 0x200         */
  __IO uint32_t              FM1R;                /*!< CAN filter mode register,            Address offset: 0x204         */
  uint32_t                   RESERVED2;           /*!< Reserved, 0x208                                                    */
  __IO uint32_t              FS1R;                /*!< CAN filter scale register,           Address offset: 0x20C         */
  uint32_t                   RESERVED3;           /*!< Reserved, 0x210                                                    */
  __IO uint32_t              FFA1R;               /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
  uint32_t                   RESERVED4;           /*!< Reserved, 0x218                                                    */
  __IO uint32_t              FA1R;                /*!< CAN filter activation register,      Address offset: 0x21C         */
  uint32_t                   RESERVED5[8];        /*!< Reserved, 0x220-0x23F                                              */
  CAN_FilterRegister_TypeDef sFilterRegister[28]; /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CAN_TypeDef;

#if defined(STM32F446xx)
/**
  * @brief Consumer Electronics Control
  */
typedef struct
{
  __IO uint32_t CR;           /*!< CEC control register,              Address offset:0x00 */
  __IO uint32_t CFGR;         /*!< CEC configuration register,        Address offset:0x04 */
  __IO uint32_t TXDR;         /*!< CEC Tx data register ,             Address offset:0x08 */
  __IO uint32_t RXDR;         /*!< CEC Rx Data Register,              Address offset:0x0C */
  __IO uint32_t ISR;          /*!< CEC Interrupt and Status Register, Address offset:0x10 */
  __IO uint32_t IER;          /*!< CEC interrupt enable register,     Address offset:0x14 */
}CEC_TypeDef;
#endif /* STM32F446xx */

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
  * @brief Digital to Analog Converter
  */

typedef struct
{
  __IO uint32_t CR;       /*!< DAC control register,                                    Address offset: 0x00 */
  __IO uint32_t SWTRIGR;  /*!< DAC software trigger register,                           Address offset: 0x04 */
  __IO uint32_t DHR12R1;  /*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
  __IO uint32_t DHR12L1;  /*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
  __IO uint32_t DHR8R1;   /*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
  __IO uint32_t DHR12R2;  /*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
  __IO uint32_t DHR12L2;  /*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
  __IO uint32_t DHR8R2;   /*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
  __IO uint32_t DHR12RD;  /*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
  __IO uint32_t DHR12LD;  /*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
  __IO uint32_t DHR8RD;   /*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
  __IO uint32_t DOR1;     /*!< DAC channel1 data output register,                       Address offset: 0x2C */
  __IO uint32_t DOR2;     /*!< DAC channel2 data output register,                       Address offset: 0x30 */
  __IO uint32_t SR;       /*!< DAC status register,                                     Address offset: 0x34 */
} DAC_TypeDef;

#if defined(STM32F412xG) || defined(STM32F413_423xx)
/**
  * @brief DFSDM module registers
  */
typedef struct
{
  __IO uint32_t FLTCR1;         /*!< DFSDM control register1,                          Address offset: 0x100 */
  __IO uint32_t FLTCR2;         /*!< DFSDM control register2,                          Address offset: 0x104 */
  __IO uint32_t FLTISR;         /*!< DFSDM interrupt and status register,              Address offset: 0x108 */
  __IO uint32_t FLTICR;         /*!< DFSDM interrupt flag clear register,              Address offset: 0x10C */
  __IO uint32_t FLTJCHGR;       /*!< DFSDM injected channel group selection register,  Address offset: 0x110 */
  __IO uint32_t FLTFCR;         /*!< DFSDM filter control register,                    Address offset: 0x114 */
  __IO uint32_t FLTJDATAR;      /*!< DFSDM data register for injected group,           Address offset: 0x118 */
  __IO uint32_t FLTRDATAR;      /*!< DFSDM data register for regular group,            Address offset: 0x11C */
  __IO uint32_t FLTAWHTR;       /*!< DFSDM analog watchdog high threshold register,    Address offset: 0x120 */
  __IO uint32_t FLTAWLTR;       /*!< DFSDM analog watchdog low threshold register,     Address offset: 0x124 */
  __IO uint32_t FLTAWSR;        /*!< DFSDM analog watchdog status register             Address offset: 0x128 */
  __IO uint32_t FLTAWCFR;       /*!< DFSDM analog watchdog clear flag register         Address offset: 0x12C */
  __IO uint32_t FLTEXMAX;       /*!< DFSDM extreme detector maximum register,          Address offset: 0x130 */
  __IO uint32_t FLTEXMIN;       /*!< DFSDM extreme detector minimum register           Address offset: 0x134 */
  __IO uint32_t FLTCNVTIMR;     /*!< DFSDM conversion timer,                           Address offset: 0x138 */
} DFSDM_Filter_TypeDef;

/**
  * @brief DFSDM channel configuration registers
  */
typedef struct
{
  __IO uint32_t CHCFGR1;     /*!< DFSDM channel configuration register1,            Address offset: 0x00 */
  __IO uint32_t CHCFGR2;     /*!< DFSDM channel configuration register2,            Address offset: 0x04 */
  __IO uint32_t CHAWSCDR;    /*!< DFSDM channel analog watchdog and
                                  short circuit detector register,                  Address offset: 0x08 */
  __IO uint32_t CHWDATAR;    /*!< DFSDM channel watchdog filter data register,      Address offset: 0x0C */
  __IO uint32_t CHDATINR;    /*!< DFSDM channel data input register,                Address offset: 0x10 */
} DFSDM_Channel_TypeDef;

/* Legacy Defines */
#define DFSDM_TypeDef        DFSDM_Filter_TypeDef
#endif /* STM32F412xG || STM32F413_423xx */
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
  * @brief DCMI
  */

typedef struct
{
  __IO uint32_t CR;       /*!< DCMI control register 1,                       Address offset: 0x00 */
  __IO uint32_t SR;       /*!< DCMI status register,                          Address offset: 0x04 */
  __IO uint32_t RISR;     /*!< DCMI raw interrupt status register,            Address offset: 0x08 */
  __IO uint32_t IER;      /*!< DCMI interrupt enable register,                Address offset: 0x0C */
  __IO uint32_t MISR;     /*!< DCMI masked interrupt status register,         Address offset: 0x10 */
  __IO uint32_t ICR;      /*!< DCMI interrupt clear register,                 Address offset: 0x14 */
  __IO uint32_t ESCR;     /*!< DCMI embedded synchronization code register,   Address offset: 0x18 */
  __IO uint32_t ESUR;     /*!< DCMI embedded synchronization unmask register, Address offset: 0x1C */
  __IO uint32_t CWSTRTR;  /*!< DCMI crop window start,                        Address offset: 0x20 */
  __IO uint32_t CWSIZER;  /*!< DCMI crop window size,                         Address offset: 0x24 */
  __IO uint32_t DR;       /*!< DCMI data register,                            Address offset: 0x28 */
} DCMI_TypeDef;

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
  * @brief DMA2D Controller
  */

typedef struct
{
  __IO uint32_t CR;            /*!< DMA2D Control Register,                         Address offset: 0x00 */
  __IO uint32_t ISR;           /*!< DMA2D Interrupt Status Register,                Address offset: 0x04 */
  __IO uint32_t IFCR;          /*!< DMA2D Interrupt Flag Clear Register,            Address offset: 0x08 */
  __IO uint32_t FGMAR;         /*!< DMA2D Foreground Memory Address Register,       Address offset: 0x0C */
  __IO uint32_t FGOR;          /*!< DMA2D Foreground Offset Register,               Address offset: 0x10 */
  __IO uint32_t BGMAR;         /*!< DMA2D Background Memory Address Register,       Address offset: 0x14 */
  __IO uint32_t BGOR;          /*!< DMA2D Background Offset Register,               Address offset: 0x18 */
  __IO uint32_t FGPFCCR;       /*!< DMA2D Foreground PFC Control Register,          Address offset: 0x1C */
  __IO uint32_t FGCOLR;        /*!< DMA2D Foreground Color Register,                Address offset: 0x20 */
  __IO uint32_t BGPFCCR;       /*!< DMA2D Background PFC Control Register,          Address offset: 0x24 */
  __IO uint32_t BGCOLR;        /*!< DMA2D Background Color Register,                Address offset: 0x28 */
  __IO uint32_t FGCMAR;        /*!< DMA2D Foreground CLUT Memory Address Register,  Address offset: 0x2C */
  __IO uint32_t BGCMAR;        /*!< DMA2D Background CLUT Memory Address Register,  Address offset: 0x30 */
  __IO uint32_t OPFCCR;        /*!< DMA2D Output PFC Control Register,              Address offset: 0x34 */
  __IO uint32_t OCOLR;         /*!< DMA2D Output Color Register,                    Address offset: 0x38 */
  __IO uint32_t OMAR;          /*!< DMA2D Output Memory Address Register,           Address offset: 0x3C */
  __IO uint32_t OOR;           /*!< DMA2D Output Offset Register,                   Address offset: 0x40 */
  __IO uint32_t NLR;           /*!< DMA2D Number of Line Register,                  Address offset: 0x44 */
  __IO uint32_t LWR;           /*!< DMA2D Line Watermark Register,                  Address offset: 0x48 */
  __IO uint32_t AMTCR;         /*!< DMA2D AHB Master Timer Configuration Register,  Address offset: 0x4C */
  uint32_t      RESERVED[236]; /*!< Reserved, 0x50-0x3FF */
  __IO uint32_t FGCLUT[256];   /*!< DMA2D Foreground CLUT,                          Address offset:400-7FF */
  __IO uint32_t BGCLUT[256];   /*!< DMA2D Background CLUT,                          Address offset:800-BFF */
} DMA2D_TypeDef;

#if defined(STM32F469_479xx)
/**
  * @brief DSI Controller
  */

typedef struct
{
  __IO uint32_t VR;             /*!< DSI Host Version Register,                                 Address offset: 0x00       */
  __IO uint32_t CR;             /*!< DSI Host Control Register,                                 Address offset: 0x04       */
  __IO uint32_t CCR;            /*!< DSI HOST Clock Control Register,                           Address offset: 0x08       */
  __IO uint32_t LVCIDR;         /*!< DSI Host LTDC VCID Register,                               Address offset: 0x0C       */
  __IO uint32_t LCOLCR;         /*!< DSI Host LTDC Color Coding Register,                       Address offset: 0x10       */
  __IO uint32_t LPCR;           /*!< DSI Host LTDC Polarity Configuration Register,             Address offset: 0x14       */
  __IO uint32_t LPMCR;          /*!< DSI Host Low-Power Mode Configuration Register,            Address offset: 0x18       */
  uint32_t      RESERVED0[4];   /*!< Reserved, 0x1C - 0x2B                                                                 */
  __IO uint32_t PCR;            /*!< DSI Host Protocol Configuration Register,                  Address offset: 0x2C       */
  __IO uint32_t GVCIDR;         /*!< DSI Host Generic VCID Register,                            Address offset: 0x30       */
  __IO uint32_t MCR;            /*!< DSI Host Mode Configuration Register,                      Address offset: 0x34       */
  __IO uint32_t VMCR;           /*!< DSI Host Video Mode Configuration Register,                Address offset: 0x38       */
  __IO uint32_t VPCR;           /*!< DSI Host Video Packet Configuration Register,              Address offset: 0x3C       */
  __IO uint32_t VCCR;           /*!< DSI Host Video Chunks Configuration Register,              Address offset: 0x40       */
  __IO uint32_t VNPCR;          /*!< DSI Host Video Null Packet Configuration Register,         Address offset: 0x44       */
  __IO uint32_t VHSACR;         /*!< DSI Host Video HSA Configuration Register,                 Address offset: 0x48       */
  __IO uint32_t VHBPCR;         /*!< DSI Host Video HBP Configuration Register,                 Address offset: 0x4C       */
  __IO uint32_t VLCR;           /*!< DSI Host Video Line Configuration Register,                Address offset: 0x50       */
  __IO uint32_t VVSACR;         /*!< DSI Host Video VSA Configuration Register,                 Address offset: 0x54       */
  __IO uint32_t VVBPCR;         /*!< DSI Host Video VBP Configuration Register,                 Address offset: 0x58       */
  __IO uint32_t VVFPCR;         /*!< DSI Host Video VFP Configuration Register,                 Address offset: 0x5C       */
  __IO uint32_t VVACR;          /*!< DSI Host Video VA Configuration Register,                  Address offset: 0x60       */
  __IO uint32_t LCCR;           /*!< DSI Host LTDC Command Configuration Register,              Address offset: 0x64       */
  __IO uint32_t CMCR;           /*!< DSI Host Command Mode Configuration Register,              Address offset: 0x68       */
  __IO uint32_t GHCR;           /*!< DSI Host Generic Header Configuration Register,            Address offset: 0x6C       */
  __IO uint32_t GPDR;           /*!< DSI Host Generic Payload Data Register,                    Address offset: 0x70       */
  __IO uint32_t GPSR;           /*!< DSI Host Generic Packet Status Register,                   Address offset: 0x74       */
  __IO uint32_t TCCR[6];        /*!< DSI Host Timeout Counter Configuration Register,           Address offset: 0x78-0x8F  */
  __IO uint32_t TDCR;           /*!< DSI Host 3D Configuration Register,                        Address offset: 0x90       */
  __IO uint32_t CLCR;           /*!< DSI Host Clock Lane Configuration Register,                Address offset: 0x94       */
  __IO uint32_t CLTCR;          /*!< DSI Host Clock Lane Timer Configuration Register,          Address offset: 0x98       */
  __IO uint32_t DLTCR;          /*!< DSI Host Data Lane Timer Configuration Register,           Address offset: 0x9C       */
  __IO uint32_t PCTLR;          /*!< DSI Host PHY Control Register,                             Address offset: 0xA0       */
  __IO uint32_t PCONFR;         /*!< DSI Host PHY Configuration Register,                       Address offset: 0xA4       */
  __IO uint32_t PUCR;           /*!< DSI Host PHY ULPS Control Register,                        Address offset: 0xA8       */
  __IO uint32_t PTTCR;          /*!< DSI Host PHY TX Triggers Configuration Register,           Address offset: 0xAC       */
  __IO uint32_t PSR;            /*!< DSI Host PHY Status Register,                              Address offset: 0xB0       */
  uint32_t      RESERVED1[2];   /*!< Reserved, 0xB4 - 0xBB                                                                 */
  __IO uint32_t ISR[2];         /*!< DSI Host Interrupt & Status Register,                      Address offset: 0xBC-0xC3  */
  __IO uint32_t IER[2];         /*!< DSI Host Interrupt Enable Register,                        Address offset: 0xC4-0xCB  */
  uint32_t      RESERVED2[3];   /*!< Reserved, 0xD0 - 0xD7                                                                 */
  __IO uint32_t FIR[2];         /*!< DSI Host Force Interrupt Register,                         Address offset: 0xD8-0xDF  */
  uint32_t      RESERVED3[8];   /*!< Reserved, 0xE0 - 0xFF                                                                 */
  __IO uint32_t VSCR;           /*!< DSI Host Video Shadow Control Register,                    Address offset: 0x100      */
  uint32_t      RESERVED4[2];   /*!< Reserved, 0x104 - 0x10B                                                               */
  __IO uint32_t LCVCIDR;        /*!< DSI Host LTDC Current VCID Register,                       Address offset: 0x10C      */
  __IO uint32_t LCCCR;          /*!< DSI Host LTDC Current Color Coding Register,               Address offset: 0x110      */
  uint32_t      RESERVED5;      /*!< Reserved, 0x114                                                                       */
  __IO uint32_t LPMCCR;         /*!< DSI Host Low-power Mode Current Configuration Register,    Address offset: 0x118      */
  uint32_t      RESERVED6[7];   /*!< Reserved, 0x11C - 0x137                                                               */
  __IO uint32_t VMCCR;          /*!< DSI Host Video Mode Current Configuration Register,        Address offset: 0x138      */
  __IO uint32_t VPCCR;          /*!< DSI Host Video Packet Current Configuration Register,      Address offset: 0x13C      */
  __IO uint32_t VCCCR;          /*!< DSI Host Video Chuncks Current Configuration Register,     Address offset: 0x140      */
  __IO uint32_t VNPCCR;         /*!< DSI Host Video Null Packet Current Configuration Register, Address offset: 0x144      */
  __IO uint32_t VHSACCR;        /*!< DSI Host Video HSA Current Configuration Register,         Address offset: 0x148      */
  __IO uint32_t VHBPCCR;        /*!< DSI Host Video HBP Current Configuration Register,         Address offset: 0x14C      */
  __IO uint32_t VLCCR;          /*!< DSI Host Video Line Current Configuration Register,        Address offset: 0x150      */
  __IO uint32_t VVSACCR;        /*!< DSI Host Video VSA Current Configuration Register,         Address offset: 0x154      */
  __IO uint32_t VVBPCCR;        /*!< DSI Host Video VBP Current Configuration Register,         Address offset: 0x158      */
  __IO uint32_t VVFPCCR;        /*!< DSI Host Video VFP Current Configuration Register,         Address offset: 0x15C      */
  __IO uint32_t VVACCR;         /*!< DSI Host Video VA Current Configuration Register,          Address offset: 0x160      */
  uint32_t      RESERVED7[11];  /*!< Reserved, 0x164 - 0x18F                                                               */
  __IO uint32_t TDCCR;          /*!< DSI Host 3D Current Configuration Register,                Address offset: 0x190      */
  uint32_t      RESERVED8[155]; /*!< Reserved, 0x194 - 0x3FF                                                               */
  __IO uint32_t WCFGR;          /*!< DSI Wrapper Configuration Register,                       Address offset: 0x400       */
  __IO uint32_t WCR;            /*!< DSI Wrapper Control Register,                             Address offset: 0x404       */
  __IO uint32_t WIER;           /*!< DSI Wrapper Interrupt Enable Register,                    Address offset: 0x408       */
  __IO uint32_t WISR;           /*!< DSI Wrapper Interrupt and Status Register,                Address offset: 0x40C       */
  __IO uint32_t WIFCR;          /*!< DSI Wrapper Interrupt Flag Clear Register,                Address offset: 0x410       */
  uint32_t      RESERVED9;      /*!< Reserved, 0x414                                                                       */
  __IO uint32_t WPCR[5];        /*!< DSI Wrapper PHY Configuration Register,                   Address offset: 0x418-0x42B */
  uint32_t      RESERVED10;     /*!< Reserved, 0x42C                                                                       */
  __IO uint32_t WRPCR;          /*!< DSI Wrapper Regulator and PLL Control Register, Address offset: 0x430                 */
} DSI_TypeDef;
#endif /* STM32F469_479xx */

/**
  * @brief Ethernet MAC
  */

typedef struct
{
  __IO uint32_t MACCR;
  __IO uint32_t MACFFR;
  __IO uint32_t MACHTHR;
  __IO uint32_t MACHTLR;
  __IO uint32_t MACMIIAR;
  __IO uint32_t MACMIIDR;
  __IO uint32_t MACFCR;
  __IO uint32_t MACVLANTR;             /*    8 */
  uint32_t      RESERVED0[2];
  __IO uint32_t MACRWUFFR;             /*   11 */
  __IO uint32_t MACPMTCSR;
  uint32_t      RESERVED1[2];
  __IO uint32_t MACSR;                 /*   15 */
  __IO uint32_t MACIMR;
  __IO uint32_t MACA0HR;
  __IO uint32_t MACA0LR;
  __IO uint32_t MACA1HR;
  __IO uint32_t MACA1LR;
  __IO uint32_t MACA2HR;
  __IO uint32_t MACA2LR;
  __IO uint32_t MACA3HR;
  __IO uint32_t MACA3LR;               /*   24 */
  uint32_t      RESERVED2[40];
  __IO uint32_t MMCCR;                 /*   65 */
  __IO uint32_t MMCRIR;
  __IO uint32_t MMCTIR;
  __IO uint32_t MMCRIMR;
  __IO uint32_t MMCTIMR;               /*   69 */
  uint32_t      RESERVED3[14];
  __IO uint32_t MMCTGFSCCR;            /*   84 */
  __IO uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  __IO uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  __IO uint32_t MMCRFCECR;
  __IO uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  __IO uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  __IO uint32_t PTPTSCR;
  __IO uint32_t PTPSSIR;
  __IO uint32_t PTPTSHR;
  __IO uint32_t PTPTSLR;
  __IO uint32_t PTPTSHUR;
  __IO uint32_t PTPTSLUR;
  __IO uint32_t PTPTSAR;
  __IO uint32_t PTPTTHR;
  __IO uint32_t PTPTTLR;
  __IO uint32_t RESERVED8;
  __IO uint32_t PTPTSSR;
  uint32_t      RESERVED9[565];
  __IO uint32_t DMABMR;
  __IO uint32_t DMATPDR;
  __IO uint32_t DMARPDR;
  __IO uint32_t DMARDLAR;
  __IO uint32_t DMATDLAR;
  __IO uint32_t DMASR;
  __IO uint32_t DMAOMR;
  __IO uint32_t DMAIER;
  __IO uint32_t DMAMFBOCR;
  __IO uint32_t DMARSWTR;
  uint32_t      RESERVED10[8];
  __IO uint32_t DMACHTDR;
  __IO uint32_t DMACHRDR;
  __IO uint32_t DMACHTBAR;
  __IO uint32_t DMACHRBAR;
} ETH_TypeDef;

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

#if defined(STM32F40_41xxx) || defined(STM32F412xG) || defined(STM32F413_423xx)
/**
  * @brief Flexible Static Memory Controller
  */

typedef struct
{
  __IO uint32_t BTCR[8];    /*!< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C */
} FSMC_Bank1_TypeDef;

/**
  * @brief Flexible Static Memory Controller Bank1E
  */

typedef struct
{
  __IO uint32_t BWTR[7];    /*!< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C */
} FSMC_Bank1E_TypeDef;

/**
  * @brief Flexible Static Memory Controller Bank2
  */

typedef struct
{
  __IO uint32_t PCR2;       /*!< NAND Flash control register 2,                       Address offset: 0x60 */
  __IO uint32_t SR2;        /*!< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64 */
  __IO uint32_t PMEM2;      /*!< NAND Flash Common memory space timing register 2,    Address offset: 0x68 */
  __IO uint32_t PATT2;      /*!< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C */
  uint32_t      RESERVED0;  /*!< Reserved, 0x70                                                            */
  __IO uint32_t ECCR2;      /*!< NAND Flash ECC result registers 2,                   Address offset: 0x74 */
} FSMC_Bank2_TypeDef;

/**
  * @brief Flexible Static Memory Controller Bank3
  */

typedef struct
{
  __IO uint32_t PCR3;       /*!< NAND Flash control register 3,                       Address offset: 0x80 */
  __IO uint32_t SR3;        /*!< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84 */
  __IO uint32_t PMEM3;      /*!< NAND Flash Common memory space timing register 3,    Address offset: 0x88 */
  __IO uint32_t PATT3;      /*!< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C */
  uint32_t      RESERVED0;  /*!< Reserved, 0x90                                                            */
  __IO uint32_t ECCR3;      /*!< NAND Flash ECC result registers 3,                   Address offset: 0x94 */
} FSMC_Bank3_TypeDef;

/**
  * @brief Flexible Static Memory Controller Bank4
  */

typedef struct
{
  __IO uint32_t PCR4;       /*!< PC Card  control register 4,                       Address offset: 0xA0 */
  __IO uint32_t SR4;        /*!< PC Card  FIFO status and interrupt register 4,     Address offset: 0xA4 */
  __IO uint32_t PMEM4;      /*!< PC Card  Common memory space timing register 4,    Address offset: 0xA8 */
  __IO uint32_t PATT4;      /*!< PC Card  Attribute memory space timing register 4, Address offset: 0xAC */
  __IO uint32_t PIO4;       /*!< PC Card  I/O space timing register 4,              Address offset: 0xB0 */
} FSMC_Bank4_TypeDef;
#endif /* STM32F40_41xxx || STM32F412xG || STM32F413_423xx */

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
/**
  * @brief Flexible Memory Controller
  */

typedef struct
{
  __IO uint32_t BTCR[8];    /*!< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C */
} FMC_Bank1_TypeDef;

/**
  * @brief Flexible Memory Controller Bank1E
  */

typedef struct
{
  __IO uint32_t BWTR[7];    /*!< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C */
} FMC_Bank1E_TypeDef;

/**
  * @brief Flexible Memory Controller Bank2
  */

typedef struct
{
  __IO uint32_t PCR2;       /*!< NAND Flash control register 2,                       Address offset: 0x60 */
  __IO uint32_t SR2;        /*!< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64 */
  __IO uint32_t PMEM2;      /*!< NAND Flash Common memory space timing register 2,    Address offset: 0x68 */
  __IO uint32_t PATT2;      /*!< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C */
  uint32_t      RESERVED0;  /*!< Reserved, 0x70                                                            */
  __IO uint32_t ECCR2;      /*!< NAND Flash ECC result registers 2,                   Address offset: 0x74 */
} FMC_Bank2_TypeDef;

/**
  * @brief Flexible Memory Controller Bank3
  */

typedef struct
{
  __IO uint32_t PCR3;       /*!< NAND Flash control register 3,                       Address offset: 0x80 */
  __IO uint32_t SR3;        /*!< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84 */
  __IO uint32_t PMEM3;      /*!< NAND Flash Common memory space timing register 3,    Address offset: 0x88 */
  __IO uint32_t PATT3;      /*!< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C */
  uint32_t      RESERVED0;  /*!< Reserved, 0x90                                                            */
  __IO uint32_t ECCR3;      /*!< NAND Flash ECC result registers 3,                   Address offset: 0x94 */
} FMC_Bank3_TypeDef;

/**
  * @brief Flexible Memory Controller Bank4
  */

typedef struct
{
  __IO uint32_t PCR4;       /*!< PC Card  control register 4,                       Address offset: 0xA0 */
  __IO uint32_t SR4;        /*!< PC Card  FIFO status and interrupt register 4,     Address offset: 0xA4 */
  __IO uint32_t PMEM4;      /*!< PC Card  Common memory space timing register 4,    Address offset: 0xA8 */
  __IO uint32_t PATT4;      /*!< PC Card  Attribute memory space timing register 4, Address offset: 0xAC */
  __IO uint32_t PIO4;       /*!< PC Card  I/O space timing register 4,              Address offset: 0xB0 */
} FMC_Bank4_TypeDef;

/**
  * @brief Flexible Memory Controller Bank5_6
  */

typedef struct
{
  __IO uint32_t SDCR[2];        /*!< SDRAM Control registers ,      Address offset: 0x140-0x144  */
  __IO uint32_t SDTR[2];        /*!< SDRAM Timing registers ,       Address offset: 0x148-0x14C  */
  __IO uint32_t SDCMR;       /*!< SDRAM Command Mode register,    Address offset: 0x150  */
  __IO uint32_t SDRTR;       /*!< SDRAM Refresh Timer register,   Address offset: 0x154  */
  __IO uint32_t SDSR;        /*!< SDRAM Status register,          Address offset: 0x158  */
} FMC_Bank5_6_TypeDef;
#endif /* STM32F427_437xx ||  STM32F429_439xx || STM32F446xx || STM32F469_479xx */

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
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
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
#if defined (STM32F410xx) || defined(STM32F412xG) || defined(STM32F413_423xx)
  uint32_t      RESERVED;     /*!< Reserved, 0x18                                                               */
  __IO uint32_t CFGR2;        /*!< Reserved, 0x1C                                                               */
  __IO uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
  uint32_t      RESERVED1[2]; /*!< Reserved, 0x24-0x28                                                          */
  __IO uint32_t CFGR;         /*!< SYSCFG Configuration register,                     Address offset: 0x2C      */
#else  /* STM32F40_41xxx || STM32F427_437xx || STM32F429_439xx || STM32F401xx || STM32F411xE || STM32F446xx || STM32F469_479xx */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  __IO uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
#endif /* STM32F410xx || defined(STM32F412xG) || defined(STM32F413_423xx) */
#if defined(STM32F413_423xx)
  __IO uint32_t MCHDLYCR;     /*!< SYSCFG multi-channel delay register,               Address offset: 0x30      */
#endif /* STM32F413_423xx */
} SYSCFG_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint16_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  uint16_t      RESERVED0;  /*!< Reserved, 0x02                                   */
  __IO uint16_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  uint16_t      RESERVED1;  /*!< Reserved, 0x06                                   */
  __IO uint16_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  uint16_t      RESERVED2;  /*!< Reserved, 0x0A                                   */
  __IO uint16_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  uint16_t      RESERVED3;  /*!< Reserved, 0x0E                                   */
  __IO uint16_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
  uint16_t      RESERVED4;  /*!< Reserved, 0x12                                   */
  __IO uint16_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  uint16_t      RESERVED5;  /*!< Reserved, 0x16                                   */
  __IO uint16_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  uint16_t      RESERVED6;  /*!< Reserved, 0x1A                                   */
  __IO uint16_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  uint16_t      RESERVED7;  /*!< Reserved, 0x1E                                   */
  __IO uint16_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
  uint16_t      RESERVED8;  /*!< Reserved, 0x22                                   */
  __IO uint16_t FLTR;       /*!< I2C FLTR register,          Address offset: 0x24 */
  uint16_t      RESERVED9;  /*!< Reserved, 0x26                                   */
} I2C_TypeDef;

#if defined(STM32F410xx) || defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
/**
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;      /*!< FMPI2C Control register 1,            Address offset: 0x00 */
  __IO uint32_t CR2;      /*!< FMPI2C Control register 2,            Address offset: 0x04 */
  __IO uint32_t OAR1;     /*!< FMPI2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;     /*!< FMPI2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;  /*!< FMPI2C Timing register,               Address offset: 0x10 */
  __IO uint32_t TIMEOUTR; /*!< FMPI2C Timeout register,              Address offset: 0x14 */
  __IO uint32_t ISR;      /*!< FMPI2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;      /*!< FMPI2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t PECR;     /*!< FMPI2C PEC register,                  Address offset: 0x20 */
  __IO uint32_t RXDR;     /*!< FMPI2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;     /*!< FMPI2C Transmit data register,        Address offset: 0x28 */
}FMPI2C_TypeDef;
#endif /* STM32F410xx || STM32F412xG || STM32F413_423xx || STM32F446xx */

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
  * @brief LCD-TFT Display Controller
  */

typedef struct
{
  uint32_t      RESERVED0[2];  /*!< Reserved, 0x00-0x04 */
  __IO uint32_t SSCR;          /*!< LTDC Synchronization Size Configuration Register,    Address offset: 0x08 */
  __IO uint32_t BPCR;          /*!< LTDC Back Porch Configuration Register,              Address offset: 0x0C */
  __IO uint32_t AWCR;          /*!< LTDC Active Width Configuration Register,            Address offset: 0x10 */
  __IO uint32_t TWCR;          /*!< LTDC Total Width Configuration Register,             Address offset: 0x14 */
  __IO uint32_t GCR;           /*!< LTDC Global Control Register,                        Address offset: 0x18 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x1C-0x20 */
  __IO uint32_t SRCR;          /*!< LTDC Shadow Reload Configuration Register,           Address offset: 0x24 */
  uint32_t      RESERVED2[1];  /*!< Reserved, 0x28 */
  __IO uint32_t BCCR;          /*!< LTDC Background Color Configuration Register,        Address offset: 0x2C */
  uint32_t      RESERVED3[1];  /*!< Reserved, 0x30 */
  __IO uint32_t IER;           /*!< LTDC Interrupt Enable Register,                      Address offset: 0x34 */
  __IO uint32_t ISR;           /*!< LTDC Interrupt Status Register,                      Address offset: 0x38 */
  __IO uint32_t ICR;           /*!< LTDC Interrupt Clear Register,                       Address offset: 0x3C */
  __IO uint32_t LIPCR;         /*!< LTDC Line Interrupt Position Configuration Register, Address offset: 0x40 */
  __IO uint32_t CPSR;          /*!< LTDC Current Position Status Register,               Address offset: 0x44 */
  __IO uint32_t CDSR;         /*!< LTDC Current Display Status Register,                       Address offset: 0x48 */
} LTDC_TypeDef;

/**
  * @brief LCD-TFT Display layer x Controller
  */

typedef struct
{
  __IO uint32_t CR;            /*!< LTDC Layerx Control Register                                  Address offset: 0x84 */
  __IO uint32_t WHPCR;         /*!< LTDC Layerx Window Horizontal Position Configuration Register Address offset: 0x88 */
  __IO uint32_t WVPCR;         /*!< LTDC Layerx Window Vertical Position Configuration Register   Address offset: 0x8C */
  __IO uint32_t CKCR;          /*!< LTDC Layerx Color Keying Configuration Register               Address offset: 0x90 */
  __IO uint32_t PFCR;          /*!< LTDC Layerx Pixel Format Configuration Register               Address offset: 0x94 */
  __IO uint32_t CACR;          /*!< LTDC Layerx Constant Alpha Configuration Register             Address offset: 0x98 */
  __IO uint32_t DCCR;          /*!< LTDC Layerx Default Color Configuration Register              Address offset: 0x9C */
  __IO uint32_t BFCR;          /*!< LTDC Layerx Blending Factors Configuration Register           Address offset: 0xA0 */
  uint32_t      RESERVED0[2];  /*!< Reserved */
  __IO uint32_t CFBAR;         /*!< LTDC Layerx Color Frame Buffer Address Register               Address offset: 0xAC */
  __IO uint32_t CFBLR;         /*!< LTDC Layerx Color Frame Buffer Length Register                Address offset: 0xB0 */
  __IO uint32_t CFBLNR;        /*!< LTDC Layerx ColorFrame Buffer Line Number Register            Address offset: 0xB4 */
  uint32_t      RESERVED1[3];  /*!< Reserved */
  __IO uint32_t CLUTWR;         /*!< LTDC Layerx CLUT Write Register                               Address offset: 0x144 */

} LTDC_Layer_TypeDef;

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
  __IO uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
  __IO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
  __IO uint32_t CKGATENR;      /*!< RCC Clocks Gated Enable Register,                            Address offset: 0x90 */ /* Only for STM32F412xG, STM32413_423xx and STM32F446xx devices */
  __IO uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */ /* Only for STM32F410xx, STM32F412xG, STM32413_423xx and STM32F446xx devices */

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
  * @brief Serial Audio Interface
  */

typedef struct
{
  __IO uint32_t GCR;      /*!< SAI global configuration register,        Address offset: 0x00 */
} SAI_TypeDef;

typedef struct
{
  __IO uint32_t CR1;      /*!< SAI block x configuration register 1,     Address offset: 0x04 */
  __IO uint32_t CR2;      /*!< SAI block x configuration register 2,     Address offset: 0x08 */
  __IO uint32_t FRCR;     /*!< SAI block x frame configuration register, Address offset: 0x0C */
  __IO uint32_t SLOTR;    /*!< SAI block x slot register,                Address offset: 0x10 */
  __IO uint32_t IMR;      /*!< SAI block x interrupt mask register,      Address offset: 0x14 */
  __IO uint32_t SR;       /*!< SAI block x status register,              Address offset: 0x18 */
  __IO uint32_t CLRFR;    /*!< SAI block x clear flag register,          Address offset: 0x1C */
  __IO uint32_t DR;       /*!< SAI block x data register,                Address offset: 0x20 */
} SAI_Block_TypeDef;

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
  __IO uint16_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  uint16_t      RESERVED0;  /*!< Reserved, 0x02                                                           */
  __IO uint16_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  uint16_t      RESERVED1;  /*!< Reserved, 0x06                                                           */
  __IO uint16_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  uint16_t      RESERVED2;  /*!< Reserved, 0x0A                                                           */
  __IO uint16_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  uint16_t      RESERVED3;  /*!< Reserved, 0x0E                                                           */
  __IO uint16_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  uint16_t      RESERVED4;  /*!< Reserved, 0x12                                                           */
  __IO uint16_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  uint16_t      RESERVED5;  /*!< Reserved, 0x16                                                           */
  __IO uint16_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  uint16_t      RESERVED6;  /*!< Reserved, 0x1A                                                           */
  __IO uint16_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  uint16_t      RESERVED7;  /*!< Reserved, 0x1E                                                           */
  __IO uint16_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
  uint16_t      RESERVED8;  /*!< Reserved, 0x22                                                           */
} SPI_TypeDef;

#if defined(STM32F446xx)
/**
  * @brief SPDIFRX Interface
  */
typedef struct
{
  __IO uint32_t   CR;           /*!< Control register,                   Address offset: 0x00 */
  __IO uint16_t   IMR;          /*!< Interrupt mask register,            Address offset: 0x04 */
  uint16_t        RESERVED0;    /*!< Reserved,  0x06                                          */
  __IO uint32_t   SR;           /*!< Status register,                    Address offset: 0x08 */
  __IO uint16_t   IFCR;         /*!< Interrupt Flag Clear register,      Address offset: 0x0C */
  uint16_t        RESERVED1;    /*!< Reserved,  0x0E                                          */
  __IO uint32_t   DR;           /*!< Data input register,                Address offset: 0x10 */
  __IO uint32_t   CSR;          /*!< Channel Status register,            Address offset: 0x14 */
   __IO uint32_t  DIR;          /*!< Debug Information register,         Address offset: 0x18 */
  uint16_t        RESERVED2;    /*!< Reserved,  0x1A                                          */
} SPDIFRX_TypeDef;
#endif /* STM32F446xx */

#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
/**
  * @brief QUAD Serial Peripheral Interface
  */
typedef struct
{
  __IO uint32_t CR;       /*!< QUADSPI Control register,                           Address offset: 0x00 */
  __IO uint32_t DCR;      /*!< QUADSPI Device Configuration register,              Address offset: 0x04 */
  __IO uint32_t SR;       /*!< QUADSPI Status register,                            Address offset: 0x08 */
  __IO uint32_t FCR;      /*!< QUADSPI Flag Clear register,                        Address offset: 0x0C */
  __IO uint32_t DLR;      /*!< QUADSPI Data Length register,                       Address offset: 0x10 */
  __IO uint32_t CCR;      /*!< QUADSPI Communication Configuration register,       Address offset: 0x14 */
  __IO uint32_t AR;       /*!< QUADSPI Address register,                           Address offset: 0x18 */
  __IO uint32_t ABR;      /*!< QUADSPI Alternate Bytes register,                   Address offset: 0x1C */
  __IO uint32_t DR;       /*!< QUADSPI Data register,                              Address offset: 0x20 */
  __IO uint32_t PSMKR;    /*!< QUADSPI Polling Status Mask register,               Address offset: 0x24 */
  __IO uint32_t PSMAR;    /*!< QUADSPI Polling Status Match register,              Address offset: 0x28 */
  __IO uint32_t PIR;      /*!< QUADSPI Polling Interval register,                  Address offset: 0x2C */
  __IO uint32_t LPTR;     /*!< QUADSPI Low Power Timeout register,                 Address offset: 0x30 */
} QUADSPI_TypeDef;
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx || STM32F469_479xx */

#if defined(STM32F446xx)
/**
  * @brief SPDIF-RX Interface
  */
typedef struct
{
  __IO uint32_t   CR;           /*!< Control register,                   Address offset: 0x00 */
  __IO uint16_t   IMR;          /*!< Interrupt mask register,            Address offset: 0x04 */
  uint16_t        RESERVED0;    /*!< Reserved,  0x06                                          */
  __IO uint32_t   SR;           /*!< Status register,                    Address offset: 0x08 */
  __IO uint16_t   IFCR;         /*!< Interrupt Flag Clear register,      Address offset: 0x0C */
  uint16_t        RESERVED1;    /*!< Reserved,  0x0E                                          */
  __IO uint32_t   DR;           /*!< Data input register,                Address offset: 0x10 */
  __IO uint32_t   CSR;          /*!< Channel Status register,            Address offset: 0x14 */
   __IO uint32_t  DIR;          /*!< Debug Information register,         Address offset: 0x18 */
  uint16_t        RESERVED2;    /*!< Reserved,  0x1A                                          */
} SPDIF_TypeDef;
#endif /* STM32F446xx */

/**
  * @brief TIM
  */

typedef struct
{
  __IO uint16_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  uint16_t      RESERVED0;   /*!< Reserved, 0x02                                            */
  __IO uint16_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  uint16_t      RESERVED1;   /*!< Reserved, 0x06                                            */
  __IO uint16_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  uint16_t      RESERVED2;   /*!< Reserved, 0x0A                                            */
  __IO uint16_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  uint16_t      RESERVED3;   /*!< Reserved, 0x0E                                            */
  __IO uint16_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  uint16_t      RESERVED4;   /*!< Reserved, 0x12                                            */
  __IO uint16_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  uint16_t      RESERVED5;   /*!< Reserved, 0x16                                            */
  __IO uint16_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  uint16_t      RESERVED6;   /*!< Reserved, 0x1A                                            */
  __IO uint16_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  uint16_t      RESERVED7;   /*!< Reserved, 0x1E                                            */
  __IO uint16_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  uint16_t      RESERVED8;   /*!< Reserved, 0x22                                            */
  __IO uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint16_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  uint16_t      RESERVED9;   /*!< Reserved, 0x2A                                            */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint16_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  uint16_t      RESERVED10;  /*!< Reserved, 0x32                                            */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint16_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  uint16_t      RESERVED11;  /*!< Reserved, 0x46                                            */
  __IO uint16_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  uint16_t      RESERVED12;  /*!< Reserved, 0x4A                                            */
  __IO uint16_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  uint16_t      RESERVED13;  /*!< Reserved, 0x4E                                            */
  __IO uint16_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
  uint16_t      RESERVED14;  /*!< Reserved, 0x52                                            */
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
  * @brief Crypto Processor
  */

typedef struct
{
  __IO uint32_t CR;         /*!< CRYP control register,                                    Address offset: 0x00 */
  __IO uint32_t SR;         /*!< CRYP status register,                                     Address offset: 0x04 */
  __IO uint32_t DR;         /*!< CRYP data input register,                                 Address offset: 0x08 */
  __IO uint32_t DOUT;       /*!< CRYP data output register,                                Address offset: 0x0C */
  __IO uint32_t DMACR;      /*!< CRYP DMA control register,                                Address offset: 0x10 */
  __IO uint32_t IMSCR;      /*!< CRYP interrupt mask set/clear register,                   Address offset: 0x14 */
  __IO uint32_t RISR;       /*!< CRYP raw interrupt status register,                       Address offset: 0x18 */
  __IO uint32_t MISR;       /*!< CRYP masked interrupt status register,                    Address offset: 0x1C */
  __IO uint32_t K0LR;       /*!< CRYP key left  register 0,                                Address offset: 0x20 */
  __IO uint32_t K0RR;       /*!< CRYP key right register 0,                                Address offset: 0x24 */
  __IO uint32_t K1LR;       /*!< CRYP key left  register 1,                                Address offset: 0x28 */
  __IO uint32_t K1RR;       /*!< CRYP key right register 1,                                Address offset: 0x2C */
  __IO uint32_t K2LR;       /*!< CRYP key left  register 2,                                Address offset: 0x30 */
  __IO uint32_t K2RR;       /*!< CRYP key right register 2,                                Address offset: 0x34 */
  __IO uint32_t K3LR;       /*!< CRYP key left  register 3,                                Address offset: 0x38 */
  __IO uint32_t K3RR;       /*!< CRYP key right register 3,                                Address offset: 0x3C */
  __IO uint32_t IV0LR;      /*!< CRYP initialization vector left-word  register 0,         Address offset: 0x40 */
  __IO uint32_t IV0RR;      /*!< CRYP initialization vector right-word register 0,         Address offset: 0x44 */
  __IO uint32_t IV1LR;      /*!< CRYP initialization vector left-word  register 1,         Address offset: 0x48 */
  __IO uint32_t IV1RR;      /*!< CRYP initialization vector right-word register 1,         Address offset: 0x4C */
  __IO uint32_t CSGCMCCM0R; /*!< CRYP GCM/GMAC or CCM/CMAC context swap register 0,        Address offset: 0x50 */
  __IO uint32_t CSGCMCCM1R; /*!< CRYP GCM/GMAC or CCM/CMAC context swap register 1,        Address offset: 0x54 */
  __IO uint32_t CSGCMCCM2R; /*!< CRYP GCM/GMAC or CCM/CMAC context swap register 2,        Address offset: 0x58 */
  __IO uint32_t CSGCMCCM3R; /*!< CRYP GCM/GMAC or CCM/CMAC context swap register 3,        Address offset: 0x5C */
  __IO uint32_t CSGCMCCM4R; /*!< CRYP GCM/GMAC or CCM/CMAC context swap register 4,        Address offset: 0x60 */
  __IO uint32_t CSGCMCCM5R; /*!< CRYP GCM/GMAC or CCM/CMAC context swap register 5,        Address offset: 0x64 */
  __IO uint32_t CSGCMCCM6R; /*!< CRYP GCM/GMAC or CCM/CMAC context swap register 6,        Address offset: 0x68 */
  __IO uint32_t CSGCMCCM7R; /*!< CRYP GCM/GMAC or CCM/CMAC context swap register 7,        Address offset: 0x6C */
  __IO uint32_t CSGCM0R;    /*!< CRYP GCM/GMAC context swap register 0,                    Address offset: 0x70 */
  __IO uint32_t CSGCM1R;    /*!< CRYP GCM/GMAC context swap register 1,                    Address offset: 0x74 */
  __IO uint32_t CSGCM2R;    /*!< CRYP GCM/GMAC context swap register 2,                    Address offset: 0x78 */
  __IO uint32_t CSGCM3R;    /*!< CRYP GCM/GMAC context swap register 3,                    Address offset: 0x7C */
  __IO uint32_t CSGCM4R;    /*!< CRYP GCM/GMAC context swap register 4,                    Address offset: 0x80 */
  __IO uint32_t CSGCM5R;    /*!< CRYP GCM/GMAC context swap register 5,                    Address offset: 0x84 */
  __IO uint32_t CSGCM6R;    /*!< CRYP GCM/GMAC context swap register 6,                    Address offset: 0x88 */
  __IO uint32_t CSGCM7R;    /*!< CRYP GCM/GMAC context swap register 7,                    Address offset: 0x8C */
} CRYP_TypeDef;

/**
  * @brief HASH
  */

typedef struct
{
  __IO uint32_t CR;               /*!< HASH control register,          Address offset: 0x00        */
  __IO uint32_t DIN;              /*!< HASH data input register,       Address offset: 0x04        */
  __IO uint32_t STR;              /*!< HASH start register,            Address offset: 0x08        */
  __IO uint32_t HR[5];            /*!< HASH digest registers,          Address offset: 0x0C-0x1C   */
  __IO uint32_t IMR;              /*!< HASH interrupt enable register, Address offset: 0x20        */
  __IO uint32_t SR;               /*!< HASH status register,           Address offset: 0x24        */
       uint32_t RESERVED[52];     /*!< Reserved, 0x28-0xF4                                         */
  __IO uint32_t CSR[54];          /*!< HASH context swap registers,    Address offset: 0x0F8-0x1CC */
} HASH_TypeDef;

/**
  * @brief HASH_DIGEST
  */

typedef struct
{
  __IO uint32_t HR[8];     /*!< HASH digest registers,          Address offset: 0x310-0x32C */
} HASH_DIGEST_TypeDef;

/**
  * @brief RNG
  */

typedef struct
{
  __IO uint32_t CR;  /*!< RNG control register, Address offset: 0x00 */
  __IO uint32_t SR;  /*!< RNG status register,  Address offset: 0x04 */
  __IO uint32_t DR;  /*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;

#if defined(STM32F410xx) || defined(STM32F413_423xx)
/**
  * @brief LPTIMER
  */
typedef struct
{
  __IO uint32_t ISR;         /*!< LPTIM Interrupt and Status register,                Address offset: 0x00 */
  __IO uint32_t ICR;         /*!< LPTIM Interrupt Clear register,                     Address offset: 0x04 */
  __IO uint32_t IER;         /*!< LPTIM Interrupt Enable register,                    Address offset: 0x08 */
  __IO uint32_t CFGR;        /*!< LPTIM Configuration register,                       Address offset: 0x0C */
  __IO uint32_t CR;          /*!< LPTIM Control register,                             Address offset: 0x10 */
  __IO uint32_t CMP;         /*!< LPTIM Compare register,                             Address offset: 0x14 */
  __IO uint32_t ARR;         /*!< LPTIM Autoreload register,                          Address offset: 0x18 */
  __IO uint32_t CNT;         /*!< LPTIM Counter register,                             Address offset: 0x1C */
  __IO uint32_t OR;          /*!< LPTIM Option register,                              Address offset: 0x20 */
} LPTIM_TypeDef;
#endif /* STM32F410xx || STM32F413_423xx */
/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            (0x08000000UL) /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define CCMDATARAM_BASE       (0x10000000UL) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define SRAM1_BASE            (0x20000000UL) /*!< SRAM1(112 KB) base address in the alias region                             */
#if defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx)
#define SRAM2_BASE            (0x2001C000UL) /*!< SRAM2(16 KB) base address in the alias region                              */
#define SRAM3_BASE            (0x20020000UL) /*!< SRAM3(64 KB) base address in the alias region                              */
#elif defined(STM32F469_479xx)
#define SRAM2_BASE            (0x20028000UL) /*!< SRAM2(16 KB) base address in the alias region                              */
#define SRAM3_BASE            (0x20030000UL) /*!< SRAM3(64 KB) base address in the alias region                              */
#elif defined(STM32F413_423xx)
#define SRAM2_BASE            (0x20040000UL) /*!< SRAM2(16 KB) base address in the alias region                              */
#else /* STM32F411xE || STM32F410xx || STM32F412xG */
#endif /* STM32F40_41xxx || STM32F427_437xx || STM32F429_439xx ||  STM32F446xx */
#define PERIPH_BASE           (0x40000000UL) /*!< Peripheral base address in the alias region                                */
#define BKPSRAM_BASE          (0x40024000UL) /*!< Backup SRAM(4 KB) base address in the alias region                         */

#if defined(STM32F40_41xxx) || defined(STM32F412xG) || defined(STM32F413_423xx)
#define FSMC_R_BASE           (0xA0000000UL) /*!< FSMC registers base address                                                */
#endif /* STM32F40_41xxx || STM32F412xG || STM32F413_423xx */

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define FMC_R_BASE            (0xA0000000UL) /*!< FMC registers base address                                                 */
#endif /* STM32F427_437xx ||  STM32F429_439xx || STM32F446xx || STM32F469_479xx */

#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define QSPI_R_BASE           (0xA0001000UL) /*!< QuadSPI registers base address                                            */
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx || STM32F469_479xx */

#define CCMDATARAM_BB_BASE    (0x12000000UL) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the bit-band region  */
#define SRAM1_BB_BASE         (0x22000000UL) /*!< SRAM1(112 KB) base address in the bit-band region                             */
#if defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx)
#define SRAM2_BB_BASE         (0x22380000UL) /*!< SRAM2(16 KB) base address in the bit-band region                              */
#define SRAM3_BB_BASE         (0x22400000UL) /*!< SRAM3(64 KB) base address in the bit-band region                              */
#elif defined(STM32F469_479xx)
#define SRAM2_BB_BASE         (0x22500000UL) /*!< SRAM2(16 KB) base address in the bit-band region                              */
#define SRAM3_BB_BASE         (0x22600000UL) /*!< SRAM3(64 KB) base address in the bit-band region                              */
#elif defined(STM32F413_423xx)
#define SRAM2_BB_BASE         (0x22800000UL) /*!< SRAM2(64 KB) base address in the bit-band region                              */
#else /* STM32F411xE || STM32F410xx || STM32F412xG */
#endif /* STM32F40_41xxx || STM32F427_437xx || STM32F429_439xx ||  STM32F446xx */
#define PERIPH_BB_BASE        (0x42000000UL) /*!< Peripheral base address in the bit-band region                                */
#define BKPSRAM_BB_BASE       (0x42480000UL) /*!< Backup SRAM(4 KB) base address in the bit-band region                         */

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
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#if defined(STM32F410xx) || defined(STM32F413_423xx)
#define LPTIM1_BASE           (APB1PERIPH_BASE + 0x2400)
#endif /* STM32F410xx || STM32F413_423xx */
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#if defined(STM32F446xx)
#define SPDIFRX_BASE          (APB1PERIPH_BASE + 0x4000)
#endif /* STM32F446xx */
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00)
#if defined(STM32F410xx) || defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
#define FMPI2C1_BASE          (APB1PERIPH_BASE + 0x6000)
#endif /* STM32F410xx || STM32F412xG || STM32F413_423xx || STM32F446xx */
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400)
#define CAN2_BASE             (APB1PERIPH_BASE + 0x6800)
#if defined(STM32F413_423xx)
#define CAN3_BASE             (APB1PERIPH_BASE + 0x6C00)
#endif /* STM32F413_423xx */
#if defined(STM32F446xx)
#define CEC_BASE              (APB1PERIPH_BASE + 0x6C00)
#endif /* STM32F446xx */
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400)
#define UART7_BASE            (APB1PERIPH_BASE + 0x7800)
#define UART8_BASE            (APB1PERIPH_BASE + 0x7C00)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400)
#define UART9_BASE            (APB2PERIPH_BASE + 0x1800U)
#define UART10_BASE           (APB2PERIPH_BASE + 0x1C00U)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2100)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x2200)
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
#define SPI6_BASE             (APB2PERIPH_BASE + 0x5400)
#define SAI1_BASE             (APB2PERIPH_BASE + 0x5800)
#define SAI1_Block_A_BASE     (SAI1_BASE + 0x004)
#define SAI1_Block_B_BASE     (SAI1_BASE + 0x024)
#if defined(STM32F446xx)
#define SAI2_BASE             (APB2PERIPH_BASE + 0x5C00)
#define SAI2_Block_A_BASE     (SAI2_BASE + 0x004)
#define SAI2_Block_B_BASE     (SAI2_BASE + 0x024)
#endif /* STM32F446xx */
#define LTDC_BASE             (APB2PERIPH_BASE + 0x6800)
#define LTDC_Layer1_BASE      (LTDC_BASE + 0x84)
#define LTDC_Layer2_BASE      (LTDC_BASE + 0x104)
#if defined(STM32F469_479xx)
#define DSI_BASE              (APB2PERIPH_BASE + 0x6C00)
#endif /* STM32F469_479xx */
#if defined(STM32F412xG) || defined(STM32F413_423xx)
#define DFSDM1_BASE           (APB2PERIPH_BASE + 0x6000)
#define DFSDM1_Channel0_BASE  (DFSDM1_BASE + 0x00)
#define DFSDM1_Channel1_BASE  (DFSDM1_BASE + 0x20)
#define DFSDM1_Channel2_BASE  (DFSDM1_BASE + 0x40)
#define DFSDM1_Channel3_BASE  (DFSDM1_BASE + 0x60)
#define DFSDM1_Filter0_BASE   (DFSDM1_BASE + 0x100)
#define DFSDM1_Filter1_BASE   (DFSDM1_BASE + 0x180)
#define DFSDM1_0              ((DFSDM_TypeDef *) DFSDM1_Filter0_BASE)
#define DFSDM1_1              ((DFSDM_TypeDef *) DFSDM1_Filter1_BASE)
/* Legacy Defines */
#define DFSDM0                DFSDM1_0
#define DFSDM1                DFSDM1_1
#if defined(STM32F413_423xx)
#define DFSDM2_BASE           (APB2PERIPH_BASE + 0x6400U)
#define DFSDM2_Channel0_BASE  (DFSDM2_BASE + 0x00U)
#define DFSDM2_Channel1_BASE  (DFSDM2_BASE + 0x20U)
#define DFSDM2_Channel2_BASE  (DFSDM2_BASE + 0x40U)
#define DFSDM2_Channel3_BASE  (DFSDM2_BASE + 0x60U)
#define DFSDM2_Channel4_BASE  (DFSDM2_BASE + 0x80U)
#define DFSDM2_Channel5_BASE  (DFSDM2_BASE + 0xA0U)
#define DFSDM2_Channel6_BASE  (DFSDM2_BASE + 0xC0U)
#define DFSDM2_Channel7_BASE  (DFSDM2_BASE + 0xE0U)
#define DFSDM2_Filter0_BASE   (DFSDM2_BASE + 0x100U)
#define DFSDM2_Filter1_BASE   (DFSDM2_BASE + 0x180U)
#define DFSDM2_Filter2_BASE   (DFSDM2_BASE + 0x200U)
#define DFSDM2_Filter3_BASE   (DFSDM2_BASE + 0x280U)
#define DFSDM2_0              ((DFSDM_TypeDef *) DFSDM2_Filter0_BASE)
#define DFSDM2_1              ((DFSDM_TypeDef *) DFSDM2_Filter1_BASE)
#define DFSDM2_2              ((DFSDM_TypeDef *) DFSDM2_Filter2_BASE)
#define DFSDM2_3              ((DFSDM_TypeDef *) DFSDM2_Filter3_BASE)
#endif /* STM32F413_423xx */
#endif /* STM32F412xG ||  STM32F413_423xx */

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASE            (AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASE            (AHB1PERIPH_BASE + 0x2800)
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
#define ETH_BASE              (AHB1PERIPH_BASE + 0x8000)
#define ETH_MAC_BASE          (ETH_BASE)
#define ETH_MMC_BASE          (ETH_BASE + 0x0100)
#define ETH_PTP_BASE          (ETH_BASE + 0x0700)
#define ETH_DMA_BASE          (ETH_BASE + 0x1000)
#define DMA2D_BASE            (AHB1PERIPH_BASE + 0xB000)

/*!< AHB2 peripherals */
#define DCMI_BASE             (AHB2PERIPH_BASE + 0x50000)
#define CRYP_BASE             (AHB2PERIPH_BASE + 0x60000)
#define HASH_BASE             (AHB2PERIPH_BASE + 0x60400)
#define HASH_DIGEST_BASE      (AHB2PERIPH_BASE + 0x60710)
#define RNG_BASE              (AHB2PERIPH_BASE + 0x60800)

#if defined(STM32F40_41xxx) || defined(STM32F412xG) || defined(STM32F413_423xx)
/*!< FSMC Bankx registers base address */
#define FSMC_Bank1_R_BASE     (FSMC_R_BASE + 0x0000)
#define FSMC_Bank1E_R_BASE    (FSMC_R_BASE + 0x0104)
#define FSMC_Bank2_R_BASE     (FSMC_R_BASE + 0x0060)
#define FSMC_Bank3_R_BASE     (FSMC_R_BASE + 0x0080)
#define FSMC_Bank4_R_BASE     (FSMC_R_BASE + 0x00A0)
#endif /* STM32F40_41xxx || STM32F412xG || STM32F413_423xx */

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
/*!< FMC Bankx registers base address */
#define FMC_Bank1_R_BASE      (FMC_R_BASE + 0x0000)
#define FMC_Bank1E_R_BASE     (FMC_R_BASE + 0x0104)
#define FMC_Bank2_R_BASE      (FMC_R_BASE + 0x0060)
#define FMC_Bank3_R_BASE      (FMC_R_BASE + 0x0080)
#define FMC_Bank4_R_BASE      (FMC_R_BASE + 0x00A0)
#define FMC_Bank5_6_R_BASE    (FMC_R_BASE + 0x0140)
#endif /* STM32F427_437xx ||  STM32F429_439xx || STM32F446xx || STM32F469_479xx */

/* Debug MCU registers base address */
#define DBGMCU_BASE           ((uint32_t )0xE0042000)

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define QUADSPI             ((QUADSPI_TypeDef *) QSPI_R_BASE)
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx || STM32F469_479xx */
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM12               ((TIM_TypeDef *) TIM12_BASE)
#define TIM13               ((TIM_TypeDef *) TIM13_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#if defined(STM32F446xx)
#define SPDIFRX             ((SPDIFRX_TypeDef *) SPDIFRX_BASE)
#endif /* STM32F446xx */
#define I2S3ext             ((SPI_TypeDef *) I2S3ext_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define UART4               ((USART_TypeDef *) UART4_BASE)
#define UART5               ((USART_TypeDef *) UART5_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#if defined(STM32F410xx) || defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
#define FMPI2C1             ((FMPI2C_TypeDef *) FMPI2C1_BASE)
#endif /* STM32F410xx || STM32F412xG || STM32F413_423xx || STM32F446xx */
#if defined(STM32F410xx) || defined(STM32F413_423xx)
#define LPTIM1              ((LPTIM_TypeDef *) LPTIM1_BASE)
#endif /* STM32F410xx || STM32F413_423xx */
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define CAN2                ((CAN_TypeDef *) CAN2_BASE)
#if defined(STM32F413_423xx)
#define CAN3                ((CAN_TypeDef *) CAN3_BASE)
#endif /* STM32F413_423xx */
#if defined(STM32F446xx)
#define CEC                 ((CEC_TypeDef *) CEC_BASE)
#endif /* STM32F446xx */
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE)
#define UART7               ((USART_TypeDef *) UART7_BASE)
#define UART8               ((USART_TypeDef *) UART8_BASE)
#define UART9               ((USART_TypeDef *) UART9_BASE)
#define UART10              ((USART_TypeDef *) UART10_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM8                ((TIM_TypeDef *) TIM8_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define ADC                 ((ADC_Common_TypeDef *) ADC_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI4                ((SPI_TypeDef *) SPI4_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define SPI5                ((SPI_TypeDef *) SPI5_BASE)
#define SPI6                ((SPI_TypeDef *) SPI6_BASE)
#define SAI1                ((SAI_TypeDef *) SAI1_BASE)
#define SAI1_Block_A        ((SAI_Block_TypeDef *)SAI1_Block_A_BASE)
#define SAI1_Block_B        ((SAI_Block_TypeDef *)SAI1_Block_B_BASE)
#if defined(STM32F446xx)
#define SAI2                ((SAI_TypeDef *) SAI2_BASE)
#define SAI2_Block_A        ((SAI_Block_TypeDef *)SAI2_Block_A_BASE)
#define SAI2_Block_B        ((SAI_Block_TypeDef *)SAI2_Block_B_BASE)
#endif /* STM32F446xx */
#define LTDC                ((LTDC_TypeDef *)LTDC_BASE)
#define LTDC_Layer1         ((LTDC_Layer_TypeDef *)LTDC_Layer1_BASE)
#define LTDC_Layer2         ((LTDC_Layer_TypeDef *)LTDC_Layer2_BASE)
#if defined(STM32F469_479xx)
#define DSI                 ((DSI_TypeDef *)DSI_BASE)
#endif /* STM32F469_479xx */
#if defined(STM32F412xG) || defined(STM32F413_423xx)
#define DFSDM1_Channel0     ((DFSDM_Channel_TypeDef *) DFSDM1_Channel0_BASE)
#define DFSDM1_Channel1     ((DFSDM_Channel_TypeDef *) DFSDM1_Channel1_BASE)
#define DFSDM1_Channel2     ((DFSDM_Channel_TypeDef *) DFSDM1_Channel2_BASE)
#define DFSDM1_Channel3     ((DFSDM_Channel_TypeDef *) DFSDM1_Channel3_BASE)
#define DFSDM1_Filter0      ((DFSDM_TypeDef *) DFSDM_Filter0_BASE)
#define DFSDM1_Filter1      ((DFSDM_TypeDef *) DFSDM_Filter1_BASE)
#if defined(STM32F413_423xx)
#define DFSDM2_Channel0     ((DFSDM_Channel_TypeDef *) DFSDM2_Channel0_BASE)
#define DFSDM2_Channel1     ((DFSDM_Channel_TypeDef *) DFSDM2_Channel1_BASE)
#define DFSDM2_Channel2     ((DFSDM_Channel_TypeDef *) DFSDM2_Channel2_BASE)
#define DFSDM2_Channel3     ((DFSDM_Channel_TypeDef *) DFSDM2_Channel3_BASE)
#define DFSDM2_Channel4     ((DFSDM_Channel_TypeDef *) DFSDM2_Channel4_BASE)
#define DFSDM2_Channel5     ((DFSDM_Channel_TypeDef *) DFSDM2_Channel5_BASE)
#define DFSDM2_Channel6     ((DFSDM_Channel_TypeDef *) DFSDM2_Channel6_BASE)
#define DFSDM2_Channel7     ((DFSDM_Channel_TypeDef *) DFSDM2_Channel7_BASE)
#define DFSDM2_Filter0      ((DFSDM_Filter_TypeDef *) DFSDM2_Filter0_BASE)
#define DFSDM2_Filter1      ((DFSDM_Filter_TypeDef *) DFSDM2_Filter1_BASE)
#define DFSDM2_Filter2      ((DFSDM_Filter_TypeDef *) DFSDM2_Filter2_BASE)
#define DFSDM2_Filter3      ((DFSDM_Filter_TypeDef *) DFSDM2_Filter3_BASE)
#endif /* STM32F413_423xx */
#endif /* STM32F412xG || STM32F413_423xx */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
#define GPIOJ               ((GPIO_TypeDef *) GPIOJ_BASE)
#define GPIOK               ((GPIO_TypeDef *) GPIOK_BASE)
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
#define ETH                 ((ETH_TypeDef *) ETH_BASE)
#define DMA2D               ((DMA2D_TypeDef *)DMA2D_BASE)
#define DCMI                ((DCMI_TypeDef *) DCMI_BASE)
#define CRYP                ((CRYP_TypeDef *) CRYP_BASE)
#define HASH                ((HASH_TypeDef *) HASH_BASE)
#define HASH_DIGEST         ((HASH_DIGEST_TypeDef *) HASH_DIGEST_BASE)
#define RNG                 ((RNG_TypeDef *) RNG_BASE)

#if defined(STM32F40_41xxx) || defined(STM32F412xG) || defined(STM32F413_423xx)
#define FSMC_Bank1          ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
#define FSMC_Bank1E         ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
#define FSMC_Bank2          ((FSMC_Bank2_TypeDef *) FSMC_Bank2_R_BASE)
#define FSMC_Bank3          ((FSMC_Bank3_TypeDef *) FSMC_Bank3_R_BASE)
#define FSMC_Bank4          ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)
#endif /* STM32F40_41xxx || STM32F412xG || STM32F413_423xx */

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define FMC_Bank1           ((FMC_Bank1_TypeDef *) FMC_Bank1_R_BASE)
#define FMC_Bank1E          ((FMC_Bank1E_TypeDef *) FMC_Bank1E_R_BASE)
#define FMC_Bank2           ((FMC_Bank2_TypeDef *) FMC_Bank2_R_BASE)
#define FMC_Bank3           ((FMC_Bank3_TypeDef *) FMC_Bank3_R_BASE)
#define FMC_Bank4           ((FMC_Bank4_TypeDef *) FMC_Bank4_R_BASE)
#define FMC_Bank5_6         ((FMC_Bank5_6_TypeDef *) FMC_Bank5_6_R_BASE)
#endif /* STM32F427_437xx ||  STM32F429_439xx || STM32F446xx || STM32F469_479xx */

#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

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
#define  ADC_SR_AWD                          (0x01U)               /*!<Analog watchdog flag               */
#define  ADC_SR_EOC                          (0x02U)               /*!<End of conversion                  */
#define  ADC_SR_JEOC                         (0x04U)               /*!<Injected channel end of conversion */
#define  ADC_SR_JSTRT                        (0x08U)               /*!<Injected channel Start flag        */
#define  ADC_SR_STRT                         (0x10U)               /*!<Regular channel Start flag         */
#define  ADC_SR_OVR                          (0x20U)               /*!<Overrun flag                       */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       (0x0000001FUL)        /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     (0x00000001UL)        /*!<Bit 0 */
#define  ADC_CR1_AWDCH_1                     (0x00000002UL)        /*!<Bit 1 */
#define  ADC_CR1_AWDCH_2                     (0x00000004UL)        /*!<Bit 2 */
#define  ADC_CR1_AWDCH_3                     (0x00000008UL)        /*!<Bit 3 */
#define  ADC_CR1_AWDCH_4                     (0x00000010UL)        /*!<Bit 4 */
#define  ADC_CR1_EOCIE                       (0x00000020UL)        /*!<Interrupt enable for EOC                              */
#define  ADC_CR1_AWDIE                       (0x00000040UL)        /*!<AAnalog Watchdog interrupt enable                     */
#define  ADC_CR1_JEOCIE                      (0x00000080UL)        /*!<Interrupt enable for injected channels                */
#define  ADC_CR1_SCAN                        (0x00000100UL)        /*!<Scan mode                                             */
#define  ADC_CR1_AWDSGL                      (0x00000200UL)        /*!<Enable the watchdog on a single channel in scan mode  */
#define  ADC_CR1_JAUTO                       (0x00000400UL)        /*!<Automatic injected group conversion                   */
#define  ADC_CR1_DISCEN                      (0x00000800UL)        /*!<Discontinuous mode on regular channels                */
#define  ADC_CR1_JDISCEN                     (0x00001000UL)        /*!<Discontinuous mode on injected channels               */
#define  ADC_CR1_DISCNUM                     (0x0000E000UL)        /*!<DISCNUM[2:0] bits (Discontinuous mode channel count)  */
#define  ADC_CR1_DISCNUM_0                   (0x00002000UL)        /*!<Bit 0 */
#define  ADC_CR1_DISCNUM_1                   (0x00004000UL)        /*!<Bit 1 */
#define  ADC_CR1_DISCNUM_2                   (0x00008000UL)        /*!<Bit 2 */
#define  ADC_CR1_JAWDEN                      (0x00400000UL)        /*!<Analog watchdog enable on injected channels           */
#define  ADC_CR1_AWDEN                       (0x00800000UL)        /*!<Analog watchdog enable on regular channels            */
#define  ADC_CR1_RES                         (0x03000000UL)        /*!<RES[2:0] bits (Resolution)                            */
#define  ADC_CR1_RES_0                       (0x01000000UL)        /*!<Bit 0 */
#define  ADC_CR1_RES_1                       (0x02000000UL)        /*!<Bit 1 */
#define  ADC_CR1_OVRIE                       (0x04000000UL)         /*!<overrun interrupt enable                              */

/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        (0x00000001UL)        /*!<A/D Converter ON / OFF             */
#define  ADC_CR2_CONT                        (0x00000002UL)        /*!<Continuous Conversion              */
#define  ADC_CR2_DMA                         (0x00000100UL)        /*!<Direct Memory access mode          */
#define  ADC_CR2_DDS                         (0x00000200UL)        /*!<DMA disable selection (Single ADC) */
#define  ADC_CR2_EOCS                        (0x00000400UL)        /*!<End of conversion selection        */
#define  ADC_CR2_ALIGN                       (0x00000800UL)        /*!<Data Alignment                     */
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
#define  ADC_JOFR1_JOFFSET1                  (0x0FFFU)            /*!<Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  (0x0FFFU)            /*!<Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  (0x0FFFU)            /*!<Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  (0x0FFFU)            /*!<Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          (0x0FFFU)            /*!<Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          (0x0FFFU)            /*!<Analog watchdog low threshold */

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
#define  ADC_JDR1_JDATA                      (0xFFFFU)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      (0xFFFFU)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      (0xFFFFU)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      (0xFFFFU)            /*!<Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         (0x0000FFFFUL)        /*!<Regular data */
#define  ADC_DR_ADC2DATA                     (0xFFFF0000UL)        /*!<ADC2 data */

/*******************  Bit definition for ADC_CSR register  ********************/
#define  ADC_CSR_AWD1                        (0x00000001UL)        /*!<ADC1 Analog watchdog flag */
#define  ADC_CSR_EOC1                        (0x00000002UL)        /*!<ADC1 End of conversion */
#define  ADC_CSR_JEOC1                       (0x00000004UL)        /*!<ADC1 Injected channel end of conversion */
#define  ADC_CSR_JSTRT1                      (0x00000008UL)        /*!<ADC1 Injected channel Start flag */
#define  ADC_CSR_STRT1                       (0x00000010UL)        /*!<ADC1 Regular channel Start flag */
#define  ADC_CSR_OVR1                        (0x00000020UL)        /*!<ADC1 DMA overrun  flag */
#define  ADC_CSR_AWD2                        (0x00000100UL)        /*!<ADC2 Analog watchdog flag */
#define  ADC_CSR_EOC2                        (0x00000200UL)        /*!<ADC2 End of conversion */
#define  ADC_CSR_JEOC2                       (0x00000400UL)        /*!<ADC2 Injected channel end of conversion */
#define  ADC_CSR_JSTRT2                      (0x00000800UL)        /*!<ADC2 Injected channel Start flag */
#define  ADC_CSR_STRT2                       (0x00001000UL)        /*!<ADC2 Regular channel Start flag */
#define  ADC_CSR_OVR2                        (0x00002000UL)        /*!<ADC2 DMA overrun  flag */
#define  ADC_CSR_AWD3                        (0x00010000UL)        /*!<ADC3 Analog watchdog flag */
#define  ADC_CSR_EOC3                        (0x00020000UL)        /*!<ADC3 End of conversion */
#define  ADC_CSR_JEOC3                       (0x00040000UL)        /*!<ADC3 Injected channel end of conversion */
#define  ADC_CSR_JSTRT3                      (0x00080000UL)        /*!<ADC3 Injected channel Start flag */
#define  ADC_CSR_STRT3                       (0x00100000UL)        /*!<ADC3 Regular channel Start flag */
#define  ADC_CSR_OVR3                        (0x00200000UL)        /*!<ADC3 DMA overrun  flag */

/* Legacy defines */
#define  ADC_CSR_DOVR1                        ADC_CSR_OVR1
#define  ADC_CSR_DOVR2                        ADC_CSR_OVR2
#define  ADC_CSR_DOVR3                        ADC_CSR_OVR3

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
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/
/*!<CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define  CAN_MCR_INRQ                        (0x0001U)            /*!<Initialization Request */
#define  CAN_MCR_SLEEP                       (0x0002U)            /*!<Sleep Mode Request */
#define  CAN_MCR_TXFP                        (0x0004U)            /*!<Transmit FIFO Priority */
#define  CAN_MCR_RFLM                        (0x0008U)            /*!<Receive FIFO Locked Mode */
#define  CAN_MCR_NART                        (0x0010U)            /*!<No Automatic Retransmission */
#define  CAN_MCR_AWUM                        (0x0020U)            /*!<Automatic Wakeup Mode */
#define  CAN_MCR_ABOM                        (0x0040U)            /*!<Automatic Bus-Off Management */
#define  CAN_MCR_TTCM                        (0x0080U)            /*!<Time Triggered Communication Mode */
#define  CAN_MCR_RESET                       (0x8000U)            /*!<bxCAN software master reset */

/*******************  Bit definition for CAN_MSR register  ********************/
#define  CAN_MSR_INAK                        (0x0001U)            /*!<Initialization Acknowledge */
#define  CAN_MSR_SLAK                        (0x0002U)            /*!<Sleep Acknowledge */
#define  CAN_MSR_ERRI                        (0x0004U)            /*!<Error Interrupt */
#define  CAN_MSR_WKUI                        (0x0008U)            /*!<Wakeup Interrupt */
#define  CAN_MSR_SLAKI                       (0x0010U)            /*!<Sleep Acknowledge Interrupt */
#define  CAN_MSR_TXM                         (0x0100U)            /*!<Transmit Mode */
#define  CAN_MSR_RXM                         (0x0200U)            /*!<Receive Mode */
#define  CAN_MSR_SAMP                        (0x0400U)            /*!<Last Sample Point */
#define  CAN_MSR_RX                          (0x0800U)            /*!<CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define  CAN_TSR_RQCP0                       (0x00000001UL)        /*!<Request Completed Mailbox0 */
#define  CAN_TSR_TXOK0                       (0x00000002UL)        /*!<Transmission OK of Mailbox0 */
#define  CAN_TSR_ALST0                       (0x00000004UL)        /*!<Arbitration Lost for Mailbox0 */
#define  CAN_TSR_TERR0                       (0x00000008UL)        /*!<Transmission Error of Mailbox0 */
#define  CAN_TSR_ABRQ0                       (0x00000080UL)        /*!<Abort Request for Mailbox0 */
#define  CAN_TSR_RQCP1                       (0x00000100UL)        /*!<Request Completed Mailbox1 */
#define  CAN_TSR_TXOK1                       (0x00000200UL)        /*!<Transmission OK of Mailbox1 */
#define  CAN_TSR_ALST1                       (0x00000400UL)        /*!<Arbitration Lost for Mailbox1 */
#define  CAN_TSR_TERR1                       (0x00000800UL)        /*!<Transmission Error of Mailbox1 */
#define  CAN_TSR_ABRQ1                       (0x00008000UL)        /*!<Abort Request for Mailbox 1 */
#define  CAN_TSR_RQCP2                       (0x00010000UL)        /*!<Request Completed Mailbox2 */
#define  CAN_TSR_TXOK2                       (0x00020000UL)        /*!<Transmission OK of Mailbox 2 */
#define  CAN_TSR_ALST2                       (0x00040000UL)        /*!<Arbitration Lost for mailbox 2 */
#define  CAN_TSR_TERR2                       (0x00080000UL)        /*!<Transmission Error of Mailbox 2 */
#define  CAN_TSR_ABRQ2                       (0x00800000UL)        /*!<Abort Request for Mailbox 2 */
#define  CAN_TSR_CODE                        (0x03000000UL)        /*!<Mailbox Code */

#define  CAN_TSR_TME                         (0x1C000000UL)        /*!<TME[2:0] bits */
#define  CAN_TSR_TME0                        (0x04000000UL)        /*!<Transmit Mailbox 0 Empty */
#define  CAN_TSR_TME1                        (0x08000000UL)        /*!<Transmit Mailbox 1 Empty */
#define  CAN_TSR_TME2                        (0x10000000UL)        /*!<Transmit Mailbox 2 Empty */

#define  CAN_TSR_LOW                         (0xE0000000UL)        /*!<LOW[2:0] bits */
#define  CAN_TSR_LOW0                        (0x20000000UL)        /*!<Lowest Priority Flag for Mailbox 0 */
#define  CAN_TSR_LOW1                        (0x40000000UL)        /*!<Lowest Priority Flag for Mailbox 1 */
#define  CAN_TSR_LOW2                        (0x80000000UL)        /*!<Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define  CAN_RF0R_FMP0                       (0x03U)               /*!<FIFO 0 Message Pending */
#define  CAN_RF0R_FULL0                      (0x08U)               /*!<FIFO 0 Full */
#define  CAN_RF0R_FOVR0                      (0x10U)               /*!<FIFO 0 Overrun */
#define  CAN_RF0R_RFOM0                      (0x20U)               /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define  CAN_RF1R_FMP1                       (0x03U)               /*!<FIFO 1 Message Pending */
#define  CAN_RF1R_FULL1                      (0x08U)               /*!<FIFO 1 Full */
#define  CAN_RF1R_FOVR1                      (0x10U)               /*!<FIFO 1 Overrun */
#define  CAN_RF1R_RFOM1                      (0x20U)               /*!<Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define  CAN_IER_TMEIE                       (0x00000001UL)        /*!<Transmit Mailbox Empty Interrupt Enable */
#define  CAN_IER_FMPIE0                      (0x00000002UL)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE0                       (0x00000004UL)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE0                      (0x00000008UL)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_FMPIE1                      (0x00000010UL)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE1                       (0x00000020UL)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE1                      (0x00000040UL)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_EWGIE                       (0x00000100UL)        /*!<Error Warning Interrupt Enable */
#define  CAN_IER_EPVIE                       (0x00000200UL)        /*!<Error Passive Interrupt Enable */
#define  CAN_IER_BOFIE                       (0x00000400UL)        /*!<Bus-Off Interrupt Enable */
#define  CAN_IER_LECIE                       (0x00000800UL)        /*!<Last Error Code Interrupt Enable */
#define  CAN_IER_ERRIE                       (0x00008000UL)        /*!<Error Interrupt Enable */
#define  CAN_IER_WKUIE                       (0x00010000UL)        /*!<Wakeup Interrupt Enable */
#define  CAN_IER_SLKIE                       (0x00020000UL)        /*!<Sleep Interrupt Enable */

/********************  Bit definition for CAN_ESR register  *******************/
#define  CAN_ESR_EWGF                        (0x00000001UL)        /*!<Error Warning Flag */
#define  CAN_ESR_EPVF                        (0x00000002UL)        /*!<Error Passive Flag */
#define  CAN_ESR_BOFF                        (0x00000004UL)        /*!<Bus-Off Flag */

#define  CAN_ESR_LEC                         (0x00000070UL)        /*!<LEC[2:0] bits (Last Error Code) */
#define  CAN_ESR_LEC_0                       (0x00000010UL)        /*!<Bit 0 */
#define  CAN_ESR_LEC_1                       (0x00000020UL)        /*!<Bit 1 */
#define  CAN_ESR_LEC_2                       (0x00000040UL)        /*!<Bit 2 */

#define  CAN_ESR_TEC                         (0x00FF0000UL)        /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define  CAN_ESR_REC                         (0xFF000000UL)        /*!<Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define  CAN_BTR_BRP                         (0x000003FFUL)        /*!<Baud Rate Prescaler */
#define  CAN_BTR_TS1                         (0x000F0000UL)        /*!<Time Segment 1 */
#define  CAN_BTR_TS2                         (0x00700000UL)        /*!<Time Segment 2 */
#define  CAN_BTR_SJW                         (0x03000000UL)        /*!<Resynchronization Jump Width */
#define  CAN_BTR_LBKM                        (0x40000000UL)        /*!<Loop Back Mode (Debug) */
#define  CAN_BTR_SILM                        (0x80000000UL)        /*!<Silent Mode */

/*!<Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define  CAN_TI0R_TXRQ                       (0x00000001UL)        /*!<Transmit Mailbox Request */
#define  CAN_TI0R_RTR                        (0x00000002UL)        /*!<Remote Transmission Request */
#define  CAN_TI0R_IDE                        (0x00000004UL)        /*!<Identifier Extension */
#define  CAN_TI0R_EXID                       (0x001FFFF8UL)        /*!<Extended Identifier */
#define  CAN_TI0R_STID                       (0xFFE00000UL)        /*!<Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define  CAN_TDT0R_DLC                       (0x0000000FUL)        /*!<Data Length Code */
#define  CAN_TDT0R_TGT                       (0x00000100UL)        /*!<Transmit Global Time */
#define  CAN_TDT0R_TIME                      (0xFFFF0000UL)        /*!<Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define  CAN_TDL0R_DATA0                     (0x000000FFUL)        /*!<Data byte 0 */
#define  CAN_TDL0R_DATA1                     (0x0000FF00UL)        /*!<Data byte 1 */
#define  CAN_TDL0R_DATA2                     (0x00FF0000UL)        /*!<Data byte 2 */
#define  CAN_TDL0R_DATA3                     (0xFF000000UL)        /*!<Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define  CAN_TDH0R_DATA4                     (0x000000FFUL)        /*!<Data byte 4 */
#define  CAN_TDH0R_DATA5                     (0x0000FF00UL)        /*!<Data byte 5 */
#define  CAN_TDH0R_DATA6                     (0x00FF0000UL)        /*!<Data byte 6 */
#define  CAN_TDH0R_DATA7                     (0xFF000000UL)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define  CAN_TI1R_TXRQ                       (0x00000001UL)        /*!<Transmit Mailbox Request */
#define  CAN_TI1R_RTR                        (0x00000002UL)        /*!<Remote Transmission Request */
#define  CAN_TI1R_IDE                        (0x00000004UL)        /*!<Identifier Extension */
#define  CAN_TI1R_EXID                       (0x001FFFF8UL)        /*!<Extended Identifier */
#define  CAN_TI1R_STID                       (0xFFE00000UL)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define  CAN_TDT1R_DLC                       (0x0000000FUL)        /*!<Data Length Code */
#define  CAN_TDT1R_TGT                       (0x00000100UL)        /*!<Transmit Global Time */
#define  CAN_TDT1R_TIME                      (0xFFFF0000UL)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define  CAN_TDL1R_DATA0                     (0x000000FFUL)        /*!<Data byte 0 */
#define  CAN_TDL1R_DATA1                     (0x0000FF00UL)        /*!<Data byte 1 */
#define  CAN_TDL1R_DATA2                     (0x00FF0000UL)        /*!<Data byte 2 */
#define  CAN_TDL1R_DATA3                     (0xFF000000UL)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define  CAN_TDH1R_DATA4                     (0x000000FFUL)        /*!<Data byte 4 */
#define  CAN_TDH1R_DATA5                     (0x0000FF00UL)        /*!<Data byte 5 */
#define  CAN_TDH1R_DATA6                     (0x00FF0000UL)        /*!<Data byte 6 */
#define  CAN_TDH1R_DATA7                     (0xFF000000UL)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define  CAN_TI2R_TXRQ                       (0x00000001UL)        /*!<Transmit Mailbox Request */
#define  CAN_TI2R_RTR                        (0x00000002UL)        /*!<Remote Transmission Request */
#define  CAN_TI2R_IDE                        (0x00000004UL)        /*!<Identifier Extension */
#define  CAN_TI2R_EXID                       (0x001FFFF8UL)        /*!<Extended identifier */
#define  CAN_TI2R_STID                       (0xFFE00000UL)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
#define  CAN_TDT2R_DLC                       (0x0000000FUL)        /*!<Data Length Code */
#define  CAN_TDT2R_TGT                       (0x00000100UL)        /*!<Transmit Global Time */
#define  CAN_TDT2R_TIME                      (0xFFFF0000UL)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define  CAN_TDL2R_DATA0                     (0x000000FFUL)        /*!<Data byte 0 */
#define  CAN_TDL2R_DATA1                     (0x0000FF00UL)        /*!<Data byte 1 */
#define  CAN_TDL2R_DATA2                     (0x00FF0000UL)        /*!<Data byte 2 */
#define  CAN_TDL2R_DATA3                     (0xFF000000UL)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define  CAN_TDH2R_DATA4                     (0x000000FFUL)        /*!<Data byte 4 */
#define  CAN_TDH2R_DATA5                     (0x0000FF00UL)        /*!<Data byte 5 */
#define  CAN_TDH2R_DATA6                     (0x00FF0000UL)        /*!<Data byte 6 */
#define  CAN_TDH2R_DATA7                     (0xFF000000UL)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define  CAN_RI0R_RTR                        (0x00000002UL)        /*!<Remote Transmission Request */
#define  CAN_RI0R_IDE                        (0x00000004UL)        /*!<Identifier Extension */
#define  CAN_RI0R_EXID                       (0x001FFFF8UL)        /*!<Extended Identifier */
#define  CAN_RI0R_STID                       (0xFFE00000UL)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define  CAN_RDT0R_DLC                       (0x0000000FUL)        /*!<Data Length Code */
#define  CAN_RDT0R_FMI                       (0x0000FF00UL)        /*!<Filter Match Index */
#define  CAN_RDT0R_TIME                      (0xFFFF0000UL)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define  CAN_RDL0R_DATA0                     (0x000000FFUL)        /*!<Data byte 0 */
#define  CAN_RDL0R_DATA1                     (0x0000FF00UL)        /*!<Data byte 1 */
#define  CAN_RDL0R_DATA2                     (0x00FF0000UL)        /*!<Data byte 2 */
#define  CAN_RDL0R_DATA3                     (0xFF000000UL)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define  CAN_RDH0R_DATA4                     (0x000000FFUL)        /*!<Data byte 4 */
#define  CAN_RDH0R_DATA5                     (0x0000FF00UL)        /*!<Data byte 5 */
#define  CAN_RDH0R_DATA6                     (0x00FF0000UL)        /*!<Data byte 6 */
#define  CAN_RDH0R_DATA7                     (0xFF000000UL)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define  CAN_RI1R_RTR                        (0x00000002UL)        /*!<Remote Transmission Request */
#define  CAN_RI1R_IDE                        (0x00000004UL)        /*!<Identifier Extension */
#define  CAN_RI1R_EXID                       (0x001FFFF8UL)        /*!<Extended identifier */
#define  CAN_RI1R_STID                       (0xFFE00000UL)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define  CAN_RDT1R_DLC                       (0x0000000FUL)        /*!<Data Length Code */
#define  CAN_RDT1R_FMI                       (0x0000FF00UL)        /*!<Filter Match Index */
#define  CAN_RDT1R_TIME                      (0xFFFF0000UL)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define  CAN_RDL1R_DATA0                     (0x000000FFUL)        /*!<Data byte 0 */
#define  CAN_RDL1R_DATA1                     (0x0000FF00UL)        /*!<Data byte 1 */
#define  CAN_RDL1R_DATA2                     (0x00FF0000UL)        /*!<Data byte 2 */
#define  CAN_RDL1R_DATA3                     (0xFF000000UL)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define  CAN_RDH1R_DATA4                     (0x000000FFUL)        /*!<Data byte 4 */
#define  CAN_RDH1R_DATA5                     (0x0000FF00UL)        /*!<Data byte 5 */
#define  CAN_RDH1R_DATA6                     (0x00FF0000UL)        /*!<Data byte 6 */
#define  CAN_RDH1R_DATA7                     (0xFF000000UL)        /*!<Data byte 7 */

/*!<CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define  CAN_FMR_FINIT                       (0x01U)               /*!<Filter Init Mode */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define  CAN_FM1R_FBM                        (0x3FFFU)            /*!<Filter Mode */
#define  CAN_FM1R_FBM0                       (0x0001U)            /*!<Filter Init Mode bit 0 */
#define  CAN_FM1R_FBM1                       (0x0002U)            /*!<Filter Init Mode bit 1 */
#define  CAN_FM1R_FBM2                       (0x0004U)            /*!<Filter Init Mode bit 2 */
#define  CAN_FM1R_FBM3                       (0x0008U)            /*!<Filter Init Mode bit 3 */
#define  CAN_FM1R_FBM4                       (0x0010U)            /*!<Filter Init Mode bit 4 */
#define  CAN_FM1R_FBM5                       (0x0020U)            /*!<Filter Init Mode bit 5 */
#define  CAN_FM1R_FBM6                       (0x0040U)            /*!<Filter Init Mode bit 6 */
#define  CAN_FM1R_FBM7                       (0x0080U)            /*!<Filter Init Mode bit 7 */
#define  CAN_FM1R_FBM8                       (0x0100U)            /*!<Filter Init Mode bit 8 */
#define  CAN_FM1R_FBM9                       (0x0200U)            /*!<Filter Init Mode bit 9 */
#define  CAN_FM1R_FBM10                      (0x0400U)            /*!<Filter Init Mode bit 10 */
#define  CAN_FM1R_FBM11                      (0x0800U)            /*!<Filter Init Mode bit 11 */
#define  CAN_FM1R_FBM12                      (0x1000U)            /*!<Filter Init Mode bit 12 */
#define  CAN_FM1R_FBM13                      (0x2000U)            /*!<Filter Init Mode bit 13 */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define  CAN_FS1R_FSC                        (0x3FFFU)            /*!<Filter Scale Configuration */
#define  CAN_FS1R_FSC0                       (0x0001U)            /*!<Filter Scale Configuration bit 0 */
#define  CAN_FS1R_FSC1                       (0x0002U)            /*!<Filter Scale Configuration bit 1 */
#define  CAN_FS1R_FSC2                       (0x0004U)            /*!<Filter Scale Configuration bit 2 */
#define  CAN_FS1R_FSC3                       (0x0008U)            /*!<Filter Scale Configuration bit 3 */
#define  CAN_FS1R_FSC4                       (0x0010U)            /*!<Filter Scale Configuration bit 4 */
#define  CAN_FS1R_FSC5                       (0x0020U)            /*!<Filter Scale Configuration bit 5 */
#define  CAN_FS1R_FSC6                       (0x0040U)            /*!<Filter Scale Configuration bit 6 */
#define  CAN_FS1R_FSC7                       (0x0080U)            /*!<Filter Scale Configuration bit 7 */
#define  CAN_FS1R_FSC8                       (0x0100U)            /*!<Filter Scale Configuration bit 8 */
#define  CAN_FS1R_FSC9                       (0x0200U)            /*!<Filter Scale Configuration bit 9 */
#define  CAN_FS1R_FSC10                      (0x0400U)            /*!<Filter Scale Configuration bit 10 */
#define  CAN_FS1R_FSC11                      (0x0800U)            /*!<Filter Scale Configuration bit 11 */
#define  CAN_FS1R_FSC12                      (0x1000U)            /*!<Filter Scale Configuration bit 12 */
#define  CAN_FS1R_FSC13                      (0x2000U)            /*!<Filter Scale Configuration bit 13 */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define  CAN_FFA1R_FFA                       (0x3FFFU)            /*!<Filter FIFO Assignment */
#define  CAN_FFA1R_FFA0                      (0x0001U)            /*!<Filter FIFO Assignment for Filter 0 */
#define  CAN_FFA1R_FFA1                      (0x0002U)            /*!<Filter FIFO Assignment for Filter 1 */
#define  CAN_FFA1R_FFA2                      (0x0004U)            /*!<Filter FIFO Assignment for Filter 2 */
#define  CAN_FFA1R_FFA3                      (0x0008U)            /*!<Filter FIFO Assignment for Filter 3 */
#define  CAN_FFA1R_FFA4                      (0x0010U)            /*!<Filter FIFO Assignment for Filter 4 */
#define  CAN_FFA1R_FFA5                      (0x0020U)            /*!<Filter FIFO Assignment for Filter 5 */
#define  CAN_FFA1R_FFA6                      (0x0040U)            /*!<Filter FIFO Assignment for Filter 6 */
#define  CAN_FFA1R_FFA7                      (0x0080U)            /*!<Filter FIFO Assignment for Filter 7 */
#define  CAN_FFA1R_FFA8                      (0x0100U)            /*!<Filter FIFO Assignment for Filter 8 */
#define  CAN_FFA1R_FFA9                      (0x0200U)            /*!<Filter FIFO Assignment for Filter 9 */
#define  CAN_FFA1R_FFA10                     (0x0400U)            /*!<Filter FIFO Assignment for Filter 10 */
#define  CAN_FFA1R_FFA11                     (0x0800U)            /*!<Filter FIFO Assignment for Filter 11 */
#define  CAN_FFA1R_FFA12                     (0x1000U)            /*!<Filter FIFO Assignment for Filter 12 */
#define  CAN_FFA1R_FFA13                     (0x2000U)            /*!<Filter FIFO Assignment for Filter 13 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define  CAN_FA1R_FACT                       (0x3FFFU)            /*!<Filter Active */
#define  CAN_FA1R_FACT0                      (0x0001U)            /*!<Filter 0 Active */
#define  CAN_FA1R_FACT1                      (0x0002U)            /*!<Filter 1 Active */
#define  CAN_FA1R_FACT2                      (0x0004U)            /*!<Filter 2 Active */
#define  CAN_FA1R_FACT3                      (0x0008U)            /*!<Filter 3 Active */
#define  CAN_FA1R_FACT4                      (0x0010U)            /*!<Filter 4 Active */
#define  CAN_FA1R_FACT5                      (0x0020U)            /*!<Filter 5 Active */
#define  CAN_FA1R_FACT6                      (0x0040U)            /*!<Filter 6 Active */
#define  CAN_FA1R_FACT7                      (0x0080U)            /*!<Filter 7 Active */
#define  CAN_FA1R_FACT8                      (0x0100U)            /*!<Filter 8 Active */
#define  CAN_FA1R_FACT9                      (0x0200U)            /*!<Filter 9 Active */
#define  CAN_FA1R_FACT10                     (0x0400U)            /*!<Filter 10 Active */
#define  CAN_FA1R_FACT11                     (0x0800U)            /*!<Filter 11 Active */
#define  CAN_FA1R_FACT12                     (0x1000U)            /*!<Filter 12 Active */
#define  CAN_FA1R_FACT13                     (0x2000U)            /*!<Filter 13 Active */

/*******************  Bit definition for CAN_F0R1 register  *******************/
#define  CAN_F0R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F0R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F0R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F0R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F0R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F0R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F0R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F0R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F0R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F0R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F0R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F0R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F0R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F0R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F0R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F0R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F0R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F0R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F0R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F0R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F0R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F0R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F0R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F0R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F0R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F0R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F0R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F0R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F0R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F0R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F0R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F0R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
#define  CAN_F1R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F1R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F1R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F1R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F1R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F1R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F1R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F1R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F1R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F1R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F1R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F1R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F1R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F1R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F1R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F1R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F1R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F1R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F1R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F1R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F1R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F1R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F1R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F1R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F1R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F1R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F1R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F1R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F1R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F1R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F1R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F1R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
#define  CAN_F2R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F2R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F2R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F2R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F2R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F2R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F2R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F2R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F2R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F2R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F2R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F2R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F2R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F2R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F2R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F2R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F2R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F2R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F2R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F2R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F2R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F2R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F2R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F2R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F2R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F2R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F2R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F2R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F2R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F2R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F2R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F2R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
#define  CAN_F3R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F3R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F3R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F3R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F3R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F3R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F3R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F3R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F3R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F3R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F3R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F3R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F3R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F3R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F3R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F3R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F3R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F3R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F3R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F3R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F3R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F3R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F3R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F3R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F3R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F3R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F3R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F3R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F3R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F3R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F3R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F3R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
#define  CAN_F4R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F4R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F4R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F4R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F4R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F4R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F4R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F4R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F4R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F4R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F4R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F4R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F4R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F4R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F4R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F4R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F4R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F4R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F4R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F4R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F4R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F4R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F4R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F4R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F4R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F4R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F4R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F4R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F4R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F4R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F4R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F4R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
#define  CAN_F5R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F5R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F5R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F5R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F5R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F5R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F5R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F5R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F5R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F5R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F5R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F5R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F5R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F5R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F5R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F5R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F5R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F5R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F5R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F5R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F5R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F5R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F5R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F5R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F5R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F5R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F5R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F5R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F5R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F5R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F5R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F5R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
#define  CAN_F6R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F6R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F6R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F6R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F6R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F6R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F6R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F6R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F6R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F6R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F6R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F6R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F6R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F6R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F6R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F6R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F6R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F6R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F6R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F6R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F6R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F6R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F6R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F6R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F6R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F6R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F6R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F6R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F6R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F6R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F6R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F6R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
#define  CAN_F7R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F7R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F7R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F7R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F7R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F7R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F7R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F7R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F7R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F7R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F7R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F7R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F7R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F7R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F7R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F7R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F7R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F7R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F7R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F7R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F7R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F7R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F7R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F7R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F7R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F7R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F7R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F7R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F7R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F7R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F7R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F7R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
#define  CAN_F8R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F8R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F8R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F8R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F8R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F8R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F8R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F8R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F8R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F8R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F8R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F8R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F8R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F8R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F8R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F8R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F8R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F8R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F8R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F8R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F8R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F8R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F8R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F8R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F8R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F8R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F8R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F8R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F8R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F8R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F8R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F8R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
#define  CAN_F9R1_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F9R1_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F9R1_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F9R1_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F9R1_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F9R1_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F9R1_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F9R1_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F9R1_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F9R1_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F9R1_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F9R1_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F9R1_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F9R1_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F9R1_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F9R1_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F9R1_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F9R1_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F9R1_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F9R1_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F9R1_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F9R1_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F9R1_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F9R1_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F9R1_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F9R1_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F9R1_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F9R1_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F9R1_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F9R1_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F9R1_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F9R1_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
#define  CAN_F10R1_FB0                       (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F10R1_FB1                       (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F10R1_FB2                       (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F10R1_FB3                       (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F10R1_FB4                       (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F10R1_FB5                       (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F10R1_FB6                       (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F10R1_FB7                       (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F10R1_FB8                       (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F10R1_FB9                       (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F10R1_FB10                      (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F10R1_FB11                      (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F10R1_FB12                      (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F10R1_FB13                      (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F10R1_FB14                      (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F10R1_FB15                      (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F10R1_FB16                      (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F10R1_FB17                      (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F10R1_FB18                      (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F10R1_FB19                      (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F10R1_FB20                      (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F10R1_FB21                      (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F10R1_FB22                      (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F10R1_FB23                      (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F10R1_FB24                      (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F10R1_FB25                      (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F10R1_FB26                      (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F10R1_FB27                      (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F10R1_FB28                      (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F10R1_FB29                      (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F10R1_FB30                      (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F10R1_FB31                      (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
#define  CAN_F11R1_FB0                       (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F11R1_FB1                       (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F11R1_FB2                       (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F11R1_FB3                       (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F11R1_FB4                       (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F11R1_FB5                       (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F11R1_FB6                       (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F11R1_FB7                       (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F11R1_FB8                       (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F11R1_FB9                       (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F11R1_FB10                      (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F11R1_FB11                      (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F11R1_FB12                      (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F11R1_FB13                      (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F11R1_FB14                      (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F11R1_FB15                      (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F11R1_FB16                      (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F11R1_FB17                      (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F11R1_FB18                      (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F11R1_FB19                      (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F11R1_FB20                      (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F11R1_FB21                      (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F11R1_FB22                      (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F11R1_FB23                      (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F11R1_FB24                      (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F11R1_FB25                      (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F11R1_FB26                      (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F11R1_FB27                      (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F11R1_FB28                      (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F11R1_FB29                      (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F11R1_FB30                      (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F11R1_FB31                      (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
#define  CAN_F12R1_FB0                       (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F12R1_FB1                       (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F12R1_FB2                       (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F12R1_FB3                       (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F12R1_FB4                       (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F12R1_FB5                       (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F12R1_FB6                       (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F12R1_FB7                       (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F12R1_FB8                       (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F12R1_FB9                       (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F12R1_FB10                      (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F12R1_FB11                      (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F12R1_FB12                      (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F12R1_FB13                      (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F12R1_FB14                      (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F12R1_FB15                      (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F12R1_FB16                      (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F12R1_FB17                      (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F12R1_FB18                      (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F12R1_FB19                      (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F12R1_FB20                      (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F12R1_FB21                      (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F12R1_FB22                      (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F12R1_FB23                      (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F12R1_FB24                      (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F12R1_FB25                      (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F12R1_FB26                      (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F12R1_FB27                      (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F12R1_FB28                      (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F12R1_FB29                      (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F12R1_FB30                      (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F12R1_FB31                      (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
#define  CAN_F13R1_FB0                       (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F13R1_FB1                       (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F13R1_FB2                       (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F13R1_FB3                       (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F13R1_FB4                       (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F13R1_FB5                       (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F13R1_FB6                       (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F13R1_FB7                       (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F13R1_FB8                       (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F13R1_FB9                       (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F13R1_FB10                      (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F13R1_FB11                      (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F13R1_FB12                      (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F13R1_FB13                      (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F13R1_FB14                      (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F13R1_FB15                      (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F13R1_FB16                      (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F13R1_FB17                      (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F13R1_FB18                      (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F13R1_FB19                      (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F13R1_FB20                      (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F13R1_FB21                      (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F13R1_FB22                      (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F13R1_FB23                      (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F13R1_FB24                      (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F13R1_FB25                      (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F13R1_FB26                      (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F13R1_FB27                      (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F13R1_FB28                      (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F13R1_FB29                      (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F13R1_FB30                      (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F13R1_FB31                      (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
#define  CAN_F0R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F0R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F0R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F0R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F0R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F0R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F0R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F0R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F0R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F0R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F0R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F0R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F0R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F0R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F0R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F0R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F0R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F0R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F0R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F0R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F0R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F0R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F0R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F0R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F0R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F0R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F0R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F0R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F0R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F0R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F0R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F0R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
#define  CAN_F1R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F1R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F1R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F1R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F1R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F1R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F1R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F1R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F1R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F1R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F1R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F1R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F1R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F1R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F1R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F1R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F1R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F1R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F1R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F1R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F1R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F1R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F1R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F1R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F1R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F1R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F1R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F1R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F1R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F1R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F1R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F1R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
#define  CAN_F2R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F2R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F2R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F2R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F2R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F2R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F2R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F2R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F2R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F2R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F2R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F2R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F2R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F2R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F2R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F2R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F2R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F2R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F2R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F2R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F2R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F2R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F2R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F2R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F2R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F2R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F2R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F2R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F2R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F2R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F2R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F2R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
#define  CAN_F3R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F3R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F3R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F3R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F3R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F3R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F3R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F3R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F3R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F3R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F3R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F3R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F3R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F3R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F3R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F3R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F3R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F3R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F3R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F3R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F3R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F3R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F3R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F3R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F3R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F3R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F3R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F3R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F3R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F3R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F3R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F3R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
#define  CAN_F4R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F4R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F4R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F4R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F4R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F4R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F4R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F4R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F4R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F4R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F4R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F4R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F4R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F4R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F4R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F4R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F4R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F4R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F4R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F4R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F4R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F4R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F4R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F4R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F4R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F4R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F4R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F4R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F4R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F4R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F4R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F4R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
#define  CAN_F5R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F5R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F5R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F5R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F5R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F5R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F5R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F5R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F5R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F5R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F5R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F5R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F5R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F5R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F5R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F5R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F5R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F5R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F5R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F5R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F5R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F5R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F5R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F5R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F5R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F5R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F5R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F5R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F5R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F5R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F5R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F5R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
#define  CAN_F6R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F6R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F6R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F6R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F6R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F6R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F6R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F6R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F6R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F6R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F6R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F6R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F6R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F6R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F6R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F6R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F6R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F6R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F6R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F6R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F6R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F6R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F6R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F6R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F6R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F6R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F6R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F6R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F6R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F6R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F6R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F6R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
#define  CAN_F7R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F7R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F7R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F7R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F7R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F7R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F7R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F7R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F7R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F7R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F7R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F7R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F7R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F7R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F7R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F7R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F7R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F7R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F7R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F7R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F7R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F7R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F7R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F7R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F7R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F7R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F7R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F7R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F7R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F7R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F7R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F7R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
#define  CAN_F8R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F8R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F8R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F8R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F8R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F8R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F8R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F8R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F8R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F8R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F8R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F8R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F8R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F8R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F8R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F8R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F8R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F8R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F8R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F8R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F8R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F8R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F8R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F8R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F8R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F8R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F8R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F8R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F8R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F8R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F8R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F8R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
#define  CAN_F9R2_FB0                        (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F9R2_FB1                        (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F9R2_FB2                        (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F9R2_FB3                        (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F9R2_FB4                        (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F9R2_FB5                        (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F9R2_FB6                        (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F9R2_FB7                        (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F9R2_FB8                        (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F9R2_FB9                        (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F9R2_FB10                       (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F9R2_FB11                       (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F9R2_FB12                       (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F9R2_FB13                       (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F9R2_FB14                       (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F9R2_FB15                       (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F9R2_FB16                       (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F9R2_FB17                       (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F9R2_FB18                       (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F9R2_FB19                       (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F9R2_FB20                       (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F9R2_FB21                       (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F9R2_FB22                       (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F9R2_FB23                       (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F9R2_FB24                       (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F9R2_FB25                       (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F9R2_FB26                       (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F9R2_FB27                       (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F9R2_FB28                       (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F9R2_FB29                       (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F9R2_FB30                       (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F9R2_FB31                       (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
#define  CAN_F10R2_FB0                       (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F10R2_FB1                       (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F10R2_FB2                       (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F10R2_FB3                       (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F10R2_FB4                       (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F10R2_FB5                       (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F10R2_FB6                       (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F10R2_FB7                       (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F10R2_FB8                       (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F10R2_FB9                       (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F10R2_FB10                      (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F10R2_FB11                      (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F10R2_FB12                      (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F10R2_FB13                      (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F10R2_FB14                      (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F10R2_FB15                      (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F10R2_FB16                      (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F10R2_FB17                      (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F10R2_FB18                      (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F10R2_FB19                      (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F10R2_FB20                      (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F10R2_FB21                      (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F10R2_FB22                      (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F10R2_FB23                      (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F10R2_FB24                      (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F10R2_FB25                      (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F10R2_FB26                      (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F10R2_FB27                      (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F10R2_FB28                      (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F10R2_FB29                      (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F10R2_FB30                      (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F10R2_FB31                      (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
#define  CAN_F11R2_FB0                       (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F11R2_FB1                       (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F11R2_FB2                       (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F11R2_FB3                       (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F11R2_FB4                       (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F11R2_FB5                       (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F11R2_FB6                       (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F11R2_FB7                       (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F11R2_FB8                       (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F11R2_FB9                       (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F11R2_FB10                      (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F11R2_FB11                      (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F11R2_FB12                      (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F11R2_FB13                      (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F11R2_FB14                      (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F11R2_FB15                      (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F11R2_FB16                      (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F11R2_FB17                      (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F11R2_FB18                      (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F11R2_FB19                      (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F11R2_FB20                      (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F11R2_FB21                      (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F11R2_FB22                      (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F11R2_FB23                      (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F11R2_FB24                      (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F11R2_FB25                      (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F11R2_FB26                      (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F11R2_FB27                      (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F11R2_FB28                      (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F11R2_FB29                      (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F11R2_FB30                      (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F11R2_FB31                      (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
#define  CAN_F12R2_FB0                       (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F12R2_FB1                       (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F12R2_FB2                       (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F12R2_FB3                       (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F12R2_FB4                       (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F12R2_FB5                       (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F12R2_FB6                       (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F12R2_FB7                       (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F12R2_FB8                       (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F12R2_FB9                       (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F12R2_FB10                      (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F12R2_FB11                      (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F12R2_FB12                      (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F12R2_FB13                      (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F12R2_FB14                      (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F12R2_FB15                      (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F12R2_FB16                      (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F12R2_FB17                      (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F12R2_FB18                      (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F12R2_FB19                      (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F12R2_FB20                      (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F12R2_FB21                      (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F12R2_FB22                      (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F12R2_FB23                      (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F12R2_FB24                      (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F12R2_FB25                      (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F12R2_FB26                      (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F12R2_FB27                      (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F12R2_FB28                      (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F12R2_FB29                      (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F12R2_FB30                      (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F12R2_FB31                      (0x80000000UL)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
#define  CAN_F13R2_FB0                       (0x00000001UL)        /*!<Filter bit 0 */
#define  CAN_F13R2_FB1                       (0x00000002UL)        /*!<Filter bit 1 */
#define  CAN_F13R2_FB2                       (0x00000004UL)        /*!<Filter bit 2 */
#define  CAN_F13R2_FB3                       (0x00000008UL)        /*!<Filter bit 3 */
#define  CAN_F13R2_FB4                       (0x00000010UL)        /*!<Filter bit 4 */
#define  CAN_F13R2_FB5                       (0x00000020UL)        /*!<Filter bit 5 */
#define  CAN_F13R2_FB6                       (0x00000040UL)        /*!<Filter bit 6 */
#define  CAN_F13R2_FB7                       (0x00000080UL)        /*!<Filter bit 7 */
#define  CAN_F13R2_FB8                       (0x00000100UL)        /*!<Filter bit 8 */
#define  CAN_F13R2_FB9                       (0x00000200UL)        /*!<Filter bit 9 */
#define  CAN_F13R2_FB10                      (0x00000400UL)        /*!<Filter bit 10 */
#define  CAN_F13R2_FB11                      (0x00000800UL)        /*!<Filter bit 11 */
#define  CAN_F13R2_FB12                      (0x00001000UL)        /*!<Filter bit 12 */
#define  CAN_F13R2_FB13                      (0x00002000UL)        /*!<Filter bit 13 */
#define  CAN_F13R2_FB14                      (0x00004000UL)        /*!<Filter bit 14 */
#define  CAN_F13R2_FB15                      (0x00008000UL)        /*!<Filter bit 15 */
#define  CAN_F13R2_FB16                      (0x00010000UL)        /*!<Filter bit 16 */
#define  CAN_F13R2_FB17                      (0x00020000UL)        /*!<Filter bit 17 */
#define  CAN_F13R2_FB18                      (0x00040000UL)        /*!<Filter bit 18 */
#define  CAN_F13R2_FB19                      (0x00080000UL)        /*!<Filter bit 19 */
#define  CAN_F13R2_FB20                      (0x00100000UL)        /*!<Filter bit 20 */
#define  CAN_F13R2_FB21                      (0x00200000UL)        /*!<Filter bit 21 */
#define  CAN_F13R2_FB22                      (0x00400000UL)        /*!<Filter bit 22 */
#define  CAN_F13R2_FB23                      (0x00800000UL)        /*!<Filter bit 23 */
#define  CAN_F13R2_FB24                      (0x01000000UL)        /*!<Filter bit 24 */
#define  CAN_F13R2_FB25                      (0x02000000UL)        /*!<Filter bit 25 */
#define  CAN_F13R2_FB26                      (0x04000000UL)        /*!<Filter bit 26 */
#define  CAN_F13R2_FB27                      (0x08000000UL)        /*!<Filter bit 27 */
#define  CAN_F13R2_FB28                      (0x10000000UL)        /*!<Filter bit 28 */
#define  CAN_F13R2_FB29                      (0x20000000UL)        /*!<Filter bit 29 */
#define  CAN_F13R2_FB30                      (0x40000000UL)        /*!<Filter bit 30 */
#define  CAN_F13R2_FB31                      (0x80000000UL)        /*!<Filter bit 31 */

#if defined(STM32F446xx)
/******************************************************************************/
/*                                                                            */
/*                          HDMI-CEC (CEC)                                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for CEC_CR register  *********************/
#define  CEC_CR_CECEN                        (0x00000001UL)       /*!< CEC Enable                              */
#define  CEC_CR_TXSOM                        (0x00000002UL)       /*!< CEC Tx Start Of Message                 */
#define  CEC_CR_TXEOM                        (0x00000004UL)       /*!< CEC Tx End Of Message                   */

/*******************  Bit definition for CEC_CFGR register  *******************/
#define  CEC_CFGR_SFT                        (0x00000007UL)       /*!< CEC Signal Free Time                    */
#define  CEC_CFGR_RXTOL                      (0x00000008UL)       /*!< CEC Tolerance                           */
#define  CEC_CFGR_BRESTP                     (0x00000010UL)       /*!< CEC Rx Stop                             */
#define  CEC_CFGR_BREGEN                     (0x00000020UL)       /*!< CEC Bit Rising Error generation         */
#define  CEC_CFGR_LREGEN                     (0x00000040UL)       /*!< CEC Long Period Error generation        */
#define  CEC_CFGR_SFTOPT                     (0x00000100UL)       /*!< CEC Signal Free Time optional           */
#define  CEC_CFGR_BRDNOGEN                   (0x00000080UL)       /*!< CEC Broadcast No error generation       */
#define  CEC_CFGR_OAR                        (0x7FFF0000UL)       /*!< CEC Own Address                         */
#define  CEC_CFGR_LSTN                       (0x80000000UL)       /*!< CEC Listen mode                         */

/*******************  Bit definition for CEC_TXDR register  *******************/
#define  CEC_TXDR_TXD                        (0x000000FFUL)       /*!< CEC Tx Data                              */

/*******************  Bit definition for CEC_RXDR register  *******************/
#define  CEC_TXDR_RXD                        (0x000000FFUL)       /*!< CEC Rx Data                              */

/*******************  Bit definition for CEC_ISR register  ********************/
#define  CEC_ISR_RXBR                        (0x00000001UL)       /*!< CEC Rx-Byte Received                      */
#define  CEC_ISR_RXEND                       (0x00000002UL)       /*!< CEC End Of Reception                      */
#define  CEC_ISR_RXOVR                       (0x00000004UL)       /*!< CEC Rx-Overrun                            */
#define  CEC_ISR_BRE                         (0x00000008UL)       /*!< CEC Rx Bit Rising Error                   */
#define  CEC_ISR_SBPE                        (0x00000010UL)       /*!< CEC Rx Short Bit period Error             */
#define  CEC_ISR_LBPE                        (0x00000020UL)       /*!< CEC Rx Long Bit period Error              */
#define  CEC_ISR_RXACKE                      (0x00000040UL)       /*!< CEC Rx Missing Acknowledge                */
#define  CEC_ISR_ARBLST                      (0x00000080UL)       /*!< CEC Arbitration Lost                      */
#define  CEC_ISR_TXBR                        (0x00000100UL)       /*!< CEC Tx Byte Request                       */
#define  CEC_ISR_TXEND                       (0x00000200UL)       /*!< CEC End of Transmission                   */
#define  CEC_ISR_TXUDR                       (0x00000400UL)       /*!< CEC Tx-Buffer Underrun                    */
#define  CEC_ISR_TXERR                       (0x00000800UL)       /*!< CEC Tx-Error                              */
#define  CEC_ISR_TXACKE                      (0x00001000UL)       /*!< CEC Tx Missing Acknowledge                */

/*******************  Bit definition for CEC_IER register  ********************/
#define  CEC_IER_RXBRIE                      (0x00000001UL)       /*!< CEC Rx-Byte Received IT Enable            */
#define  CEC_IER_RXENDIE                     (0x00000002UL)       /*!< CEC End Of Reception IT Enable            */
#define  CEC_IER_RXOVRIE                     (0x00000004UL)       /*!< CEC Rx-Overrun IT Enable                  */
#define  CEC_IER_BREIEIE                     (0x00000008UL)       /*!< CEC Rx Bit Rising Error IT Enable         */
#define  CEC_IER_SBPEIE                      (0x00000010UL)       /*!< CEC Rx Short Bit period Error IT Enable   */
#define  CEC_IER_LBPEIE                      (0x00000020UL)       /*!< CEC Rx Long Bit period Error IT Enable    */
#define  CEC_IER_RXACKEIE                    (0x00000040UL)       /*!< CEC Rx Missing Acknowledge IT Enable      */
#define  CEC_IER_ARBLSTIE                    (0x00000080UL)       /*!< CEC Arbitration Lost IT Enable            */
#define  CEC_IER_TXBRIE                      (0x00000100UL)       /*!< CEC Tx Byte Request  IT Enable            */
#define  CEC_IER_TXENDIE                     (0x00000200UL)       /*!< CEC End of Transmission IT Enable         */
#define  CEC_IER_TXUDRIE                     (0x00000400UL)       /*!< CEC Tx-Buffer Underrun IT Enable          */
#define  CEC_IER_TXERRIE                     (0x00000800UL)       /*!< CEC Tx-Error IT Enable                    */
#define  CEC_IER_TXACKEIE                    (0x00001000UL)       /*!< CEC Tx Missing Acknowledge IT Enable      */
#endif /* STM32F446xx */

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           (0xFFFFFFFFUL) /*!< Data register bits */


/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         (0xFFU)        /*!< General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        (0x01U)        /*!< RESET bit */

/******************************************************************************/
/*                                                                            */
/*                            Crypto Processor                                */
/*                                                                            */
/******************************************************************************/
/******************* Bits definition for CRYP_CR register  ********************/
#define CRYP_CR_ALGODIR                      (0x00000004UL)

#define CRYP_CR_ALGOMODE                     (0x00080038UL)
#define CRYP_CR_ALGOMODE_0                   (0x00000008UL)
#define CRYP_CR_ALGOMODE_1                   (0x00000010UL)
#define CRYP_CR_ALGOMODE_2                   (0x00000020UL)
#define CRYP_CR_ALGOMODE_TDES_ECB            (0x00000000UL)
#define CRYP_CR_ALGOMODE_TDES_CBC            (0x00000008UL)
#define CRYP_CR_ALGOMODE_DES_ECB             (0x00000010UL)
#define CRYP_CR_ALGOMODE_DES_CBC             (0x00000018UL)
#define CRYP_CR_ALGOMODE_AES_ECB             (0x00000020UL)
#define CRYP_CR_ALGOMODE_AES_CBC             (0x00000028UL)
#define CRYP_CR_ALGOMODE_AES_CTR             (0x00000030UL)
#define CRYP_CR_ALGOMODE_AES_KEY             (0x00000038UL)

#define CRYP_CR_DATATYPE                     (0x000000C0UL)
#define CRYP_CR_DATATYPE_0                   (0x00000040UL)
#define CRYP_CR_DATATYPE_1                   (0x00000080UL)
#define CRYP_CR_KEYSIZE                      (0x00000300UL)
#define CRYP_CR_KEYSIZE_0                    (0x00000100UL)
#define CRYP_CR_KEYSIZE_1                    (0x00000200UL)
#define CRYP_CR_FFLUSH                       (0x00004000UL)
#define CRYP_CR_CRYPEN                       (0x00008000UL)

#define CRYP_CR_GCM_CCMPH                    (0x00030000UL)
#define CRYP_CR_GCM_CCMPH_0                  (0x00010000UL)
#define CRYP_CR_GCM_CCMPH_1                  (0x00020000UL)
#define CRYP_CR_ALGOMODE_3                   (0x00080000UL)

/****************** Bits definition for CRYP_SR register  *********************/
#define CRYP_SR_IFEM                         (0x00000001UL)
#define CRYP_SR_IFNF                         (0x00000002UL)
#define CRYP_SR_OFNE                         (0x00000004UL)
#define CRYP_SR_OFFU                         (0x00000008UL)
#define CRYP_SR_BUSY                         (0x00000010UL)
/****************** Bits definition for CRYP_DMACR register  ******************/
#define CRYP_DMACR_DIEN                      (0x00000001UL)
#define CRYP_DMACR_DOEN                      (0x00000002UL)
/*****************  Bits definition for CRYP_IMSCR register  ******************/
#define CRYP_IMSCR_INIM                      (0x00000001UL)
#define CRYP_IMSCR_OUTIM                     (0x00000002UL)
/****************** Bits definition for CRYP_RISR register  *******************/
#define CRYP_RISR_OUTRIS                     (0x00000001UL)
#define CRYP_RISR_INRIS                      (0x00000002UL)
/****************** Bits definition for CRYP_MISR register  *******************/
#define CRYP_MISR_INMIS                      (0x00000001UL)
#define CRYP_MISR_OUTMIS                     (0x00000002UL)

/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          (0x00000001UL)        /*!<DAC channel1 enable */
#define  DAC_CR_BOFF1                        (0x00000002UL)        /*!<DAC channel1 output buffer disable */
#define  DAC_CR_TEN1                         (0x00000004UL)        /*!<DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        (0x00000038UL)        /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      (0x00000008UL)        /*!<Bit 0 */
#define  DAC_CR_TSEL1_1                      (0x00000010UL)        /*!<Bit 1 */
#define  DAC_CR_TSEL1_2                      (0x00000020UL)        /*!<Bit 2 */

#define  DAC_CR_WAVE1                        (0x000000C0UL)        /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      (0x00000040UL)        /*!<Bit 0 */
#define  DAC_CR_WAVE1_1                      (0x00000080UL)        /*!<Bit 1 */

#define  DAC_CR_MAMP1                        (0x00000F00UL)        /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      (0x00000100UL)        /*!<Bit 0 */
#define  DAC_CR_MAMP1_1                      (0x00000200UL)        /*!<Bit 1 */
#define  DAC_CR_MAMP1_2                      (0x00000400UL)        /*!<Bit 2 */
#define  DAC_CR_MAMP1_3                      (0x00000800UL)        /*!<Bit 3 */

#define  DAC_CR_DMAEN1                       (0x00001000UL)        /*!<DAC channel1 DMA enable */
#define  DAC_CR_DMAUDRIE1                    (0x00002000UL)        /*!<DAC channel1 DMA underrun interrupt enable*/
#define  DAC_CR_EN2                          (0x00010000UL)        /*!<DAC channel2 enable */
#define  DAC_CR_BOFF2                        (0x00020000UL)        /*!<DAC channel2 output buffer disable */
#define  DAC_CR_TEN2                         (0x00040000UL)        /*!<DAC channel2 Trigger enable */

#define  DAC_CR_TSEL2                        (0x00380000UL)        /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CR_TSEL2_0                      (0x00080000UL)        /*!<Bit 0 */
#define  DAC_CR_TSEL2_1                      (0x00100000UL)        /*!<Bit 1 */
#define  DAC_CR_TSEL2_2                      (0x00200000UL)        /*!<Bit 2 */

#define  DAC_CR_WAVE2                        (0x00C00000UL)        /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE2_0                      (0x00400000UL)        /*!<Bit 0 */
#define  DAC_CR_WAVE2_1                      (0x00800000UL)        /*!<Bit 1 */

#define  DAC_CR_MAMP2                        (0x0F000000UL)        /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CR_MAMP2_0                      (0x01000000UL)        /*!<Bit 0 */
#define  DAC_CR_MAMP2_1                      (0x02000000UL)        /*!<Bit 1 */
#define  DAC_CR_MAMP2_2                      (0x04000000UL)        /*!<Bit 2 */
#define  DAC_CR_MAMP2_3                      (0x08000000UL)        /*!<Bit 3 */

#define  DAC_CR_DMAEN2                       (0x10000000UL)        /*!<DAC channel2 DMA enabled */
#define  DAC_CR_DMAUDRIE2                    ((uint32_t)0x20000000U)        /*!<DAC channel2 DMA underrun interrupt enable*/

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 (0x01U)               /*!<DAC channel1 software trigger */
#define  DAC_SWTRIGR_SWTRIG2                 (0x02U)               /*!<DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                (0x0FFFU)            /*!<DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                (0xFFF0U)            /*!<DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 (0xFFU)               /*!<DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define  DAC_DHR12R2_DACC2DHR                (0x0FFFU)            /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define  DAC_DHR12L2_DACC2DHR                (0xFFF0U)            /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define  DAC_DHR8R2_DACC2DHR                 (0xFFU)               /*!<DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                (0x00000FFFUL)        /*!<DAC channel1 12-bit Right aligned data */
#define  DAC_DHR12RD_DACC2DHR                (0x0FFF0000UL)        /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                (0x0000FFF0UL)        /*!<DAC channel1 12-bit Left aligned data */
#define  DAC_DHR12LD_DACC2DHR                (0xFFF00000UL)        /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 (0x00FFU)            /*!<DAC channel1 8-bit Right aligned data */
#define  DAC_DHR8RD_DACC2DHR                 (0xFF00U)            /*!<DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   (0x0FFFU)            /*!<DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DOR2_DACC2DOR                   (0x0FFFU)            /*!<DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define  DAC_SR_DMAUDR1                      (0x00002000UL)        /*!<DAC channel1 DMA underrun flag */
#define  DAC_SR_DMAUDR2                      (0x20000000UL)        /*!<DAC channel2 DMA underrun flag */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                                    DCMI                                    */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DCMI_CR register  ******************/
#define DCMI_CR_CAPTURE                      (0x00000001UL)
#define DCMI_CR_CM                           (0x00000002UL)
#define DCMI_CR_CROP                         (0x00000004UL)
#define DCMI_CR_JPEG                         (0x00000008UL)
#define DCMI_CR_ESS                          (0x00000010UL)
#define DCMI_CR_PCKPOL                       (0x00000020UL)
#define DCMI_CR_HSPOL                        (0x00000040UL)
#define DCMI_CR_VSPOL                        (0x00000080UL)
#define DCMI_CR_FCRC_0                       (0x00000100UL)
#define DCMI_CR_FCRC_1                       (0x00000200UL)
#define DCMI_CR_EDM_0                        (0x00000400UL)
#define DCMI_CR_EDM_1                        (0x00000800UL)
#define DCMI_CR_CRE                          (0x00001000UL)
#define DCMI_CR_ENABLE                       (0x00004000UL)

/********************  Bits definition for DCMI_SR register  ******************/
#define DCMI_SR_HSYNC                        (0x00000001UL)
#define DCMI_SR_VSYNC                        (0x00000002UL)
#define DCMI_SR_FNE                          (0x00000004UL)

/********************  Bits definition for DCMI_RIS register  *****************/
#define DCMI_RIS_FRAME_RIS                   (0x00000001UL)
#define DCMI_RIS_OVR_RIS                     (0x00000002UL)
#define DCMI_RIS_ERR_RIS                     (0x00000004UL)
#define DCMI_RIS_VSYNC_RIS                   (0x00000008UL)
#define DCMI_RIS_LINE_RIS                     (0x00000010UL)
/* Legacy defines */
#define DCMI_RISR_FRAME_RIS                  DCMI_RIS_FRAME_RIS
#define DCMI_RISR_OVR_RIS                    DCMI_RIS_OVR_RIS
#define DCMI_RISR_ERR_RIS                    DCMI_RIS_ERR_RIS
#define DCMI_RISR_VSYNC_RIS                  DCMI_RIS_VSYNC_RIS
#define DCMI_RISR_LINE_RIS                   DCMI_RIS_LINE_RIS
#define DCMI_RISR_OVF_RIS                    DCMI_RIS_OVR_RIS

/********************  Bits definition for DCMI_IER register  *****************/
#define DCMI_IER_FRAME_IE                    (0x00000001UL)
#define DCMI_IER_OVR_IE                      (0x00000002UL)
#define DCMI_IER_ERR_IE                      (0x00000004UL)
#define DCMI_IER_VSYNC_IE                    (0x00000008UL)
#define DCMI_IER_LINE_IE                     (0x00000010UL)

/* Legacy defines */
#define DCMI_IER_OVF_IE                      DCMI_IER_OVR_IE

/********************  Bits definition for DCMI_MIS register  ****************/
#define DCMI_MIS_FRAME_MIS                   (0x00000001UL)
#define DCMI_MIS_OVR_MIS                     (0x00000002UL)
#define DCMI_MIS_ERR_MIS                     (0x00000004UL)
#define DCMI_MIS_VSYNC_MIS                   (0x00000008UL)
#define DCMI_MIS_LINE_MIS                    (0x00000010UL)

/* Legacy defines */
#define DCMI_MISR_FRAME_MIS                  DCMI_MIS_FRAME_MIS
#define DCMI_MISR_OVF_MIS                    DCMI_MIS_OVR_MIS
#define DCMI_MISR_ERR_MIS                    DCMI_MIS_ERR_MIS
#define DCMI_MISR_VSYNC_MIS                  DCMI_MIS_VSYNC_MIS
#define DCMI_MISR_LINE_MIS                   DCMI_MIS_LINE_MIS

/********************  Bits definition for DCMI_ICR register  *****************/
#define DCMI_ICR_FRAME_ISC                   (0x00000001UL)
#define DCMI_ICR_OVR_ISC                     (0x00000002UL)
#define DCMI_ICR_ERR_ISC                     (0x00000004UL)
#define DCMI_ICR_VSYNC_ISC                   (0x00000008UL)
#define DCMI_ICR_LINE_ISC                    (0x00000010UL)

/* Legacy defines */
#define DCMI_ICR_OVF_ISC                     DCMI_ICR_OVR_ISC

/********************  Bits definition for DCMI_ESCR register  ******************/
#define DCMI_ESCR_FSC                        (0x000000FFUL)
#define DCMI_ESCR_LSC                        (0x0000FF00UL)
#define DCMI_ESCR_LEC                        (0x00FF0000UL)
#define DCMI_ESCR_FEC                        (0xFF000000UL)

/********************  Bits definition for DCMI_ESUR register  ******************/
#define DCMI_ESUR_FSU                        (0x000000FFUL)
#define DCMI_ESUR_LSU                        (0x0000FF00UL)
#define DCMI_ESUR_LEU                        (0x00FF0000UL)
#define DCMI_ESUR_FEU                        (0xFF000000UL)

/********************  Bits definition for DCMI_CWSTRT register  ******************/
#define DCMI_CWSTRT_HOFFCNT                  (0x00003FFFUL)
#define DCMI_CWSTRT_VST                      (0x1FFF0000UL)

/********************  Bits definition for DCMI_CWSIZE register  ******************/
#define DCMI_CWSIZE_CAPCNT                   (0x00003FFFUL)
#define DCMI_CWSIZE_VLINE                    (0x3FFF0000UL)

/********************  Bits definition for DCMI_DR register  ******************/
#define DCMI_DR_BYTE0                        (0x000000FFUL)
#define DCMI_DR_BYTE1                        (0x0000FF00UL)
#define DCMI_DR_BYTE2                        (0x00FF0000UL)
#define DCMI_DR_BYTE3                        (0xFF000000UL)

/******************************************************************************/
/*                                                                            */
/*                 Digital Filter for Sigma Delta Modulators                  */
/*                                                                            */
/******************************************************************************/

/****************   DFSDM channel configuration registers  ********************/

/***************  Bit definition for DFSDM_CHCFGR1 register  ******************/
#define  DFSDM_CHCFGR1_DFSDMEN                (0x80000000UL)            /*!< Global enable for DFSDM interface */
#define  DFSDM_CHCFGR1_CKOUTSRC               (0x40000000UL)            /*!< Output serial clock source selection */
#define  DFSDM_CHCFGR1_CKOUTDIV               (0x00FF0000UL)            /*!< CKOUTDIV[7:0] output serial clock divider */
#define  DFSDM_CHCFGR1_DATPACK                (0x0000C000UL)            /*!< DATPACK[1:0] Data packing mode */
#define  DFSDM_CHCFGR1_DATPACK_1              (0x00008000UL)            /*!< Data packing mode, Bit 1 */
#define  DFSDM_CHCFGR1_DATPACK_0              (0x00004000UL)            /*!< Data packing mode, Bit 0 */
#define  DFSDM_CHCFGR1_DATMPX                 (0x00003000UL)            /*!< DATMPX[1:0] Input data multiplexer for channel y */
#define  DFSDM_CHCFGR1_DATMPX_1               (0x00002000UL)            /*!< Input data multiplexer for channel y, Bit 1 */
#define  DFSDM_CHCFGR1_DATMPX_0               (0x00001000UL)            /*!< Input data multiplexer for channel y, Bit 0 */
#define  DFSDM_CHCFGR1_CHINSEL                (0x00000100UL)            /*!< Serial inputs selection for channel y */
#define  DFSDM_CHCFGR1_CHEN                   (0x00000080UL)            /*!< Channel y enable */
#define  DFSDM_CHCFGR1_CKABEN                 (0x00000040UL)            /*!< Clock absence detector enable on channel y */
#define  DFSDM_CHCFGR1_SCDEN                  (0x00000020UL)            /*!< Short circuit detector enable on channel y */
#define  DFSDM_CHCFGR1_SPICKSEL               (0x0000000CUL)            /*!< SPICKSEL[1:0] SPI clock select for channel y */
#define  DFSDM_CHCFGR1_SPICKSEL_1             (0x00000008UL)            /*!< SPI clock select for channel y, Bit 1 */
#define  DFSDM_CHCFGR1_SPICKSEL_0             (0x00000004UL)            /*!< SPI clock select for channel y, Bit 0 */
#define  DFSDM_CHCFGR1_SITP                   (0x00000003UL)            /*!< SITP[1:0] Serial interface type for channel y */
#define  DFSDM_CHCFGR1_SITP_1                 (0x00000002UL)            /*!< Serial interface type for channel y, Bit 1 */
#define  DFSDM_CHCFGR1_SITP_0                 (0x00000001UL)            /*!< Serial interface type for channel y, Bit 0 */

/***************  Bit definition for DFSDM_CHCFGR2 register  ******************/
#define  DFSDM_CHCFGR2_OFFSET                 (0xFFFFFF00UL)            /*!< OFFSET[23:0] 24-bit calibration offset for channel y */
#define  DFSDM_CHCFGR2_DTRBS                  (0x000000F8UL)            /*!< DTRBS[4:0] Data right bit-shift for channel y */

/******************  Bit definition for DFSDM_CHAWSCDR register ***************/
#define  DFSDM_CHAWSCDR_AWFORD                (0x00C00000UL)            /*!< AWFORD[1:0] Analog watchdog Sinc filter order on channel y */
#define  DFSDM_CHAWSCDR_AWFORD_1              (0x00800000UL)            /*!< Analog watchdog Sinc filter order on channel y, Bit 1 */
#define  DFSDM_CHAWSCDR_AWFORD_0              (0x00400000UL)            /*!< Analog watchdog Sinc filter order on channel y, Bit 0 */
#define  DFSDM_CHAWSCDR_AWFOSR                (0x001F0000UL)            /*!< AWFOSR[4:0] Analog watchdog filter oversampling ratio on channel y */
#define  DFSDM_CHAWSCDR_BKSCD                 (0x0000F000UL)            /*!< BKSCD[3:0] Break signal assignment for short circuit detector on channel y */
#define  DFSDM_CHAWSCDR_SCDT                  (0x000000FFUL)            /*!< SCDT[7:0] Short circuit detector threshold for channel y */

/****************  Bit definition for DFSDM_CHWDATR register *******************/
#define  DFSDM_CHWDATR_WDATA                  (0x0000FFFFUL)            /*!< WDATA[15:0] Input channel y watchdog data */

/****************  Bit definition for DFSDM_CHDATINR register *****************/
#define  DFSDM_CHDATINR_INDAT0                (0x0000FFFFUL)            /*!< INDAT0[31:16] Input data for channel y or channel (y+1) */
#define  DFSDM_CHDATINR_INDAT1                (0xFFFF0000UL)            /*!< INDAT0[15:0] Input data for channel y */

/************************   DFSDM module registers  ****************************/

/*****************  Bit definition for DFSDM_FLTCR1 register *******************/
#define  DFSDM_FLTCR1_AWFSEL                  (0x40000000UL)            /*!< Analog watchdog fast mode select */
#define  DFSDM_FLTCR1_FAST                    (0x20000000UL)            /*!< Fast conversion mode selection */
#define  DFSDM_FLTCR1_RCH                     (0x07000000UL)            /*!< RCH[2:0] Regular channel selection */
#define  DFSDM_FLTCR1_RDMAEN                  (0x00200000UL)            /*!< DMA channel enabled to read data for the regular conversion */
#define  DFSDM_FLTCR1_RSYNC                   (0x00080000UL)            /*!< Launch regular conversion synchronously with DFSDMx */
#define  DFSDM_FLTCR1_RCONT                   (0x00040000UL)            /*!< Continuous mode selection for regular conversions */
#define  DFSDM_FLTCR1_RSWSTART                (0x00020000UL)            /*!< Software start of a conversion on the regular channel */
#define  DFSDM_FLTCR1_JEXTEN                  (0x00006000UL)            /*!< JEXTEN[1:0] Trigger enable and trigger edge selection for injected conversions */
#define  DFSDM_FLTCR1_JEXTEN_1                (0x00004000UL)            /*!< Trigger enable and trigger edge selection for injected conversions, Bit 1 */
#define  DFSDM_FLTCR1_JEXTEN_0                (0x00002000UL)            /*!< Trigger enable and trigger edge selection for injected conversions, Bit 0 */
#define  DFSDM_FLTCR1_JEXTSEL                 (0x00000700UL)            /*!< JEXTSEL[2:0]Trigger signal selection for launching injected conversions */
#define  DFSDM_FLTCR1_JEXTSEL_2               (0x00000400UL)            /*!< Trigger signal selection for launching injected conversions, Bit 2 */
#define  DFSDM_FLTCR1_JEXTSEL_1               (0x00000200UL)            /*!< Trigger signal selection for launching injected conversions, Bit 1 */
#define  DFSDM_FLTCR1_JEXTSEL_0               (0x00000100UL)            /*!< Trigger signal selection for launching injected conversions, Bit 0 */
#define  DFSDM_FLTCR1_JDMAEN                  (0x00000020UL)            /*!< DMA channel enabled to read data for the injected channel group */
#define  DFSDM_FLTCR1_JSCAN                   (0x00000010UL)            /*!< Scanning conversion in continuous mode selection for injected conversions */
#define  DFSDM_FLTCR1_JSYNC                   (0x00000008UL)            /*!< Launch an injected conversion synchronously with DFSDMx JSWSTART trigger  */
#define  DFSDM_FLTCR1_JSWSTART                (0x00000002UL)            /*!< Start the conversion of the injected group of channels */
#define  DFSDM_FLTCR1_DFEN                    (0x00000001UL)            /*!< DFSDM enable */

/********************  Bit definition for DFSDM_FLTCR2 register ***************/
#define  DFSDM_FLTCR2_AWDCH                   (0x000F0000UL)            /*!< AWDCH[7:0] Analog watchdog channel selection */
#define  DFSDM_FLTCR2_EXCH                    (0x00000F00UL)            /*!< EXCH[7:0] Extreme detector channel selection */
#define  DFSDM_FLTCR2_CKABIE                  (0x00000040UL)            /*!< Clock absence interrupt enable */
#define  DFSDM_FLTCR2_SCDIE                   (0x00000020UL)            /*!< Short circuit detector interrupt enable */
#define  DFSDM_FLTCR2_AWDIE                   (0x00000010UL)            /*!< Analog watchdog interrupt enable */
#define  DFSDM_FLTCR2_ROVRIE                  (0x00000008UL)            /*!< Regular data overrun interrupt enable */
#define  DFSDM_FLTCR2_JOVRIE                  (0x00000004UL)            /*!< Injected data overrun interrupt enable */
#define  DFSDM_FLTCR2_REOCIE                  (0x00000002UL)            /*!< Regular end of conversion interrupt enable */
#define  DFSDM_FLTCR2_JEOCIE                  (0x00000001UL)            /*!< Injected end of conversion interrupt enable */

/*****************  Bit definition for DFSDM_FLTISR register *******************/
#define  DFSDM_FLTISR_SCDF                    (0x0F000000UL)            /*!< SCDF[7:0] Short circuit detector flag */
#define  DFSDM_FLTISR_CKABF                   (0x000F0000UL)            /*!< CKABF[7:0] Clock absence flag */
#define  DFSDM_FLTISR_RCIP                    (0x00004000UL)            /*!< Regular conversion in progress status */
#define  DFSDM_FLTISR_JCIP                    (0x00002000UL)            /*!< Injected conversion in progress status */
#define  DFSDM_FLTISR_AWDF                    (0x00000010UL)            /*!< Analog watchdog */
#define  DFSDM_FLTISR_ROVRF                   (0x00000008UL)            /*!< Regular conversion overrun flag */
#define  DFSDM_FLTISR_JOVRF                   (0x00000004UL)            /*!< Injected conversion overrun flag */
#define  DFSDM_FLTISR_REOCF                   (0x00000002UL)            /*!< End of regular conversion flag */
#define  DFSDM_FLTISR_JEOCF                   (0x00000001UL)            /*!< End of injected conversion flag */

/*****************  Bit definition for DFSDM_FLTICR register *******************/
#define  DFSDM_FLTICR_CLRSCSDF                (0x0F000000UL)            /*!< CLRSCSDF[7:0] Clear the short circuit detector flag */
#define  DFSDM_FLTICR_CLRCKABF                (0x000F0000UL)            /*!< CLRCKABF[7:0] Clear the clock absence flag */
#define  DFSDM_FLTICR_CLRROVRF                (0x00000008UL)            /*!< Clear the regular conversion overrun flag */
#define  DFSDM_FLTICR_CLRJOVRF                (0x00000004UL)            /*!< Clear the injected conversion overrun flag */

/****************  Bit definition for DFSDM_FLTJCHGR register ******************/
#define  DFSDM_FLTJCHGR_JCHG                  (0x000000FFUL)            /*!< JCHG[7:0] Injected channel group selection */

/*****************  Bit definition for DFSDM_FLTFCR register *******************/
#define  DFSDM_FLTFCR_FORD                    (0xE0000000UL)            /*!< FORD[2:0] Sinc filter order */
#define  DFSDM_FLTFCR_FORD_2                  (0x80000000UL)            /*!< Sinc filter order, Bit 2 */
#define  DFSDM_FLTFCR_FORD_1                  (0x40000000UL)            /*!< Sinc filter order, Bit 1 */
#define  DFSDM_FLTFCR_FORD_0                  (0x20000000UL)            /*!< Sinc filter order, Bit 0 */
#define  DFSDM_FLTFCR_FOSR                    (0x03FF0000UL)            /*!< FOSR[9:0] Sinc filter oversampling ratio (decimation rate) */
#define  DFSDM_FLTFCR_IOSR                    (0x000000FFUL)            /*!< IOSR[7:0] Integrator oversampling ratio (averaging length) */

/***************  Bit definition for DFSDM_FLTJDATAR register *****************/
#define  DFSDM_FLTJDATAR_JDATA                (0xFFFFFF00UL)            /*!< JDATA[23:0] Injected group conversion data */
#define  DFSDM_FLTJDATAR_JDATACH              (0x00000007UL)            /*!< JDATACH[2:0] Injected channel most recently converted */

/***************  Bit definition for DFSDM_FLTRDATAR register *****************/
#define  DFSDM_FLTRDATAR_RDATA                (0xFFFFFF00UL)            /*!< RDATA[23:0] Regular channel conversion data */
#define  DFSDM_FLTRDATAR_RPEND                (0x00000010UL)            /*!< RPEND Regular channel pending data */
#define  DFSDM_FLTRDATAR_RDATACH              (0x00000007UL)            /*!< RDATACH[2:0] Regular channel most recently converted */

/***************  Bit definition for DFSDM_FLTAWHTR register ******************/
#define  DFSDM_FLTAWHTR_AWHT                 (0xFFFFFF00UL)             /*!< AWHT[23:0] Analog watchdog high threshold */
#define  DFSDM_FLTAWHTR_BKAWH                (0x0000000FUL)             /*!< BKAWH[3:0] Break signal assignment to analog watchdog high threshold event */

/***************  Bit definition for DFSDM_FLTAWLTR register ******************/
#define  DFSDM_FLTAWLTR_AWLT                 (0xFFFFFF00UL)             /*!< AWLT[23:0] Analog watchdog low threshold */
#define  DFSDM_FLTAWLTR_BKAWL                (0x0000000FUL)             /*!< BKAWL[3:0] Break signal assignment to analog watchdog low threshold event */

/***************  Bit definition for DFSDM_FLTAWSR register *******************/
#define  DFSDM_FLTAWSR_AWHTF                 (0x00000F00UL)             /*!< AWHTF[15:8] Analog watchdog high threshold error on given channels */
#define  DFSDM_FLTAWSR_AWLTF                 (0x0000000FUL)             /*!< AWLTF[7:0] Analog watchdog low threshold error on given channels */

/***************  Bit definition for DFSDM_FLTAWCFR register ******************/
#define  DFSDM_FLTAWCFR_CLRAWHTF             (0x00000F00UL)             /*!< CLRAWHTF[15:8] Clear the Analog watchdog high threshold flag */
#define  DFSDM_FLTAWCFR_CLRAWLTF             (0x0000000FUL)             /*!< CLRAWLTF[7:0] Clear the Analog watchdog low threshold flag */

/***************  Bit definition for DFSDM_FLTEXMAX register ******************/
#define  DFSDM_FLTEXMAX_EXMAX                (0xFFFFFF00UL)             /*!< EXMAX[23:0] Extreme detector maximum value */
#define  DFSDM_FLTEXMAX_EXMAXCH              (0x00000007UL)             /*!< EXMAXCH[2:0] Extreme detector maximum data channel */

/***************  Bit definition for DFSDM_FLTEXMIN register ******************/
#define  DFSDM_FLTEXMIN_EXMIN                (0xFFFFFF00UL)             /*!< EXMIN[23:0] Extreme detector minimum value */
#define  DFSDM_FLTEXMIN_EXMINCH              (0x00000007UL)             /*!< EXMINCH[2:0] Extreme detector minimum data channel */

/***************  Bit definition for DFSDM_FLTCNVTIMR register ****************/
#define  DFSDM_FLTCNVTIMR_CNVCNT             (0xFFFFFFF0UL)             /*!< CNVCNT[27:0]: 28-bit timer counting conversion time */

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
/*                         AHB Master DMA2D Controller (DMA2D)                */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for DMA2D_CR register  ******************/

#define DMA2D_CR_START                     (0x00000001UL)               /*!< Start transfer */
#define DMA2D_CR_SUSP                      (0x00000002UL)               /*!< Suspend transfer */
#define DMA2D_CR_ABORT                     (0x00000004UL)               /*!< Abort transfer */
#define DMA2D_CR_TEIE                      (0x00000100UL)               /*!< Transfer Error Interrupt Enable */
#define DMA2D_CR_TCIE                      (0x00000200UL)               /*!< Transfer Complete Interrupt Enable */
#define DMA2D_CR_TWIE                      (0x00000400UL)               /*!< Transfer Watermark Interrupt Enable */
#define DMA2D_CR_CAEIE                     (0x00000800UL)               /*!< CLUT Access Error Interrupt Enable */
#define DMA2D_CR_CTCIE                     (0x00001000UL)               /*!< CLUT Transfer Complete Interrupt Enable */
#define DMA2D_CR_CEIE                      (0x00002000UL)               /*!< Configuration Error Interrupt Enable */
#define DMA2D_CR_MODE                      (0x00030000UL)               /*!< DMA2D Mode */

/********************  Bit definition for DMA2D_ISR register  *****************/

#define DMA2D_ISR_TEIF                     (0x00000001UL)               /*!< Transfer Error Interrupt Flag */
#define DMA2D_ISR_TCIF                     (0x00000002UL)               /*!< Transfer Complete Interrupt Flag */
#define DMA2D_ISR_TWIF                     (0x00000004UL)               /*!< Transfer Watermark Interrupt Flag */
#define DMA2D_ISR_CAEIF                    (0x00000008UL)               /*!< CLUT Access Error Interrupt Flag */
#define DMA2D_ISR_CTCIF                    (0x00000010UL)               /*!< CLUT Transfer Complete Interrupt Flag */
#define DMA2D_ISR_CEIF                     (0x00000020UL)               /*!< Configuration Error Interrupt Flag */

/********************  Bit definition for DMA2D_IFCR register  ****************/

#define DMA2D_IFCR_CTEIF                   (0x00000001UL)               /*!< Clears Transfer Error Interrupt Flag         */
#define DMA2D_IFCR_CTCIF                   (0x00000002UL)               /*!< Clears Transfer Complete Interrupt Flag      */
#define DMA2D_IFCR_CTWIF                   (0x00000004UL)               /*!< Clears Transfer Watermark Interrupt Flag     */
#define DMA2D_IFCR_CAECIF                  (0x00000008UL)               /*!< Clears CLUT Access Error Interrupt Flag      */
#define DMA2D_IFCR_CCTCIF                  (0x00000010UL)               /*!< Clears CLUT Transfer Complete Interrupt Flag */
#define DMA2D_IFCR_CCEIF                   (0x00000020UL)               /*!< Clears Configuration Error Interrupt Flag    */

/* Legacy defines */
#define DMA2D_IFSR_CTEIF                   DMA2D_IFCR_CTEIF                     /*!< Clears Transfer Error Interrupt Flag         */
#define DMA2D_IFSR_CTCIF                   DMA2D_IFCR_CTCIF                     /*!< Clears Transfer Complete Interrupt Flag      */
#define DMA2D_IFSR_CTWIF                   DMA2D_IFCR_CTWIF                     /*!< Clears Transfer Watermark Interrupt Flag     */
#define DMA2D_IFSR_CCAEIF                  DMA2D_IFCR_CAECIF                    /*!< Clears CLUT Access Error Interrupt Flag      */
#define DMA2D_IFSR_CCTCIF                  DMA2D_IFCR_CCTCIF                    /*!< Clears CLUT Transfer Complete Interrupt Flag */
#define DMA2D_IFSR_CCEIF                   DMA2D_IFCR_CCEIF                     /*!< Clears Configuration Error Interrupt Flag    */

/********************  Bit definition for DMA2D_FGMAR register  ***************/

#define DMA2D_FGMAR_MA                     (0xFFFFFFFFUL)               /*!< Memory Address */

/********************  Bit definition for DMA2D_FGOR register  ****************/

#define DMA2D_FGOR_LO                      (0x00003FFFUL)               /*!< Line Offset */

/********************  Bit definition for DMA2D_BGMAR register  ***************/

#define DMA2D_BGMAR_MA                     (0xFFFFFFFFUL)               /*!< Memory Address */

/********************  Bit definition for DMA2D_BGOR register  ****************/

#define DMA2D_BGOR_LO                      (0x00003FFFUL)               /*!< Line Offset */

/********************  Bit definition for DMA2D_FGPFCCR register  *************/

#define DMA2D_FGPFCCR_CM                   (0x0000000FUL)               /*!< Input color mode CM[3:0] */
#define DMA2D_FGPFCCR_CM_0                 (0x00000001UL)               /*!< Input color mode CM bit 0 */
#define DMA2D_FGPFCCR_CM_1                 (0x00000002UL)               /*!< Input color mode CM bit 1 */
#define DMA2D_FGPFCCR_CM_2                 (0x00000004UL)               /*!< Input color mode CM bit 2 */
#define DMA2D_FGPFCCR_CM_3                 (0x00000008UL)               /*!< Input color mode CM bit 3 */
#define DMA2D_FGPFCCR_CCM                  (0x00000010UL)               /*!< CLUT Color mode */
#define DMA2D_FGPFCCR_START                (0x00000020UL)               /*!< Start */
#define DMA2D_FGPFCCR_CS                   (0x0000FF00UL)               /*!< CLUT size */
#define DMA2D_FGPFCCR_AM                   (0x00030000UL)               /*!< Alpha mode AM[1:0] */
#define DMA2D_FGPFCCR_AM_0                 (0x00010000UL)               /*!< Alpha mode AM bit 0 */
#define DMA2D_FGPFCCR_AM_1                 (0x00020000UL)               /*!< Alpha mode AM bit 1 */
#define DMA2D_FGPFCCR_ALPHA                (0xFF000000UL)               /*!< Alpha value */

/********************  Bit definition for DMA2D_FGCOLR register  **************/

#define DMA2D_FGCOLR_BLUE                  (0x000000FFUL)               /*!< Blue Value */
#define DMA2D_FGCOLR_GREEN                 (0x0000FF00UL)               /*!< Green Value */
#define DMA2D_FGCOLR_RED                   (0x00FF0000UL)               /*!< Red Value */

/********************  Bit definition for DMA2D_BGPFCCR register  *************/

#define DMA2D_BGPFCCR_CM                   (0x0000000FUL)               /*!< Input color mode CM[3:0] */
#define DMA2D_BGPFCCR_CM_0                 (0x00000001UL)               /*!< Input color mode CM bit 0 */
#define DMA2D_BGPFCCR_CM_1                 (0x00000002UL)               /*!< Input color mode CM bit 1 */
#define DMA2D_BGPFCCR_CM_2                 (0x00000004UL)               /*!< Input color mode CM bit 2 */
#define DMA2D_FGPFCCR_CM_3                 (0x00000008UL)               /*!< Input color mode CM bit 3 */
#define DMA2D_BGPFCCR_CCM                  (0x00000010UL)               /*!< CLUT Color mode */
#define DMA2D_BGPFCCR_START                (0x00000020UL)               /*!< Start */
#define DMA2D_BGPFCCR_CS                   (0x0000FF00UL)               /*!< CLUT size */
#define DMA2D_BGPFCCR_AM                   (0x00030000UL)               /*!< Alpha mode AM[1:0] */
#define DMA2D_BGPFCCR_AM_0                 (0x00010000UL)               /*!< Alpha mode AM bit 0 */
#define DMA2D_BGPFCCR_AM_1                 (0x00020000UL)               /*!< Alpha mode AM bit 1 */
#define DMA2D_BGPFCCR_ALPHA                (0xFF000000UL)               /*!< Alpha value */

/********************  Bit definition for DMA2D_BGCOLR register  **************/

#define DMA2D_BGCOLR_BLUE                  (0x000000FFUL)               /*!< Blue Value */
#define DMA2D_BGCOLR_GREEN                 (0x0000FF00UL)               /*!< Green Value */
#define DMA2D_BGCOLR_RED                   (0x00FF0000UL)               /*!< Red Value */

/********************  Bit definition for DMA2D_FGCMAR register  **************/

#define DMA2D_FGCMAR_MA                    (0xFFFFFFFFUL)               /*!< Memory Address */

/********************  Bit definition for DMA2D_BGCMAR register  **************/

#define DMA2D_BGCMAR_MA                    (0xFFFFFFFFUL)               /*!< Memory Address */

/********************  Bit definition for DMA2D_OPFCCR register  **************/

#define DMA2D_OPFCCR_CM                    (0x00000007UL)               /*!< Color mode CM[2:0] */
#define DMA2D_OPFCCR_CM_0                  (0x00000001UL)               /*!< Color mode CM bit 0 */
#define DMA2D_OPFCCR_CM_1                  (0x00000002UL)               /*!< Color mode CM bit 1 */
#define DMA2D_OPFCCR_CM_2                  (0x00000004UL)               /*!< Color mode CM bit 2 */

/********************  Bit definition for DMA2D_OCOLR register  ***************/

/*!<Mode_ARGB8888/RGB888 */

#define DMA2D_OCOLR_BLUE_1                 (0x000000FFUL)               /*!< BLUE Value */
#define DMA2D_OCOLR_GREEN_1                (0x0000FF00UL)               /*!< GREEN Value  */
#define DMA2D_OCOLR_RED_1                  (0x00FF0000UL)               /*!< Red Value */
#define DMA2D_OCOLR_ALPHA_1                (0xFF000000UL)               /*!< Alpha Channel Value */

/*!<Mode_RGB565 */
#define DMA2D_OCOLR_BLUE_2                 (0x0000001FUL)               /*!< BLUE Value */
#define DMA2D_OCOLR_GREEN_2                (0x000007E0UL)               /*!< GREEN Value  */
#define DMA2D_OCOLR_RED_2                  (0x0000F800UL)               /*!< Red Value */

/*!<Mode_ARGB1555 */
#define DMA2D_OCOLR_BLUE_3                 (0x0000001FUL)               /*!< BLUE Value */
#define DMA2D_OCOLR_GREEN_3                (0x000003E0UL)               /*!< GREEN Value  */
#define DMA2D_OCOLR_RED_3                  (0x00007C00UL)               /*!< Red Value */
#define DMA2D_OCOLR_ALPHA_3                (0x00008000UL)               /*!< Alpha Channel Value */

/*!<Mode_ARGB4444 */
#define DMA2D_OCOLR_BLUE_4                 (0x0000000FUL)               /*!< BLUE Value */
#define DMA2D_OCOLR_GREEN_4                (0x000000F0UL)               /*!< GREEN Value  */
#define DMA2D_OCOLR_RED_4                  (0x00000F00UL)               /*!< Red Value */
#define DMA2D_OCOLR_ALPHA_4                (0x0000F000UL)               /*!< Alpha Channel Value */

/********************  Bit definition for DMA2D_OMAR register  ****************/

#define DMA2D_OMAR_MA                      (0xFFFFFFFFUL)               /*!< Memory Address */

/********************  Bit definition for DMA2D_OOR register  *****************/

#define DMA2D_OOR_LO                       (0x00003FFFUL)               /*!< Line Offset */

/********************  Bit definition for DMA2D_NLR register  *****************/

#define DMA2D_NLR_NL                       (0x0000FFFFUL)               /*!< Number of Lines */
#define DMA2D_NLR_PL                       (0x3FFF0000UL)               /*!< Pixel per Lines */

/********************  Bit definition for DMA2D_LWR register  *****************/

#define DMA2D_LWR_LW                       (0x0000FFFFUL)               /*!< Line Watermark */

/********************  Bit definition for DMA2D_AMTCR register  ***************/

#define DMA2D_AMTCR_EN                     (0x00000001UL)               /*!< Enable */
#define DMA2D_AMTCR_DT                     (0x0000FF00UL)               /*!< Dead Time */



/********************  Bit definition for DMA2D_FGCLUT register  **************/

/********************  Bit definition for DMA2D_BGCLUT register  **************/


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
#define  EXTI_IMR_MR23                       (0x00800000UL)        /*!< Interrupt Mask on line 23 */

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
#define  EXTI_EMR_MR23                       (0x00800000UL)        /*!< Event Mask on line 19 */

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
#define  EXTI_RTSR_TR23                      (0x00800000UL)        /*!< Rising trigger event configuration bit of line 23 */

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
#define  EXTI_FTSR_TR23                      (0x00800000UL)        /*!< Falling trigger event configuration bit of line 23 */

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
#define  EXTI_SWIER_SWIER23                  (0x00800000UL)        /*!< Software Interrupt on line 23 */

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
#define  EXTI_PR_PR23                        (0x00800000UL)        /*!< Pending bit for line 23 */

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
#define FLASH_ACR_LATENCY_8WS                (0x00000008UL)
#define FLASH_ACR_LATENCY_9WS                (0x00000009UL)
#define FLASH_ACR_LATENCY_10WS               (0x0000000AUL)
#define FLASH_ACR_LATENCY_11WS               (0x0000000BUL)
#define FLASH_ACR_LATENCY_12WS               (0x0000000CUL)
#define FLASH_ACR_LATENCY_13WS               (0x0000000DUL)
#define FLASH_ACR_LATENCY_14WS               (0x0000000EUL)
#define FLASH_ACR_LATENCY_15WS               (0x0000000FUL)

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
#define FLASH_CR_MER1                        FLASH_CR_MER
#define FLASH_CR_SNB                         (0x000000F8UL)
#define FLASH_CR_SNB_0                       (0x00000008UL)
#define FLASH_CR_SNB_1                       (0x00000010UL)
#define FLASH_CR_SNB_2                       (0x00000020UL)
#define FLASH_CR_SNB_3                       (0x00000040UL)
#define FLASH_CR_SNB_4                       (0x00000040UL)
#define FLASH_CR_PSIZE                       (0x00000300UL)
#define FLASH_CR_PSIZE_0                     (0x00000100UL)
#define FLASH_CR_PSIZE_1                     (0x00000200UL)
#define FLASH_CR_MER2                        (0x00008000UL)
#define FLASH_CR_STRT                        (0x00010000UL)
#define FLASH_CR_EOPIE                       (0x01000000UL)
#define FLASH_CR_LOCK                        (0x80000000UL)

/*******************  Bits definition for FLASH_OPTCR register  ***************/
#define FLASH_OPTCR_OPTLOCK                 (0x00000001UL)
#define FLASH_OPTCR_OPTSTRT                 (0x00000002UL)
#define FLASH_OPTCR_BOR_LEV_0               (0x00000004UL)
#define FLASH_OPTCR_BOR_LEV_1               (0x00000008UL)
#define FLASH_OPTCR_BOR_LEV                 (0x0000000CUL)
#define FLASH_OPTCR_BFB2                    (0x00000010UL)

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

#define FLASH_OPTCR_DB1M                    (0x40000000UL)
#define FLASH_OPTCR_SPRMOD                  (0x80000000UL)

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

#if defined(STM32F40_41xxx) || defined(STM32F412xG) || defined(STM32F413_423xx)
/******************************************************************************/
/*                                                                            */
/*                       Flexible Static Memory Controller                    */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for FSMC_BCR1 register  *******************/
#define  FSMC_BCR1_MBKEN                     (0x00000001UL)        /*!<Memory bank enable bit                 */
#define  FSMC_BCR1_MUXEN                     (0x00000002UL)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR1_MTYP                      (0x0000000CUL)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR1_MTYP_0                    (0x00000004UL)        /*!<Bit 0 */
#define  FSMC_BCR1_MTYP_1                    (0x00000008UL)        /*!<Bit 1 */

#define  FSMC_BCR1_MWID                      (0x00000030UL)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR1_MWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BCR1_MWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FSMC_BCR1_FACCEN                    (0x00000040UL)        /*!<Flash access enable                    */
#define  FSMC_BCR1_BURSTEN                   (0x00000100UL)        /*!<Burst enable bit                       */
#define  FSMC_BCR1_WAITPOL                   (0x00000200UL)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR1_WRAPMOD                   (0x00000400UL)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR1_WAITCFG                   (0x00000800UL)        /*!<Wait timing configuration              */
#define  FSMC_BCR1_WREN                      (0x00001000UL)        /*!<Write enable bit                       */
#define  FSMC_BCR1_WAITEN                    (0x00002000UL)        /*!<Wait enable bit                        */
#define  FSMC_BCR1_EXTMOD                    (0x00004000UL)        /*!<Extended mode enable                   */
#define  FSMC_BCR1_ASYNCWAIT                 (0x00008000UL)        /*!<Asynchronous wait                      */
#define  FSMC_BCR1_CBURSTRW                  (0x00080000UL)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BCR2 register  *******************/
#define  FSMC_BCR2_MBKEN                     (0x00000001UL)        /*!<Memory bank enable bit                */
#define  FSMC_BCR2_MUXEN                     (0x00000002UL)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR2_MTYP                      (0x0000000CUL)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR2_MTYP_0                    (0x00000004UL)        /*!<Bit 0 */
#define  FSMC_BCR2_MTYP_1                    (0x00000008UL)        /*!<Bit 1 */

#define  FSMC_BCR2_MWID                      (0x00000030UL)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR2_MWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BCR2_MWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FSMC_BCR2_FACCEN                    (0x00000040UL)        /*!<Flash access enable                    */
#define  FSMC_BCR2_BURSTEN                   (0x00000100UL)        /*!<Burst enable bit                       */
#define  FSMC_BCR2_WAITPOL                   (0x00000200UL)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR2_WRAPMOD                   (0x00000400UL)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR2_WAITCFG                   (0x00000800UL)        /*!<Wait timing configuration              */
#define  FSMC_BCR2_WREN                      (0x00001000UL)        /*!<Write enable bit                       */
#define  FSMC_BCR2_WAITEN                    (0x00002000UL)        /*!<Wait enable bit                        */
#define  FSMC_BCR2_EXTMOD                    (0x00004000UL)        /*!<Extended mode enable                   */
#define  FSMC_BCR2_ASYNCWAIT                 (0x00008000UL)        /*!<Asynchronous wait                      */
#define  FSMC_BCR2_CBURSTRW                  (0x00080000UL)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BCR3 register  *******************/
#define  FSMC_BCR3_MBKEN                     (0x00000001UL)        /*!<Memory bank enable bit                 */
#define  FSMC_BCR3_MUXEN                     (0x00000002UL)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR3_MTYP                      (0x0000000CUL)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR3_MTYP_0                    (0x00000004UL)        /*!<Bit 0 */
#define  FSMC_BCR3_MTYP_1                    (0x00000008UL)        /*!<Bit 1 */

#define  FSMC_BCR3_MWID                      (0x00000030UL)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR3_MWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BCR3_MWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FSMC_BCR3_FACCEN                    (0x00000040UL)        /*!<Flash access enable                    */
#define  FSMC_BCR3_BURSTEN                   (0x00000100UL)        /*!<Burst enable bit                       */
#define  FSMC_BCR3_WAITPOL                   (0x00000200UL)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR3_WRAPMOD                   (0x00000400UL)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR3_WAITCFG                   (0x00000800UL)        /*!<Wait timing configuration              */
#define  FSMC_BCR3_WREN                      (0x00001000UL)        /*!<Write enable bit                       */
#define  FSMC_BCR3_WAITEN                    (0x00002000UL)        /*!<Wait enable bit                        */
#define  FSMC_BCR3_EXTMOD                    (0x00004000UL)        /*!<Extended mode enable                   */
#define  FSMC_BCR3_ASYNCWAIT                 (0x00008000UL)        /*!<Asynchronous wait                      */
#define  FSMC_BCR3_CBURSTRW                  (0x00080000UL)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BCR4 register  *******************/
#define  FSMC_BCR4_MBKEN                     (0x00000001UL)        /*!<Memory bank enable bit */
#define  FSMC_BCR4_MUXEN                     (0x00000002UL)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR4_MTYP                      (0x0000000CUL)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR4_MTYP_0                    (0x00000004UL)        /*!<Bit 0 */
#define  FSMC_BCR4_MTYP_1                    (0x00000008UL)        /*!<Bit 1 */

#define  FSMC_BCR4_MWID                      (0x00000030UL)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR4_MWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BCR4_MWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FSMC_BCR4_FACCEN                    (0x00000040UL)        /*!<Flash access enable                    */
#define  FSMC_BCR4_BURSTEN                   (0x00000100UL)        /*!<Burst enable bit                       */
#define  FSMC_BCR4_WAITPOL                   (0x00000200UL)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR4_WRAPMOD                   (0x00000400UL)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR4_WAITCFG                   (0x00000800UL)        /*!<Wait timing configuration              */
#define  FSMC_BCR4_WREN                      (0x00001000UL)        /*!<Write enable bit                       */
#define  FSMC_BCR4_WAITEN                    (0x00002000UL)        /*!<Wait enable bit                        */
#define  FSMC_BCR4_EXTMOD                    (0x00004000UL)        /*!<Extended mode enable                   */
#define  FSMC_BCR4_ASYNCWAIT                 (0x00008000UL)        /*!<Asynchronous wait                      */
#define  FSMC_BCR4_CBURSTRW                  (0x00080000UL)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BTR1 register  ******************/
#define  FSMC_BTR1_ADDSET                    (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR1_ADDSET_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_BTR1_ADDSET_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_BTR1_ADDSET_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_BTR1_ADDSET_3                  (0x00000008UL)        /*!<Bit 3 */

#define  FSMC_BTR1_ADDHLD                    (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR1_ADDHLD_0                  (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BTR1_ADDHLD_1                  (0x00000020UL)        /*!<Bit 1 */
#define  FSMC_BTR1_ADDHLD_2                  (0x00000040UL)        /*!<Bit 2 */
#define  FSMC_BTR1_ADDHLD_3                  (0x00000080UL)        /*!<Bit 3 */

#define  FSMC_BTR1_DATAST                    (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR1_DATAST_0                  (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_BTR1_DATAST_1                  (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_BTR1_DATAST_2                  (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_BTR1_DATAST_3                  (0x00000800UL)        /*!<Bit 3 */

#define  FSMC_BTR1_BUSTURN                   (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR1_BUSTURN_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_BTR1_BUSTURN_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_BTR1_BUSTURN_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_BTR1_BUSTURN_3                 (0x00080000UL)        /*!<Bit 3 */

#define  FSMC_BTR1_CLKDIV                    (0x00F00000UL)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR1_CLKDIV_0                  (0x00100000UL)        /*!<Bit 0 */
#define  FSMC_BTR1_CLKDIV_1                  (0x00200000UL)        /*!<Bit 1 */
#define  FSMC_BTR1_CLKDIV_2                  (0x00400000UL)        /*!<Bit 2 */
#define  FSMC_BTR1_CLKDIV_3                  (0x00800000UL)        /*!<Bit 3 */

#define  FSMC_BTR1_DATLAT                    (0x0F000000UL)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR1_DATLAT_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_BTR1_DATLAT_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_BTR1_DATLAT_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_BTR1_DATLAT_3                  (0x08000000UL)        /*!<Bit 3 */

#define  FSMC_BTR1_ACCMOD                    (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR1_ACCMOD_0                  (0x10000000UL)        /*!<Bit 0 */
#define  FSMC_BTR1_ACCMOD_1                  (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BTR2 register  *******************/
#define  FSMC_BTR2_ADDSET                    (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR2_ADDSET_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_BTR2_ADDSET_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_BTR2_ADDSET_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_BTR2_ADDSET_3                  (0x00000008UL)        /*!<Bit 3 */

#define  FSMC_BTR2_ADDHLD                    (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR2_ADDHLD_0                  (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BTR2_ADDHLD_1                  (0x00000020UL)        /*!<Bit 1 */
#define  FSMC_BTR2_ADDHLD_2                  (0x00000040UL)        /*!<Bit 2 */
#define  FSMC_BTR2_ADDHLD_3                  (0x00000080UL)        /*!<Bit 3 */

#define  FSMC_BTR2_DATAST                    (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR2_DATAST_0                  (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_BTR2_DATAST_1                  (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_BTR2_DATAST_2                  (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_BTR2_DATAST_3                  (0x00000800UL)        /*!<Bit 3 */

#define  FSMC_BTR2_BUSTURN                   (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR2_BUSTURN_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_BTR2_BUSTURN_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_BTR2_BUSTURN_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_BTR2_BUSTURN_3                 (0x00080000UL)        /*!<Bit 3 */

#define  FSMC_BTR2_CLKDIV                    (0x00F00000UL)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR2_CLKDIV_0                  (0x00100000UL)        /*!<Bit 0 */
#define  FSMC_BTR2_CLKDIV_1                  (0x00200000UL)        /*!<Bit 1 */
#define  FSMC_BTR2_CLKDIV_2                  (0x00400000UL)        /*!<Bit 2 */
#define  FSMC_BTR2_CLKDIV_3                  (0x00800000UL)        /*!<Bit 3 */

#define  FSMC_BTR2_DATLAT                    (0x0F000000UL)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR2_DATLAT_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_BTR2_DATLAT_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_BTR2_DATLAT_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_BTR2_DATLAT_3                  (0x08000000UL)        /*!<Bit 3 */

#define  FSMC_BTR2_ACCMOD                    (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR2_ACCMOD_0                  (0x10000000UL)        /*!<Bit 0 */
#define  FSMC_BTR2_ACCMOD_1                  (0x20000000UL)        /*!<Bit 1 */

/*******************  Bit definition for FSMC_BTR3 register  *******************/
#define  FSMC_BTR3_ADDSET                    (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR3_ADDSET_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_BTR3_ADDSET_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_BTR3_ADDSET_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_BTR3_ADDSET_3                  (0x00000008UL)        /*!<Bit 3 */

#define  FSMC_BTR3_ADDHLD                    (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR3_ADDHLD_0                  (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BTR3_ADDHLD_1                  (0x00000020UL)        /*!<Bit 1 */
#define  FSMC_BTR3_ADDHLD_2                  (0x00000040UL)        /*!<Bit 2 */
#define  FSMC_BTR3_ADDHLD_3                  (0x00000080UL)        /*!<Bit 3 */

#define  FSMC_BTR3_DATAST                    (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR3_DATAST_0                  (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_BTR3_DATAST_1                  (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_BTR3_DATAST_2                  (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_BTR3_DATAST_3                  (0x00000800UL)        /*!<Bit 3 */

#define  FSMC_BTR3_BUSTURN                   (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR3_BUSTURN_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_BTR3_BUSTURN_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_BTR3_BUSTURN_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_BTR3_BUSTURN_3                 (0x00080000UL)        /*!<Bit 3 */

#define  FSMC_BTR3_CLKDIV                    (0x00F00000UL)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR3_CLKDIV_0                  (0x00100000UL)        /*!<Bit 0 */
#define  FSMC_BTR3_CLKDIV_1                  (0x00200000UL)        /*!<Bit 1 */
#define  FSMC_BTR3_CLKDIV_2                  (0x00400000UL)        /*!<Bit 2 */
#define  FSMC_BTR3_CLKDIV_3                  (0x00800000UL)        /*!<Bit 3 */

#define  FSMC_BTR3_DATLAT                    (0x0F000000UL)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR3_DATLAT_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_BTR3_DATLAT_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_BTR3_DATLAT_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_BTR3_DATLAT_3                  (0x08000000UL)        /*!<Bit 3 */

#define  FSMC_BTR3_ACCMOD                    (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR3_ACCMOD_0                  (0x10000000UL)        /*!<Bit 0 */
#define  FSMC_BTR3_ACCMOD_1                  (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BTR4 register  *******************/
#define  FSMC_BTR4_ADDSET                    (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR4_ADDSET_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_BTR4_ADDSET_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_BTR4_ADDSET_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_BTR4_ADDSET_3                  (0x00000008UL)        /*!<Bit 3 */

#define  FSMC_BTR4_ADDHLD                    (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR4_ADDHLD_0                  (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BTR4_ADDHLD_1                  (0x00000020UL)        /*!<Bit 1 */
#define  FSMC_BTR4_ADDHLD_2                  (0x00000040UL)        /*!<Bit 2 */
#define  FSMC_BTR4_ADDHLD_3                  (0x00000080UL)        /*!<Bit 3 */

#define  FSMC_BTR4_DATAST                    (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR4_DATAST_0                  (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_BTR4_DATAST_1                  (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_BTR4_DATAST_2                  (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_BTR4_DATAST_3                  (0x00000800UL)        /*!<Bit 3 */

#define  FSMC_BTR4_BUSTURN                   (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR4_BUSTURN_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_BTR4_BUSTURN_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_BTR4_BUSTURN_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_BTR4_BUSTURN_3                 (0x00080000UL)        /*!<Bit 3 */

#define  FSMC_BTR4_CLKDIV                    (0x00F00000UL)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR4_CLKDIV_0                  (0x00100000UL)        /*!<Bit 0 */
#define  FSMC_BTR4_CLKDIV_1                  (0x00200000UL)        /*!<Bit 1 */
#define  FSMC_BTR4_CLKDIV_2                  (0x00400000UL)        /*!<Bit 2 */
#define  FSMC_BTR4_CLKDIV_3                  (0x00800000UL)        /*!<Bit 3 */

#define  FSMC_BTR4_DATLAT                    (0x0F000000UL)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR4_DATLAT_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_BTR4_DATLAT_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_BTR4_DATLAT_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_BTR4_DATLAT_3                  (0x08000000UL)        /*!<Bit 3 */

#define  FSMC_BTR4_ACCMOD                    (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR4_ACCMOD_0                  (0x10000000UL)        /*!<Bit 0 */
#define  FSMC_BTR4_ACCMOD_1                  (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR1 register  ******************/
#define  FSMC_BWTR1_ADDSET                   (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR1_ADDSET_0                 (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_BWTR1_ADDSET_1                 (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_BWTR1_ADDSET_2                 (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_BWTR1_ADDSET_3                 (0x00000008UL)        /*!<Bit 3 */

#define  FSMC_BWTR1_ADDHLD                   (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR1_ADDHLD_0                 (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BWTR1_ADDHLD_1                 (0x00000020UL)        /*!<Bit 1 */
#define  FSMC_BWTR1_ADDHLD_2                 (0x00000040UL)        /*!<Bit 2 */
#define  FSMC_BWTR1_ADDHLD_3                 (0x00000080UL)        /*!<Bit 3 */

#define  FSMC_BWTR1_DATAST                   (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR1_DATAST_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_BWTR1_DATAST_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_BWTR1_DATAST_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_BWTR1_DATAST_3                 (0x00000800UL)        /*!<Bit 3 */

#define  FSMC_BWTR1_BUSTURN                  (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FSMC_BWTR1_BUSTURN_0                (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_BWTR1_BUSTURN_1                (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_BWTR1_BUSTURN_2                (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_BWTR1_BUSTURN_3                (0x00080000UL)        /*!<Bit 3 */

#define  FSMC_BWTR1_ACCMOD                   (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR1_ACCMOD_0                 (0x10000000UL)        /*!<Bit 0 */
#define  FSMC_BWTR1_ACCMOD_1                 (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR2 register  ******************/
#define  FSMC_BWTR2_ADDSET                   (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR2_ADDSET_0                 (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_BWTR2_ADDSET_1                 (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_BWTR2_ADDSET_2                 (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_BWTR2_ADDSET_3                 (0x00000008UL)        /*!<Bit 3 */

#define  FSMC_BWTR2_ADDHLD                   (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR2_ADDHLD_0                 (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BWTR2_ADDHLD_1                 (0x00000020UL)        /*!<Bit 1 */
#define  FSMC_BWTR2_ADDHLD_2                 (0x00000040UL)        /*!<Bit 2 */
#define  FSMC_BWTR2_ADDHLD_3                 (0x00000080UL)        /*!<Bit 3 */

#define  FSMC_BWTR2_DATAST                   (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR2_DATAST_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_BWTR2_DATAST_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_BWTR2_DATAST_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_BWTR2_DATAST_3                 (0x00000800UL)        /*!<Bit 3 */

#define  FSMC_BWTR2_BUSTURN                  (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FSMC_BWTR2_BUSTURN_0                (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_BWTR2_BUSTURN_1                (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_BWTR2_BUSTURN_2                (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_BWTR2_BUSTURN_3                (0x00080000UL)        /*!<Bit 3 */

#define  FSMC_BWTR2_ACCMOD                   (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR2_ACCMOD_0                 (0x10000000UL)        /*!<Bit 0 */
#define  FSMC_BWTR2_ACCMOD_1                 (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR3 register  ******************/
#define  FSMC_BWTR3_ADDSET                   (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR3_ADDSET_0                 (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_BWTR3_ADDSET_1                 (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_BWTR3_ADDSET_2                 (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_BWTR3_ADDSET_3                 (0x00000008UL)        /*!<Bit 3 */

#define  FSMC_BWTR3_ADDHLD                   (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR3_ADDHLD_0                 (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BWTR3_ADDHLD_1                 (0x00000020UL)        /*!<Bit 1 */
#define  FSMC_BWTR3_ADDHLD_2                 (0x00000040UL)        /*!<Bit 2 */
#define  FSMC_BWTR3_ADDHLD_3                 (0x00000080UL)        /*!<Bit 3 */

#define  FSMC_BWTR3_DATAST                   (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR3_DATAST_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_BWTR3_DATAST_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_BWTR3_DATAST_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_BWTR3_DATAST_3                 (0x00000800UL)        /*!<Bit 3 */

#define  FSMC_BWTR3_BUSTURN                  (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FSMC_BWTR3_BUSTURN_0                (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_BWTR3_BUSTURN_1                (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_BWTR3_BUSTURN_2                (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_BWTR3_BUSTURN_3                (0x00080000UL)        /*!<Bit 3 */

#define  FSMC_BWTR3_ACCMOD                   (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR3_ACCMOD_0                 (0x10000000UL)        /*!<Bit 0 */
#define  FSMC_BWTR3_ACCMOD_1                 (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR4 register  ******************/
#define  FSMC_BWTR4_ADDSET                   (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR4_ADDSET_0                 (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_BWTR4_ADDSET_1                 (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_BWTR4_ADDSET_2                 (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_BWTR4_ADDSET_3                 (0x00000008UL)        /*!<Bit 3 */

#define  FSMC_BWTR4_ADDHLD                   (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR4_ADDHLD_0                 (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_BWTR4_ADDHLD_1                 (0x00000020UL)        /*!<Bit 1 */
#define  FSMC_BWTR4_ADDHLD_2                 (0x00000040UL)        /*!<Bit 2 */
#define  FSMC_BWTR4_ADDHLD_3                 (0x00000080UL)        /*!<Bit 3 */

#define  FSMC_BWTR4_DATAST                   (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR4_DATAST_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_BWTR4_DATAST_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_BWTR4_DATAST_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_BWTR4_DATAST_3                 (0x00000800UL)        /*!<Bit 3 */

#define  FSMC_BWTR4_BUSTURN                  (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FSMC_BWTR4_BUSTURN_0                (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_BWTR4_BUSTURN_1                (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_BWTR4_BUSTURN_2                (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_BWTR4_BUSTURN_3                (0x00080000UL)        /*!<Bit 3 */

#define  FSMC_BWTR4_ACCMOD                   (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR4_ACCMOD_0                 (0x10000000UL)        /*!<Bit 0 */
#define  FSMC_BWTR4_ACCMOD_1                 (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FSMC_PCR2 register  *******************/
#define  FSMC_PCR2_PWAITEN                   (0x00000002UL)        /*!<Wait feature enable bit */
#define  FSMC_PCR2_PBKEN                     (0x00000004UL)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR2_PTYP                      (0x00000008UL)        /*!<Memory type */

#define  FSMC_PCR2_PWID                      (0x00000030UL)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR2_PWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_PCR2_PWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FSMC_PCR2_ECCEN                     (0x00000040UL)        /*!<ECC computation logic enable bit */

#define  FSMC_PCR2_TCLR                      (0x00001E00UL)        /*!<TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR2_TCLR_0                    (0x00000200UL)        /*!<Bit 0 */
#define  FSMC_PCR2_TCLR_1                    (0x00000400UL)        /*!<Bit 1 */
#define  FSMC_PCR2_TCLR_2                    (0x00000800UL)        /*!<Bit 2 */
#define  FSMC_PCR2_TCLR_3                    (0x00001000UL)        /*!<Bit 3 */

#define  FSMC_PCR2_TAR                       (0x0001E000UL)        /*!<TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR2_TAR_0                     (0x00002000UL)        /*!<Bit 0 */
#define  FSMC_PCR2_TAR_1                     (0x00004000UL)        /*!<Bit 1 */
#define  FSMC_PCR2_TAR_2                     (0x00008000UL)        /*!<Bit 2 */
#define  FSMC_PCR2_TAR_3                     (0x00010000UL)        /*!<Bit 3 */

#define  FSMC_PCR2_ECCPS                     (0x000E0000UL)        /*!<ECCPS[1:0] bits (ECC page size) */
#define  FSMC_PCR2_ECCPS_0                   (0x00020000UL)        /*!<Bit 0 */
#define  FSMC_PCR2_ECCPS_1                   (0x00040000UL)        /*!<Bit 1 */
#define  FSMC_PCR2_ECCPS_2                   (0x00080000UL)        /*!<Bit 2 */

/******************  Bit definition for FSMC_PCR3 register  *******************/
#define  FSMC_PCR3_PWAITEN                   (0x00000002UL)        /*!<Wait feature enable bit */
#define  FSMC_PCR3_PBKEN                     (0x00000004UL)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR3_PTYP                      (0x00000008UL)        /*!<Memory type */

#define  FSMC_PCR3_PWID                      (0x00000030UL)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR3_PWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_PCR3_PWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FSMC_PCR3_ECCEN                     (0x00000040UL)        /*!<ECC computation logic enable bit */

#define  FSMC_PCR3_TCLR                      (0x00001E00UL)        /*!<TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR3_TCLR_0                    (0x00000200UL)        /*!<Bit 0 */
#define  FSMC_PCR3_TCLR_1                    (0x00000400UL)        /*!<Bit 1 */
#define  FSMC_PCR3_TCLR_2                    (0x00000800UL)        /*!<Bit 2 */
#define  FSMC_PCR3_TCLR_3                    (0x00001000UL)        /*!<Bit 3 */

#define  FSMC_PCR3_TAR                       (0x0001E000UL)        /*!<TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR3_TAR_0                     (0x00002000UL)        /*!<Bit 0 */
#define  FSMC_PCR3_TAR_1                     (0x00004000UL)        /*!<Bit 1 */
#define  FSMC_PCR3_TAR_2                     (0x00008000UL)        /*!<Bit 2 */
#define  FSMC_PCR3_TAR_3                     (0x00010000UL)        /*!<Bit 3 */

#define  FSMC_PCR3_ECCPS                     (0x000E0000UL)        /*!<ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR3_ECCPS_0                   (0x00020000UL)        /*!<Bit 0 */
#define  FSMC_PCR3_ECCPS_1                   (0x00040000UL)        /*!<Bit 1 */
#define  FSMC_PCR3_ECCPS_2                   (0x00080000UL)        /*!<Bit 2 */

/******************  Bit definition for FSMC_PCR4 register  *******************/
#define  FSMC_PCR4_PWAITEN                   (0x00000002UL)        /*!<Wait feature enable bit */
#define  FSMC_PCR4_PBKEN                     (0x00000004UL)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR4_PTYP                      (0x00000008UL)        /*!<Memory type */

#define  FSMC_PCR4_PWID                      (0x00000030UL)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR4_PWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FSMC_PCR4_PWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FSMC_PCR4_ECCEN                     (0x00000040UL)        /*!<ECC computation logic enable bit */

#define  FSMC_PCR4_TCLR                      (0x00001E00UL)        /*!<TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR4_TCLR_0                    (0x00000200UL)        /*!<Bit 0 */
#define  FSMC_PCR4_TCLR_1                    (0x00000400UL)        /*!<Bit 1 */
#define  FSMC_PCR4_TCLR_2                    (0x00000800UL)        /*!<Bit 2 */
#define  FSMC_PCR4_TCLR_3                    (0x00001000UL)        /*!<Bit 3 */

#define  FSMC_PCR4_TAR                       (0x0001E000UL)        /*!<TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR4_TAR_0                     (0x00002000UL)        /*!<Bit 0 */
#define  FSMC_PCR4_TAR_1                     (0x00004000UL)        /*!<Bit 1 */
#define  FSMC_PCR4_TAR_2                     (0x00008000UL)        /*!<Bit 2 */
#define  FSMC_PCR4_TAR_3                     (0x00010000UL)        /*!<Bit 3 */

#define  FSMC_PCR4_ECCPS                     (0x000E0000UL)        /*!<ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR4_ECCPS_0                   (0x00020000UL)        /*!<Bit 0 */
#define  FSMC_PCR4_ECCPS_1                   (0x00040000UL)        /*!<Bit 1 */
#define  FSMC_PCR4_ECCPS_2                   (0x00080000UL)        /*!<Bit 2 */

/*******************  Bit definition for FSMC_SR2 register  *******************/
#define  FSMC_SR2_IRS                        (0x01U)               /*!<Interrupt Rising Edge status                */
#define  FSMC_SR2_ILS                        (0x02U)               /*!<Interrupt Level status                      */
#define  FSMC_SR2_IFS                        (0x04U)               /*!<Interrupt Falling Edge status               */
#define  FSMC_SR2_IREN                       (0x08U)               /*!<Interrupt Rising Edge detection Enable bit  */
#define  FSMC_SR2_ILEN                       (0x10U)               /*!<Interrupt Level detection Enable bit        */
#define  FSMC_SR2_IFEN                       (0x20U)               /*!<Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR2_FEMPT                      (0x40U)               /*!<FIFO empty */

/*******************  Bit definition for FSMC_SR3 register  *******************/
#define  FSMC_SR3_IRS                        (0x01U)               /*!<Interrupt Rising Edge status                */
#define  FSMC_SR3_ILS                        (0x02U)               /*!<Interrupt Level status                      */
#define  FSMC_SR3_IFS                        (0x04U)               /*!<Interrupt Falling Edge status               */
#define  FSMC_SR3_IREN                       (0x08U)               /*!<Interrupt Rising Edge detection Enable bit  */
#define  FSMC_SR3_ILEN                       (0x10U)               /*!<Interrupt Level detection Enable bit        */
#define  FSMC_SR3_IFEN                       (0x20U)               /*!<Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR3_FEMPT                      (0x40U)               /*!<FIFO empty */

/*******************  Bit definition for FSMC_SR4 register  *******************/
#define  FSMC_SR4_IRS                        (0x01U)               /*!<Interrupt Rising Edge status                 */
#define  FSMC_SR4_ILS                        (0x02U)               /*!<Interrupt Level status                       */
#define  FSMC_SR4_IFS                        (0x04U)               /*!<Interrupt Falling Edge status                */
#define  FSMC_SR4_IREN                       (0x08U)               /*!<Interrupt Rising Edge detection Enable bit   */
#define  FSMC_SR4_ILEN                       (0x10U)               /*!<Interrupt Level detection Enable bit         */
#define  FSMC_SR4_IFEN                       (0x20U)               /*!<Interrupt Falling Edge detection Enable bit  */
#define  FSMC_SR4_FEMPT                      (0x40U)               /*!<FIFO empty */

/******************  Bit definition for FSMC_PMEM2 register  ******************/
#define  FSMC_PMEM2_MEMSET2                  (0x000000FFUL)        /*!<MEMSET2[7:0] bits (Common memory 2 setup time) */
#define  FSMC_PMEM2_MEMSET2_0                (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMSET2_1                (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMSET2_2                (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMSET2_3                (0x00000008UL)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMSET2_4                (0x00000010UL)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMSET2_5                (0x00000020UL)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMSET2_6                (0x00000040UL)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMSET2_7                (0x00000080UL)        /*!<Bit 7 */

#define  FSMC_PMEM2_MEMWAIT2                 (0x0000FF00UL)        /*!<MEMWAIT2[7:0] bits (Common memory 2 wait time) */
#define  FSMC_PMEM2_MEMWAIT2_0               (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMWAIT2_1               (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMWAIT2_2               (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMWAIT2_3               (0x00000800UL)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMWAIT2_4               (0x00001000UL)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMWAIT2_5               (0x00002000UL)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMWAIT2_6               (0x00004000UL)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMWAIT2_7               (0x00008000UL)        /*!<Bit 7 */

#define  FSMC_PMEM2_MEMHOLD2                 (0x00FF0000UL)        /*!<MEMHOLD2[7:0] bits (Common memory 2 hold time) */
#define  FSMC_PMEM2_MEMHOLD2_0               (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMHOLD2_1               (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMHOLD2_2               (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMHOLD2_3               (0x00080000UL)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMHOLD2_4               (0x00100000UL)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMHOLD2_5               (0x00200000UL)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMHOLD2_6               (0x00400000UL)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMHOLD2_7               (0x00800000UL)        /*!<Bit 7 */

#define  FSMC_PMEM2_MEMHIZ2                  (0xFF000000UL)        /*!<MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time) */
#define  FSMC_PMEM2_MEMHIZ2_0                (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMHIZ2_1                (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMHIZ2_2                (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMHIZ2_3                (0x08000000UL)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMHIZ2_4                (0x10000000UL)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMHIZ2_5                (0x20000000UL)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMHIZ2_6                (0x40000000UL)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMHIZ2_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PMEM3 register  ******************/
#define  FSMC_PMEM3_MEMSET3                  (0x000000FFUL)        /*!<MEMSET3[7:0] bits (Common memory 3 setup time) */
#define  FSMC_PMEM3_MEMSET3_0                (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMSET3_1                (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMSET3_2                (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMSET3_3                (0x00000008UL)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMSET3_4                (0x00000010UL)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMSET3_5                (0x00000020UL)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMSET3_6                (0x00000040UL)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMSET3_7                (0x00000080UL)        /*!<Bit 7 */

#define  FSMC_PMEM3_MEMWAIT3                 (0x0000FF00UL)        /*!<MEMWAIT3[7:0] bits (Common memory 3 wait time) */
#define  FSMC_PMEM3_MEMWAIT3_0               (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMWAIT3_1               (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMWAIT3_2               (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMWAIT3_3               (0x00000800UL)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMWAIT3_4               (0x00001000UL)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMWAIT3_5               (0x00002000UL)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMWAIT3_6               (0x00004000UL)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMWAIT3_7               (0x00008000UL)        /*!<Bit 7 */

#define  FSMC_PMEM3_MEMHOLD3                 (0x00FF0000UL)        /*!<MEMHOLD3[7:0] bits (Common memory 3 hold time) */
#define  FSMC_PMEM3_MEMHOLD3_0               (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMHOLD3_1               (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMHOLD3_2               (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMHOLD3_3               (0x00080000UL)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMHOLD3_4               (0x00100000UL)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMHOLD3_5               (0x00200000UL)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMHOLD3_6               (0x00400000UL)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMHOLD3_7               (0x00800000UL)        /*!<Bit 7 */

#define  FSMC_PMEM3_MEMHIZ3                  (0xFF000000UL)        /*!<MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time) */
#define  FSMC_PMEM3_MEMHIZ3_0                (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMHIZ3_1                (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMHIZ3_2                (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMHIZ3_3                (0x08000000UL)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMHIZ3_4                (0x10000000UL)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMHIZ3_5                (0x20000000UL)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMHIZ3_6                (0x40000000UL)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMHIZ3_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PMEM4 register  ******************/
#define  FSMC_PMEM4_MEMSET4                  (0x000000FFUL)        /*!<MEMSET4[7:0] bits (Common memory 4 setup time) */
#define  FSMC_PMEM4_MEMSET4_0                (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMSET4_1                (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMSET4_2                (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMSET4_3                (0x00000008UL)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMSET4_4                (0x00000010UL)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMSET4_5                (0x00000020UL)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMSET4_6                (0x00000040UL)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMSET4_7                (0x00000080UL)        /*!<Bit 7 */

#define  FSMC_PMEM4_MEMWAIT4                 (0x0000FF00UL)        /*!<MEMWAIT4[7:0] bits (Common memory 4 wait time) */
#define  FSMC_PMEM4_MEMWAIT4_0               (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMWAIT4_1               (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMWAIT4_2               (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMWAIT4_3               (0x00000800UL)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMWAIT4_4               (0x00001000UL)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMWAIT4_5               (0x00002000UL)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMWAIT4_6               (0x00004000UL)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMWAIT4_7               (0x00008000UL)        /*!<Bit 7 */

#define  FSMC_PMEM4_MEMHOLD4                 (0x00FF0000UL)        /*!<MEMHOLD4[7:0] bits (Common memory 4 hold time) */
#define  FSMC_PMEM4_MEMHOLD4_0               (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMHOLD4_1               (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMHOLD4_2               (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMHOLD4_3               (0x00080000UL)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMHOLD4_4               (0x00100000UL)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMHOLD4_5               (0x00200000UL)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMHOLD4_6               (0x00400000UL)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMHOLD4_7               (0x00800000UL)        /*!<Bit 7 */

#define  FSMC_PMEM4_MEMHIZ4                  (0xFF000000UL)        /*!<MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time) */
#define  FSMC_PMEM4_MEMHIZ4_0                (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMHIZ4_1                (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMHIZ4_2                (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMHIZ4_3                (0x08000000UL)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMHIZ4_4                (0x10000000UL)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMHIZ4_5                (0x20000000UL)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMHIZ4_6                (0x40000000UL)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMHIZ4_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PATT2 register  ******************/
#define  FSMC_PATT2_ATTSET2                  (0x000000FFUL)        /*!<ATTSET2[7:0] bits (Attribute memory 2 setup time) */
#define  FSMC_PATT2_ATTSET2_0                (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTSET2_1                (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTSET2_2                (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTSET2_3                (0x00000008UL)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTSET2_4                (0x00000010UL)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTSET2_5                (0x00000020UL)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTSET2_6                (0x00000040UL)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTSET2_7                (0x00000080UL)        /*!<Bit 7 */

#define  FSMC_PATT2_ATTWAIT2                 (0x0000FF00UL)        /*!<ATTWAIT2[7:0] bits (Attribute memory 2 wait time) */
#define  FSMC_PATT2_ATTWAIT2_0               (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTWAIT2_1               (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTWAIT2_2               (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTWAIT2_3               (0x00000800UL)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTWAIT2_4               (0x00001000UL)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTWAIT2_5               (0x00002000UL)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTWAIT2_6               (0x00004000UL)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTWAIT2_7               (0x00008000UL)        /*!<Bit 7 */

#define  FSMC_PATT2_ATTHOLD2                 (0x00FF0000UL)        /*!<ATTHOLD2[7:0] bits (Attribute memory 2 hold time) */
#define  FSMC_PATT2_ATTHOLD2_0               (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTHOLD2_1               (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTHOLD2_2               (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTHOLD2_3               (0x00080000UL)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTHOLD2_4               (0x00100000UL)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTHOLD2_5               (0x00200000UL)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTHOLD2_6               (0x00400000UL)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTHOLD2_7               (0x00800000UL)        /*!<Bit 7 */

#define  FSMC_PATT2_ATTHIZ2                  (0xFF000000UL)        /*!<ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time) */
#define  FSMC_PATT2_ATTHIZ2_0                (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTHIZ2_1                (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTHIZ2_2                (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTHIZ2_3                (0x08000000UL)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTHIZ2_4                (0x10000000UL)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTHIZ2_5                (0x20000000UL)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTHIZ2_6                (0x40000000UL)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTHIZ2_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PATT3 register  ******************/
#define  FSMC_PATT3_ATTSET3                  (0x000000FFUL)        /*!<ATTSET3[7:0] bits (Attribute memory 3 setup time) */
#define  FSMC_PATT3_ATTSET3_0                (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTSET3_1                (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTSET3_2                (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTSET3_3                (0x00000008UL)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTSET3_4                (0x00000010UL)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTSET3_5                (0x00000020UL)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTSET3_6                (0x00000040UL)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTSET3_7                (0x00000080UL)        /*!<Bit 7 */

#define  FSMC_PATT3_ATTWAIT3                 (0x0000FF00UL)        /*!<ATTWAIT3[7:0] bits (Attribute memory 3 wait time) */
#define  FSMC_PATT3_ATTWAIT3_0               (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTWAIT3_1               (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTWAIT3_2               (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTWAIT3_3               (0x00000800UL)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTWAIT3_4               (0x00001000UL)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTWAIT3_5               (0x00002000UL)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTWAIT3_6               (0x00004000UL)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTWAIT3_7               (0x00008000UL)        /*!<Bit 7 */

#define  FSMC_PATT3_ATTHOLD3                 (0x00FF0000UL)        /*!<ATTHOLD3[7:0] bits (Attribute memory 3 hold time) */
#define  FSMC_PATT3_ATTHOLD3_0               (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTHOLD3_1               (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTHOLD3_2               (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTHOLD3_3               (0x00080000UL)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTHOLD3_4               (0x00100000UL)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTHOLD3_5               (0x00200000UL)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTHOLD3_6               (0x00400000UL)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTHOLD3_7               (0x00800000UL)        /*!<Bit 7 */

#define  FSMC_PATT3_ATTHIZ3                  (0xFF000000UL)        /*!<ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time) */
#define  FSMC_PATT3_ATTHIZ3_0                (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTHIZ3_1                (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTHIZ3_2                (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTHIZ3_3                (0x08000000UL)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTHIZ3_4                (0x10000000UL)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTHIZ3_5                (0x20000000UL)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTHIZ3_6                (0x40000000UL)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTHIZ3_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PATT4 register  ******************/
#define  FSMC_PATT4_ATTSET4                  (0x000000FFUL)        /*!<ATTSET4[7:0] bits (Attribute memory 4 setup time) */
#define  FSMC_PATT4_ATTSET4_0                (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTSET4_1                (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTSET4_2                (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTSET4_3                (0x00000008UL)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTSET4_4                (0x00000010UL)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTSET4_5                (0x00000020UL)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTSET4_6                (0x00000040UL)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTSET4_7                (0x00000080UL)        /*!<Bit 7 */

#define  FSMC_PATT4_ATTWAIT4                 (0x0000FF00UL)        /*!<ATTWAIT4[7:0] bits (Attribute memory 4 wait time) */
#define  FSMC_PATT4_ATTWAIT4_0               (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTWAIT4_1               (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTWAIT4_2               (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTWAIT4_3               (0x00000800UL)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTWAIT4_4               (0x00001000UL)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTWAIT4_5               (0x00002000UL)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTWAIT4_6               (0x00004000UL)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTWAIT4_7               (0x00008000UL)        /*!<Bit 7 */

#define  FSMC_PATT4_ATTHOLD4                 (0x00FF0000UL)        /*!<ATTHOLD4[7:0] bits (Attribute memory 4 hold time) */
#define  FSMC_PATT4_ATTHOLD4_0               (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTHOLD4_1               (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTHOLD4_2               (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTHOLD4_3               (0x00080000UL)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTHOLD4_4               (0x00100000UL)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTHOLD4_5               (0x00200000UL)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTHOLD4_6               (0x00400000UL)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTHOLD4_7               (0x00800000UL)        /*!<Bit 7 */

#define  FSMC_PATT4_ATTHIZ4                  (0xFF000000UL)        /*!<ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time) */
#define  FSMC_PATT4_ATTHIZ4_0                (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTHIZ4_1                (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTHIZ4_2                (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTHIZ4_3                (0x08000000UL)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTHIZ4_4                (0x10000000UL)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTHIZ4_5                (0x20000000UL)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTHIZ4_6                (0x40000000UL)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTHIZ4_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PIO4 register  *******************/
#define  FSMC_PIO4_IOSET4                    (0x000000FFUL)        /*!<IOSET4[7:0] bits (I/O 4 setup time) */
#define  FSMC_PIO4_IOSET4_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FSMC_PIO4_IOSET4_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FSMC_PIO4_IOSET4_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FSMC_PIO4_IOSET4_3                  (0x00000008UL)        /*!<Bit 3 */
#define  FSMC_PIO4_IOSET4_4                  (0x00000010UL)        /*!<Bit 4 */
#define  FSMC_PIO4_IOSET4_5                  (0x00000020UL)        /*!<Bit 5 */
#define  FSMC_PIO4_IOSET4_6                  (0x00000040UL)        /*!<Bit 6 */
#define  FSMC_PIO4_IOSET4_7                  (0x00000080UL)        /*!<Bit 7 */

#define  FSMC_PIO4_IOWAIT4                   (0x0000FF00UL)        /*!<IOWAIT4[7:0] bits (I/O 4 wait time) */
#define  FSMC_PIO4_IOWAIT4_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FSMC_PIO4_IOWAIT4_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FSMC_PIO4_IOWAIT4_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FSMC_PIO4_IOWAIT4_3                 (0x00000800UL)        /*!<Bit 3 */
#define  FSMC_PIO4_IOWAIT4_4                 (0x00001000UL)        /*!<Bit 4 */
#define  FSMC_PIO4_IOWAIT4_5                 (0x00002000UL)        /*!<Bit 5 */
#define  FSMC_PIO4_IOWAIT4_6                 (0x00004000UL)        /*!<Bit 6 */
#define  FSMC_PIO4_IOWAIT4_7                 (0x00008000UL)        /*!<Bit 7 */

#define  FSMC_PIO4_IOHOLD4                   (0x00FF0000UL)        /*!<IOHOLD4[7:0] bits (I/O 4 hold time) */
#define  FSMC_PIO4_IOHOLD4_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FSMC_PIO4_IOHOLD4_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FSMC_PIO4_IOHOLD4_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FSMC_PIO4_IOHOLD4_3                 (0x00080000UL)        /*!<Bit 3 */
#define  FSMC_PIO4_IOHOLD4_4                 (0x00100000UL)        /*!<Bit 4 */
#define  FSMC_PIO4_IOHOLD4_5                 (0x00200000UL)        /*!<Bit 5 */
#define  FSMC_PIO4_IOHOLD4_6                 (0x00400000UL)        /*!<Bit 6 */
#define  FSMC_PIO4_IOHOLD4_7                 (0x00800000UL)        /*!<Bit 7 */

#define  FSMC_PIO4_IOHIZ4                    (0xFF000000UL)        /*!<IOHIZ4[7:0] bits (I/O 4 databus HiZ time) */
#define  FSMC_PIO4_IOHIZ4_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FSMC_PIO4_IOHIZ4_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FSMC_PIO4_IOHIZ4_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FSMC_PIO4_IOHIZ4_3                  (0x08000000UL)        /*!<Bit 3 */
#define  FSMC_PIO4_IOHIZ4_4                  (0x10000000UL)        /*!<Bit 4 */
#define  FSMC_PIO4_IOHIZ4_5                  (0x20000000UL)        /*!<Bit 5 */
#define  FSMC_PIO4_IOHIZ4_6                  (0x40000000UL)        /*!<Bit 6 */
#define  FSMC_PIO4_IOHIZ4_7                  (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FSMC_ECCR2 register  ******************/
#define  FSMC_ECCR2_ECC2                     (0xFFFFFFFFUL)        /*!<ECC result */

/******************  Bit definition for FSMC_ECCR3 register  ******************/
#define  FSMC_ECCR3_ECC3                     (0xFFFFFFFFUL)        /*!<ECC result */
#endif /* STM32F40_41xxx || STM32F412xG || STM32F413_423xx */

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
/******************************************************************************/
/*                                                                            */
/*                          Flexible Memory Controller                        */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for FMC_BCR1 register  *******************/
#define  FMC_BCR1_MBKEN                     (0x00000001UL)        /*!<Memory bank enable bit                 */
#define  FMC_BCR1_MUXEN                     (0x00000002UL)        /*!<Address/data multiplexing enable bit   */

#define  FMC_BCR1_MTYP                      (0x0000000CUL)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FMC_BCR1_MTYP_0                    (0x00000004UL)        /*!<Bit 0 */
#define  FMC_BCR1_MTYP_1                    (0x00000008UL)        /*!<Bit 1 */

#define  FMC_BCR1_MWID                      (0x00000030UL)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FMC_BCR1_MWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BCR1_MWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FMC_BCR1_FACCEN                    (0x00000040UL)        /*!<Flash access enable        */
#define  FMC_BCR1_BURSTEN                   (0x00000100UL)        /*!<Burst enable bit           */
#define  FMC_BCR1_WAITPOL                   (0x00000200UL)        /*!<Wait signal polarity bit   */
#define  FMC_BCR1_WRAPMOD                   (0x00000400UL)        /*!<Wrapped burst mode support */
#define  FMC_BCR1_WAITCFG                   (0x00000800UL)        /*!<Wait timing configuration  */
#define  FMC_BCR1_WREN                      (0x00001000UL)        /*!<Write enable bit           */
#define  FMC_BCR1_WAITEN                    (0x00002000UL)        /*!<Wait enable bit            */
#define  FMC_BCR1_EXTMOD                    (0x00004000UL)        /*!<Extended mode enable       */
#define  FMC_BCR1_ASYNCWAIT                 (0x00008000UL)        /*!<Asynchronous wait          */
#define  FMC_BCR1_CBURSTRW                  (0x00080000UL)        /*!<Write burst enable         */
#define  FMC_BCR1_CCLKEN                    (0x00100000UL)        /*!<Continous clock enable     */

/******************  Bit definition for FMC_BCR2 register  *******************/
#define  FMC_BCR2_MBKEN                     (0x00000001UL)        /*!<Memory bank enable bit                 */
#define  FMC_BCR2_MUXEN                     (0x00000002UL)        /*!<Address/data multiplexing enable bit   */

#define  FMC_BCR2_MTYP                      (0x0000000CUL)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FMC_BCR2_MTYP_0                    (0x00000004UL)        /*!<Bit 0 */
#define  FMC_BCR2_MTYP_1                    (0x00000008UL)        /*!<Bit 1 */

#define  FMC_BCR2_MWID                      (0x00000030UL)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FMC_BCR2_MWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BCR2_MWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FMC_BCR2_FACCEN                    (0x00000040UL)        /*!<Flash access enable        */
#define  FMC_BCR2_BURSTEN                   (0x00000100UL)        /*!<Burst enable bit           */
#define  FMC_BCR2_WAITPOL                   (0x00000200UL)        /*!<Wait signal polarity bit   */
#define  FMC_BCR2_WRAPMOD                   (0x00000400UL)        /*!<Wrapped burst mode support */
#define  FMC_BCR2_WAITCFG                   (0x00000800UL)        /*!<Wait timing configuration  */
#define  FMC_BCR2_WREN                      (0x00001000UL)        /*!<Write enable bit           */
#define  FMC_BCR2_WAITEN                    (0x00002000UL)        /*!<Wait enable bit            */
#define  FMC_BCR2_EXTMOD                    (0x00004000UL)        /*!<Extended mode enable       */
#define  FMC_BCR2_ASYNCWAIT                 (0x00008000UL)        /*!<Asynchronous wait          */
#define  FMC_BCR2_CBURSTRW                  (0x00080000UL)        /*!<Write burst enable         */

/******************  Bit definition for FMC_BCR3 register  *******************/
#define  FMC_BCR3_MBKEN                     (0x00000001UL)        /*!<Memory bank enable bit                 */
#define  FMC_BCR3_MUXEN                     (0x00000002UL)        /*!<Address/data multiplexing enable bit   */

#define  FMC_BCR3_MTYP                      (0x0000000CUL)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FMC_BCR3_MTYP_0                    (0x00000004UL)        /*!<Bit 0 */
#define  FMC_BCR3_MTYP_1                    (0x00000008UL)        /*!<Bit 1 */

#define  FMC_BCR3_MWID                      (0x00000030UL)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FMC_BCR3_MWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BCR3_MWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FMC_BCR3_FACCEN                    (0x00000040UL)        /*!<Flash access enable        */
#define  FMC_BCR3_BURSTEN                   (0x00000100UL)        /*!<Burst enable bit           */
#define  FMC_BCR3_WAITPOL                   (0x00000200UL)        /*!<Wait signal polarity bit   */
#define  FMC_BCR3_WRAPMOD                   (0x00000400UL)        /*!<Wrapped burst mode support */
#define  FMC_BCR3_WAITCFG                   (0x00000800UL)        /*!<Wait timing configuration  */
#define  FMC_BCR3_WREN                      (0x00001000UL)        /*!<Write enable bit           */
#define  FMC_BCR3_WAITEN                    (0x00002000UL)        /*!<Wait enable bit            */
#define  FMC_BCR3_EXTMOD                    (0x00004000UL)        /*!<Extended mode enable       */
#define  FMC_BCR3_ASYNCWAIT                 (0x00008000UL)        /*!<Asynchronous wait          */
#define  FMC_BCR3_CBURSTRW                  (0x00080000UL)        /*!<Write burst enable         */

/******************  Bit definition for FMC_BCR4 register  *******************/
#define  FMC_BCR4_MBKEN                     (0x00000001UL)        /*!<Memory bank enable bit                 */
#define  FMC_BCR4_MUXEN                     (0x00000002UL)        /*!<Address/data multiplexing enable bit   */

#define  FMC_BCR4_MTYP                      (0x0000000CUL)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FMC_BCR4_MTYP_0                    (0x00000004UL)        /*!<Bit 0 */
#define  FMC_BCR4_MTYP_1                    (0x00000008UL)        /*!<Bit 1 */

#define  FMC_BCR4_MWID                      (0x00000030UL)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FMC_BCR4_MWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BCR4_MWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FMC_BCR4_FACCEN                    (0x00000040UL)        /*!<Flash access enable        */
#define  FMC_BCR4_BURSTEN                   (0x00000100UL)        /*!<Burst enable bit           */
#define  FMC_BCR4_WAITPOL                   (0x00000200UL)        /*!<Wait signal polarity bit   */
#define  FMC_BCR4_WRAPMOD                   (0x00000400UL)        /*!<Wrapped burst mode support */
#define  FMC_BCR4_WAITCFG                   (0x00000800UL)        /*!<Wait timing configuration  */
#define  FMC_BCR4_WREN                      (0x00001000UL)        /*!<Write enable bit           */
#define  FMC_BCR4_WAITEN                    (0x00002000UL)        /*!<Wait enable bit            */
#define  FMC_BCR4_EXTMOD                    (0x00004000UL)        /*!<Extended mode enable       */
#define  FMC_BCR4_ASYNCWAIT                 (0x00008000UL)        /*!<Asynchronous wait          */
#define  FMC_BCR4_CBURSTRW                  (0x00080000UL)        /*!<Write burst enable         */

/******************  Bit definition for FMC_BTR1 register  ******************/
#define  FMC_BTR1_ADDSET                    (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BTR1_ADDSET_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FMC_BTR1_ADDSET_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FMC_BTR1_ADDSET_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FMC_BTR1_ADDSET_3                  (0x00000008UL)        /*!<Bit 3 */

#define  FMC_BTR1_ADDHLD                    (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration)  */
#define  FMC_BTR1_ADDHLD_0                  (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BTR1_ADDHLD_1                  (0x00000020UL)        /*!<Bit 1 */
#define  FMC_BTR1_ADDHLD_2                  (0x00000040UL)        /*!<Bit 2 */
#define  FMC_BTR1_ADDHLD_3                  (0x00000080UL)        /*!<Bit 3 */

#define  FMC_BTR1_DATAST                    (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BTR1_DATAST_0                  (0x00000100UL)        /*!<Bit 0 */
#define  FMC_BTR1_DATAST_1                  (0x00000200UL)        /*!<Bit 1 */
#define  FMC_BTR1_DATAST_2                  (0x00000400UL)        /*!<Bit 2 */
#define  FMC_BTR1_DATAST_3                  (0x00000800UL)        /*!<Bit 3 */
#define  FMC_BTR1_DATAST_4                  (0x00001000UL)        /*!<Bit 4 */
#define  FMC_BTR1_DATAST_5                  (0x00002000UL)        /*!<Bit 5 */
#define  FMC_BTR1_DATAST_6                  (0x00004000UL)        /*!<Bit 6 */
#define  FMC_BTR1_DATAST_7                  (0x00008000UL)        /*!<Bit 7 */

#define  FMC_BTR1_BUSTURN                   (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FMC_BTR1_BUSTURN_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FMC_BTR1_BUSTURN_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FMC_BTR1_BUSTURN_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FMC_BTR1_BUSTURN_3                 (0x00080000UL)        /*!<Bit 3 */

#define  FMC_BTR1_CLKDIV                    (0x00F00000UL)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FMC_BTR1_CLKDIV_0                  (0x00100000UL)        /*!<Bit 0 */
#define  FMC_BTR1_CLKDIV_1                  (0x00200000UL)        /*!<Bit 1 */
#define  FMC_BTR1_CLKDIV_2                  (0x00400000UL)        /*!<Bit 2 */
#define  FMC_BTR1_CLKDIV_3                  (0x00800000UL)        /*!<Bit 3 */

#define  FMC_BTR1_DATLAT                    (0x0F000000UL)        /*!<DATLA[3:0] bits (Data latency) */
#define  FMC_BTR1_DATLAT_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FMC_BTR1_DATLAT_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FMC_BTR1_DATLAT_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FMC_BTR1_DATLAT_3                  (0x08000000UL)        /*!<Bit 3 */

#define  FMC_BTR1_ACCMOD                    (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BTR1_ACCMOD_0                  (0x10000000UL)        /*!<Bit 0 */
#define  FMC_BTR1_ACCMOD_1                  (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FMC_BTR2 register  *******************/
#define  FMC_BTR2_ADDSET                    (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BTR2_ADDSET_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FMC_BTR2_ADDSET_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FMC_BTR2_ADDSET_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FMC_BTR2_ADDSET_3                  (0x00000008UL)        /*!<Bit 3 */

#define  FMC_BTR2_ADDHLD                    (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FMC_BTR2_ADDHLD_0                  (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BTR2_ADDHLD_1                  (0x00000020UL)        /*!<Bit 1 */
#define  FMC_BTR2_ADDHLD_2                  (0x00000040UL)        /*!<Bit 2 */
#define  FMC_BTR2_ADDHLD_3                  (0x00000080UL)        /*!<Bit 3 */

#define  FMC_BTR2_DATAST                    (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BTR2_DATAST_0                  (0x00000100UL)        /*!<Bit 0 */
#define  FMC_BTR2_DATAST_1                  (0x00000200UL)        /*!<Bit 1 */
#define  FMC_BTR2_DATAST_2                  (0x00000400UL)        /*!<Bit 2 */
#define  FMC_BTR2_DATAST_3                  (0x00000800UL)        /*!<Bit 3 */
#define  FMC_BTR2_DATAST_4                  (0x00001000UL)        /*!<Bit 4 */
#define  FMC_BTR2_DATAST_5                  (0x00002000UL)        /*!<Bit 5 */
#define  FMC_BTR2_DATAST_6                  (0x00004000UL)        /*!<Bit 6 */
#define  FMC_BTR2_DATAST_7                  (0x00008000UL)        /*!<Bit 7 */

#define  FMC_BTR2_BUSTURN                   (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FMC_BTR2_BUSTURN_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FMC_BTR2_BUSTURN_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FMC_BTR2_BUSTURN_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FMC_BTR2_BUSTURN_3                 (0x00080000UL)        /*!<Bit 3 */

#define  FMC_BTR2_CLKDIV                    (0x00F00000UL)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FMC_BTR2_CLKDIV_0                  (0x00100000UL)        /*!<Bit 0 */
#define  FMC_BTR2_CLKDIV_1                  (0x00200000UL)        /*!<Bit 1 */
#define  FMC_BTR2_CLKDIV_2                  (0x00400000UL)        /*!<Bit 2 */
#define  FMC_BTR2_CLKDIV_3                  (0x00800000UL)        /*!<Bit 3 */

#define  FMC_BTR2_DATLAT                    (0x0F000000UL)        /*!<DATLA[3:0] bits (Data latency) */
#define  FMC_BTR2_DATLAT_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FMC_BTR2_DATLAT_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FMC_BTR2_DATLAT_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FMC_BTR2_DATLAT_3                  (0x08000000UL)        /*!<Bit 3 */

#define  FMC_BTR2_ACCMOD                    (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BTR2_ACCMOD_0                  (0x10000000UL)        /*!<Bit 0 */
#define  FMC_BTR2_ACCMOD_1                  (0x20000000UL)        /*!<Bit 1 */

/*******************  Bit definition for FMC_BTR3 register  *******************/
#define  FMC_BTR3_ADDSET                    (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BTR3_ADDSET_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FMC_BTR3_ADDSET_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FMC_BTR3_ADDSET_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FMC_BTR3_ADDSET_3                  (0x00000008UL)        /*!<Bit 3 */

#define  FMC_BTR3_ADDHLD                    (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FMC_BTR3_ADDHLD_0                  (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BTR3_ADDHLD_1                  (0x00000020UL)        /*!<Bit 1 */
#define  FMC_BTR3_ADDHLD_2                  (0x00000040UL)        /*!<Bit 2 */
#define  FMC_BTR3_ADDHLD_3                  (0x00000080UL)        /*!<Bit 3 */

#define  FMC_BTR3_DATAST                    (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BTR3_DATAST_0                  (0x00000100UL)        /*!<Bit 0 */
#define  FMC_BTR3_DATAST_1                  (0x00000200UL)        /*!<Bit 1 */
#define  FMC_BTR3_DATAST_2                  (0x00000400UL)        /*!<Bit 2 */
#define  FMC_BTR3_DATAST_3                  (0x00000800UL)        /*!<Bit 3 */
#define  FMC_BTR3_DATAST_4                  (0x00001000UL)        /*!<Bit 4 */
#define  FMC_BTR3_DATAST_5                  (0x00002000UL)        /*!<Bit 5 */
#define  FMC_BTR3_DATAST_6                  (0x00004000UL)        /*!<Bit 6 */
#define  FMC_BTR3_DATAST_7                  (0x00008000UL)        /*!<Bit 7 */

#define  FMC_BTR3_BUSTURN                   (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FMC_BTR3_BUSTURN_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FMC_BTR3_BUSTURN_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FMC_BTR3_BUSTURN_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FMC_BTR3_BUSTURN_3                 (0x00080000UL)        /*!<Bit 3 */

#define  FMC_BTR3_CLKDIV                    (0x00F00000UL)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FMC_BTR3_CLKDIV_0                  (0x00100000UL)        /*!<Bit 0 */
#define  FMC_BTR3_CLKDIV_1                  (0x00200000UL)        /*!<Bit 1 */
#define  FMC_BTR3_CLKDIV_2                  (0x00400000UL)        /*!<Bit 2 */
#define  FMC_BTR3_CLKDIV_3                  (0x00800000UL)        /*!<Bit 3 */

#define  FMC_BTR3_DATLAT                    (0x0F000000UL)        /*!<DATLA[3:0] bits (Data latency) */
#define  FMC_BTR3_DATLAT_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FMC_BTR3_DATLAT_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FMC_BTR3_DATLAT_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FMC_BTR3_DATLAT_3                  (0x08000000UL)        /*!<Bit 3 */

#define  FMC_BTR3_ACCMOD                    (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BTR3_ACCMOD_0                  (0x10000000UL)        /*!<Bit 0 */
#define  FMC_BTR3_ACCMOD_1                  (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FMC_BTR4 register  *******************/
#define  FMC_BTR4_ADDSET                    (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BTR4_ADDSET_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FMC_BTR4_ADDSET_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FMC_BTR4_ADDSET_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FMC_BTR4_ADDSET_3                  (0x00000008UL)        /*!<Bit 3 */

#define  FMC_BTR4_ADDHLD                    (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FMC_BTR4_ADDHLD_0                  (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BTR4_ADDHLD_1                  (0x00000020UL)        /*!<Bit 1 */
#define  FMC_BTR4_ADDHLD_2                  (0x00000040UL)        /*!<Bit 2 */
#define  FMC_BTR4_ADDHLD_3                  (0x00000080UL)        /*!<Bit 3 */

#define  FMC_BTR4_DATAST                    (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BTR4_DATAST_0                  (0x00000100UL)        /*!<Bit 0 */
#define  FMC_BTR4_DATAST_1                  (0x00000200UL)        /*!<Bit 1 */
#define  FMC_BTR4_DATAST_2                  (0x00000400UL)        /*!<Bit 2 */
#define  FMC_BTR4_DATAST_3                  (0x00000800UL)        /*!<Bit 3 */
#define  FMC_BTR4_DATAST_4                  (0x00001000UL)        /*!<Bit 4 */
#define  FMC_BTR4_DATAST_5                  (0x00002000UL)        /*!<Bit 5 */
#define  FMC_BTR4_DATAST_6                  (0x00004000UL)        /*!<Bit 6 */
#define  FMC_BTR4_DATAST_7                  (0x00008000UL)        /*!<Bit 7 */

#define  FMC_BTR4_BUSTURN                   (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FMC_BTR4_BUSTURN_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FMC_BTR4_BUSTURN_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FMC_BTR4_BUSTURN_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FMC_BTR4_BUSTURN_3                 (0x00080000UL)        /*!<Bit 3 */

#define  FMC_BTR4_CLKDIV                    (0x00F00000UL)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FMC_BTR4_CLKDIV_0                  (0x00100000UL)        /*!<Bit 0 */
#define  FMC_BTR4_CLKDIV_1                  (0x00200000UL)        /*!<Bit 1 */
#define  FMC_BTR4_CLKDIV_2                  (0x00400000UL)        /*!<Bit 2 */
#define  FMC_BTR4_CLKDIV_3                  (0x00800000UL)        /*!<Bit 3 */

#define  FMC_BTR4_DATLAT                    (0x0F000000UL)        /*!<DATLA[3:0] bits (Data latency) */
#define  FMC_BTR4_DATLAT_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FMC_BTR4_DATLAT_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FMC_BTR4_DATLAT_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FMC_BTR4_DATLAT_3                  (0x08000000UL)        /*!<Bit 3 */

#define  FMC_BTR4_ACCMOD                    (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BTR4_ACCMOD_0                  (0x10000000UL)        /*!<Bit 0 */
#define  FMC_BTR4_ACCMOD_1                  (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FMC_BWTR1 register  ******************/
#define  FMC_BWTR1_ADDSET                   (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BWTR1_ADDSET_0                 (0x00000001UL)        /*!<Bit 0 */
#define  FMC_BWTR1_ADDSET_1                 (0x00000002UL)        /*!<Bit 1 */
#define  FMC_BWTR1_ADDSET_2                 (0x00000004UL)        /*!<Bit 2 */
#define  FMC_BWTR1_ADDSET_3                 (0x00000008UL)        /*!<Bit 3 */

#define  FMC_BWTR1_ADDHLD                   (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FMC_BWTR1_ADDHLD_0                 (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BWTR1_ADDHLD_1                 (0x00000020UL)        /*!<Bit 1 */
#define  FMC_BWTR1_ADDHLD_2                 (0x00000040UL)        /*!<Bit 2 */
#define  FMC_BWTR1_ADDHLD_3                 (0x00000080UL)        /*!<Bit 3 */

#define  FMC_BWTR1_DATAST                   (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BWTR1_DATAST_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FMC_BWTR1_DATAST_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FMC_BWTR1_DATAST_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FMC_BWTR1_DATAST_3                 (0x00000800UL)        /*!<Bit 3 */
#define  FMC_BWTR1_DATAST_4                 (0x00001000UL)        /*!<Bit 4 */
#define  FMC_BWTR1_DATAST_5                 (0x00002000UL)        /*!<Bit 5 */
#define  FMC_BWTR1_DATAST_6                 (0x00004000UL)        /*!<Bit 6 */
#define  FMC_BWTR1_DATAST_7                 (0x00008000UL)        /*!<Bit 7 */

#define  FMC_BWTR1_BUSTURN                  (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FMC_BWTR1_BUSTURN_0                (0x00010000UL)        /*!<Bit 0 */
#define  FMC_BWTR1_BUSTURN_1                (0x00020000UL)        /*!<Bit 1 */
#define  FMC_BWTR1_BUSTURN_2                (0x00040000UL)        /*!<Bit 2 */
#define  FMC_BWTR1_BUSTURN_3                (0x00080000UL)        /*!<Bit 3 */

#define  FMC_BWTR1_ACCMOD                   (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BWTR1_ACCMOD_0                 (0x10000000UL)        /*!<Bit 0 */
#define  FMC_BWTR1_ACCMOD_1                 (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FMC_BWTR2 register  ******************/
#define  FMC_BWTR2_ADDSET                   (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BWTR2_ADDSET_0                 (0x00000001UL)        /*!<Bit 0 */
#define  FMC_BWTR2_ADDSET_1                 (0x00000002UL)        /*!<Bit 1 */
#define  FMC_BWTR2_ADDSET_2                 (0x00000004UL)        /*!<Bit 2 */
#define  FMC_BWTR2_ADDSET_3                 (0x00000008UL)        /*!<Bit 3 */

#define  FMC_BWTR2_ADDHLD                   (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FMC_BWTR2_ADDHLD_0                 (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BWTR2_ADDHLD_1                 (0x00000020UL)        /*!<Bit 1 */
#define  FMC_BWTR2_ADDHLD_2                 (0x00000040UL)        /*!<Bit 2 */
#define  FMC_BWTR2_ADDHLD_3                 (0x00000080UL)        /*!<Bit 3 */

#define  FMC_BWTR2_DATAST                   (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BWTR2_DATAST_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FMC_BWTR2_DATAST_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FMC_BWTR2_DATAST_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FMC_BWTR2_DATAST_3                 (0x00000800UL)        /*!<Bit 3 */
#define  FMC_BWTR2_DATAST_4                 (0x00001000UL)        /*!<Bit 4 */
#define  FMC_BWTR2_DATAST_5                 (0x00002000UL)        /*!<Bit 5 */
#define  FMC_BWTR2_DATAST_6                 (0x00004000UL)        /*!<Bit 6 */
#define  FMC_BWTR2_DATAST_7                 (0x00008000UL)        /*!<Bit 7 */

#define  FMC_BWTR2_BUSTURN                  (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FMC_BWTR2_BUSTURN_0                (0x00010000UL)        /*!<Bit 0 */
#define  FMC_BWTR2_BUSTURN_1                (0x00020000UL)        /*!<Bit 1 */
#define  FMC_BWTR2_BUSTURN_2                (0x00040000UL)        /*!<Bit 2 */
#define  FMC_BWTR2_BUSTURN_3                (0x00080000UL)        /*!<Bit 3 */

#define  FMC_BWTR2_ACCMOD                   (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BWTR2_ACCMOD_0                 (0x10000000UL)        /*!<Bit 0 */
#define  FMC_BWTR2_ACCMOD_1                 (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FMC_BWTR3 register  ******************/
#define  FMC_BWTR3_ADDSET                   (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BWTR3_ADDSET_0                 (0x00000001UL)        /*!<Bit 0 */
#define  FMC_BWTR3_ADDSET_1                 (0x00000002UL)        /*!<Bit 1 */
#define  FMC_BWTR3_ADDSET_2                 (0x00000004UL)        /*!<Bit 2 */
#define  FMC_BWTR3_ADDSET_3                 (0x00000008UL)        /*!<Bit 3 */

#define  FMC_BWTR3_ADDHLD                   (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FMC_BWTR3_ADDHLD_0                 (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BWTR3_ADDHLD_1                 (0x00000020UL)        /*!<Bit 1 */
#define  FMC_BWTR3_ADDHLD_2                 (0x00000040UL)        /*!<Bit 2 */
#define  FMC_BWTR3_ADDHLD_3                 (0x00000080UL)        /*!<Bit 3 */

#define  FMC_BWTR3_DATAST                   (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BWTR3_DATAST_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FMC_BWTR3_DATAST_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FMC_BWTR3_DATAST_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FMC_BWTR3_DATAST_3                 (0x00000800UL)        /*!<Bit 3 */
#define  FMC_BWTR3_DATAST_4                 (0x00001000UL)        /*!<Bit 4 */
#define  FMC_BWTR3_DATAST_5                 (0x00002000UL)        /*!<Bit 5 */
#define  FMC_BWTR3_DATAST_6                 (0x00004000UL)        /*!<Bit 6 */
#define  FMC_BWTR3_DATAST_7                 (0x00008000UL)        /*!<Bit 7 */

#define  FMC_BWTR3_BUSTURN                  (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FMC_BWTR3_BUSTURN_0                (0x00010000UL)        /*!<Bit 0 */
#define  FMC_BWTR3_BUSTURN_1                (0x00020000UL)        /*!<Bit 1 */
#define  FMC_BWTR3_BUSTURN_2                (0x00040000UL)        /*!<Bit 2 */
#define  FMC_BWTR3_BUSTURN_3                (0x00080000UL)        /*!<Bit 3 */

#define  FMC_BWTR3_ACCMOD                   (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BWTR3_ACCMOD_0                 (0x10000000UL)        /*!<Bit 0 */
#define  FMC_BWTR3_ACCMOD_1                 (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FMC_BWTR4 register  ******************/
#define  FMC_BWTR4_ADDSET                   (0x0000000FUL)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BWTR4_ADDSET_0                 (0x00000001UL)        /*!<Bit 0 */
#define  FMC_BWTR4_ADDSET_1                 (0x00000002UL)        /*!<Bit 1 */
#define  FMC_BWTR4_ADDSET_2                 (0x00000004UL)        /*!<Bit 2 */
#define  FMC_BWTR4_ADDSET_3                 (0x00000008UL)        /*!<Bit 3 */

#define  FMC_BWTR4_ADDHLD                   (0x000000F0UL)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FMC_BWTR4_ADDHLD_0                 (0x00000010UL)        /*!<Bit 0 */
#define  FMC_BWTR4_ADDHLD_1                 (0x00000020UL)        /*!<Bit 1 */
#define  FMC_BWTR4_ADDHLD_2                 (0x00000040UL)        /*!<Bit 2 */
#define  FMC_BWTR4_ADDHLD_3                 (0x00000080UL)        /*!<Bit 3 */

#define  FMC_BWTR4_DATAST                   (0x0000FF00UL)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BWTR4_DATAST_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FMC_BWTR4_DATAST_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FMC_BWTR4_DATAST_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FMC_BWTR4_DATAST_3                 (0x00000800UL)        /*!<Bit 3 */
#define  FMC_BWTR4_DATAST_4                 (0x00001000UL)        /*!<Bit 4 */
#define  FMC_BWTR4_DATAST_5                 (0x00002000UL)        /*!<Bit 5 */
#define  FMC_BWTR4_DATAST_6                 (0x00004000UL)        /*!<Bit 6 */
#define  FMC_BWTR4_DATAST_7                 (0x00008000UL)        /*!<Bit 7 */

#define  FMC_BWTR4_BUSTURN                  (0x000F0000UL)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FMC_BWTR4_BUSTURN_0                (0x00010000UL)        /*!<Bit 0 */
#define  FMC_BWTR4_BUSTURN_1                (0x00020000UL)        /*!<Bit 1 */
#define  FMC_BWTR4_BUSTURN_2                (0x00040000UL)        /*!<Bit 2 */
#define  FMC_BWTR4_BUSTURN_3                (0x00080000UL)        /*!<Bit 3 */

#define  FMC_BWTR4_ACCMOD                   (0x30000000UL)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BWTR4_ACCMOD_0                 (0x10000000UL)        /*!<Bit 0 */
#define  FMC_BWTR4_ACCMOD_1                 (0x20000000UL)        /*!<Bit 1 */

/******************  Bit definition for FMC_PCR2 register  *******************/
#define  FMC_PCR2_PWAITEN                   (0x00000002UL)        /*!<Wait feature enable bit                   */
#define  FMC_PCR2_PBKEN                     (0x00000004UL)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FMC_PCR2_PTYP                      (0x00000008UL)        /*!<Memory type                               */

#define  FMC_PCR2_PWID                      (0x00000030UL)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FMC_PCR2_PWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FMC_PCR2_PWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FMC_PCR2_ECCEN                     (0x00000040UL)        /*!<ECC computation logic enable bit          */

#define  FMC_PCR2_TCLR                      (0x00001E00UL)        /*!<TCLR[3:0] bits (CLE to RE delay)          */
#define  FMC_PCR2_TCLR_0                    (0x00000200UL)        /*!<Bit 0 */
#define  FMC_PCR2_TCLR_1                    (0x00000400UL)        /*!<Bit 1 */
#define  FMC_PCR2_TCLR_2                    (0x00000800UL)        /*!<Bit 2 */
#define  FMC_PCR2_TCLR_3                    (0x00001000UL)        /*!<Bit 3 */

#define  FMC_PCR2_TAR                       (0x0001E000UL)        /*!<TAR[3:0] bits (ALE to RE delay)           */
#define  FMC_PCR2_TAR_0                     (0x00002000UL)        /*!<Bit 0 */
#define  FMC_PCR2_TAR_1                     (0x00004000UL)        /*!<Bit 1 */
#define  FMC_PCR2_TAR_2                     (0x00008000UL)        /*!<Bit 2 */
#define  FMC_PCR2_TAR_3                     (0x00010000UL)        /*!<Bit 3 */

#define  FMC_PCR2_ECCPS                     (0x000E0000UL)        /*!<ECCPS[1:0] bits (ECC page size)           */
#define  FMC_PCR2_ECCPS_0                   (0x00020000UL)        /*!<Bit 0 */
#define  FMC_PCR2_ECCPS_1                   (0x00040000UL)        /*!<Bit 1 */
#define  FMC_PCR2_ECCPS_2                   (0x00080000UL)        /*!<Bit 2 */

/******************  Bit definition for FMC_PCR3 register  *******************/
#define  FMC_PCR3_PWAITEN                   (0x00000002UL)        /*!<Wait feature enable bit                   */
#define  FMC_PCR3_PBKEN                     (0x00000004UL)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FMC_PCR3_PTYP                      (0x00000008UL)        /*!<Memory type                               */

#define  FMC_PCR3_PWID                      (0x00000030UL)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FMC_PCR3_PWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FMC_PCR3_PWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FMC_PCR3_ECCEN                     (0x00000040UL)        /*!<ECC computation logic enable bit          */

#define  FMC_PCR3_TCLR                      (0x00001E00UL)        /*!<TCLR[3:0] bits (CLE to RE delay)          */
#define  FMC_PCR3_TCLR_0                    (0x00000200UL)        /*!<Bit 0 */
#define  FMC_PCR3_TCLR_1                    (0x00000400UL)        /*!<Bit 1 */
#define  FMC_PCR3_TCLR_2                    (0x00000800UL)        /*!<Bit 2 */
#define  FMC_PCR3_TCLR_3                    (0x00001000UL)        /*!<Bit 3 */

#define  FMC_PCR3_TAR                       (0x0001E000UL)        /*!<TAR[3:0] bits (ALE to RE delay)           */
#define  FMC_PCR3_TAR_0                     (0x00002000UL)        /*!<Bit 0 */
#define  FMC_PCR3_TAR_1                     (0x00004000UL)        /*!<Bit 1 */
#define  FMC_PCR3_TAR_2                     (0x00008000UL)        /*!<Bit 2 */
#define  FMC_PCR3_TAR_3                     (0x00010000UL)        /*!<Bit 3 */

#define  FMC_PCR3_ECCPS                     (0x000E0000UL)        /*!<ECCPS[2:0] bits (ECC page size)           */
#define  FMC_PCR3_ECCPS_0                   (0x00020000UL)        /*!<Bit 0 */
#define  FMC_PCR3_ECCPS_1                   (0x00040000UL)        /*!<Bit 1 */
#define  FMC_PCR3_ECCPS_2                   (0x00080000UL)        /*!<Bit 2 */

/******************  Bit definition for FMC_PCR4 register  *******************/
#define  FMC_PCR4_PWAITEN                   (0x00000002UL)        /*!<Wait feature enable bit                   */
#define  FMC_PCR4_PBKEN                     (0x00000004UL)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FMC_PCR4_PTYP                      (0x00000008UL)        /*!<Memory type                               */

#define  FMC_PCR4_PWID                      (0x00000030UL)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FMC_PCR4_PWID_0                    (0x00000010UL)        /*!<Bit 0 */
#define  FMC_PCR4_PWID_1                    (0x00000020UL)        /*!<Bit 1 */

#define  FMC_PCR4_ECCEN                     (0x00000040UL)        /*!<ECC computation logic enable bit          */

#define  FMC_PCR4_TCLR                      (0x00001E00UL)        /*!<TCLR[3:0] bits (CLE to RE delay)          */
#define  FMC_PCR4_TCLR_0                    (0x00000200UL)        /*!<Bit 0 */
#define  FMC_PCR4_TCLR_1                    (0x00000400UL)        /*!<Bit 1 */
#define  FMC_PCR4_TCLR_2                    (0x00000800UL)        /*!<Bit 2 */
#define  FMC_PCR4_TCLR_3                    (0x00001000UL)        /*!<Bit 3 */

#define  FMC_PCR4_TAR                       (0x0001E000UL)        /*!<TAR[3:0] bits (ALE to RE delay)           */
#define  FMC_PCR4_TAR_0                     (0x00002000UL)        /*!<Bit 0 */
#define  FMC_PCR4_TAR_1                     (0x00004000UL)        /*!<Bit 1 */
#define  FMC_PCR4_TAR_2                     (0x00008000UL)        /*!<Bit 2 */
#define  FMC_PCR4_TAR_3                     (0x00010000UL)        /*!<Bit 3 */

#define  FMC_PCR4_ECCPS                     (0x000E0000UL)        /*!<ECCPS[2:0] bits (ECC page size)           */
#define  FMC_PCR4_ECCPS_0                   (0x00020000UL)        /*!<Bit 0 */
#define  FMC_PCR4_ECCPS_1                   (0x00040000UL)        /*!<Bit 1 */
#define  FMC_PCR4_ECCPS_2                   (0x00080000UL)        /*!<Bit 2 */

/*******************  Bit definition for FMC_SR2 register  *******************/
#define  FMC_SR2_IRS                        (0x01U)               /*!<Interrupt Rising Edge status                */
#define  FMC_SR2_ILS                        (0x02U)               /*!<Interrupt Level status                      */
#define  FMC_SR2_IFS                        (0x04U)               /*!<Interrupt Falling Edge status               */
#define  FMC_SR2_IREN                       (0x08U)               /*!<Interrupt Rising Edge detection Enable bit  */
#define  FMC_SR2_ILEN                       (0x10U)               /*!<Interrupt Level detection Enable bit        */
#define  FMC_SR2_IFEN                       (0x20U)               /*!<Interrupt Falling Edge detection Enable bit */
#define  FMC_SR2_FEMPT                      (0x40U)               /*!<FIFO empty                                  */

/*******************  Bit definition for FMC_SR3 register  *******************/
#define  FMC_SR3_IRS                        (0x01U)               /*!<Interrupt Rising Edge status                */
#define  FMC_SR3_ILS                        (0x02U)               /*!<Interrupt Level status                      */
#define  FMC_SR3_IFS                        (0x04U)               /*!<Interrupt Falling Edge status               */
#define  FMC_SR3_IREN                       (0x08U)               /*!<Interrupt Rising Edge detection Enable bit  */
#define  FMC_SR3_ILEN                       (0x10U)               /*!<Interrupt Level detection Enable bit        */
#define  FMC_SR3_IFEN                       (0x20U)               /*!<Interrupt Falling Edge detection Enable bit */
#define  FMC_SR3_FEMPT                      (0x40U)               /*!<FIFO empty                                  */

/*******************  Bit definition for FMC_SR4 register  *******************/
#define  FMC_SR4_IRS                        (0x01U)               /*!<Interrupt Rising Edge status                */
#define  FMC_SR4_ILS                        (0x02U)               /*!<Interrupt Level status                      */
#define  FMC_SR4_IFS                        (0x04U)               /*!<Interrupt Falling Edge status               */
#define  FMC_SR4_IREN                       (0x08U)               /*!<Interrupt Rising Edge detection Enable bit  */
#define  FMC_SR4_ILEN                       (0x10U)               /*!<Interrupt Level detection Enable bit        */
#define  FMC_SR4_IFEN                       (0x20U)               /*!<Interrupt Falling Edge detection Enable bit */
#define  FMC_SR4_FEMPT                      (0x40U)               /*!<FIFO empty                                  */

/******************  Bit definition for FMC_PMEM2 register  ******************/
#define  FMC_PMEM2_MEMSET2                  (0x000000FFUL)        /*!<MEMSET2[7:0] bits (Common memory 2 setup time) */
#define  FMC_PMEM2_MEMSET2_0                (0x00000001UL)        /*!<Bit 0 */
#define  FMC_PMEM2_MEMSET2_1                (0x00000002UL)        /*!<Bit 1 */
#define  FMC_PMEM2_MEMSET2_2                (0x00000004UL)        /*!<Bit 2 */
#define  FMC_PMEM2_MEMSET2_3                (0x00000008UL)        /*!<Bit 3 */
#define  FMC_PMEM2_MEMSET2_4                (0x00000010UL)        /*!<Bit 4 */
#define  FMC_PMEM2_MEMSET2_5                (0x00000020UL)        /*!<Bit 5 */
#define  FMC_PMEM2_MEMSET2_6                (0x00000040UL)        /*!<Bit 6 */
#define  FMC_PMEM2_MEMSET2_7                (0x00000080UL)        /*!<Bit 7 */

#define  FMC_PMEM2_MEMWAIT2                 (0x0000FF00UL)        /*!<MEMWAIT2[7:0] bits (Common memory 2 wait time) */
#define  FMC_PMEM2_MEMWAIT2_0               (0x00000100UL)        /*!<Bit 0 */
#define  FMC_PMEM2_MEMWAIT2_1               (0x00000200UL)        /*!<Bit 1 */
#define  FMC_PMEM2_MEMWAIT2_2               (0x00000400UL)        /*!<Bit 2 */
#define  FMC_PMEM2_MEMWAIT2_3               (0x00000800UL)        /*!<Bit 3 */
#define  FMC_PMEM2_MEMWAIT2_4               (0x00001000UL)        /*!<Bit 4 */
#define  FMC_PMEM2_MEMWAIT2_5               (0x00002000UL)        /*!<Bit 5 */
#define  FMC_PMEM2_MEMWAIT2_6               (0x00004000UL)        /*!<Bit 6 */
#define  FMC_PMEM2_MEMWAIT2_7               (0x00008000UL)        /*!<Bit 7 */

#define  FMC_PMEM2_MEMHOLD2                 (0x00FF0000UL)        /*!<MEMHOLD2[7:0] bits (Common memory 2 hold time) */
#define  FMC_PMEM2_MEMHOLD2_0               (0x00010000UL)        /*!<Bit 0 */
#define  FMC_PMEM2_MEMHOLD2_1               (0x00020000UL)        /*!<Bit 1 */
#define  FMC_PMEM2_MEMHOLD2_2               (0x00040000UL)        /*!<Bit 2 */
#define  FMC_PMEM2_MEMHOLD2_3               (0x00080000UL)        /*!<Bit 3 */
#define  FMC_PMEM2_MEMHOLD2_4               (0x00100000UL)        /*!<Bit 4 */
#define  FMC_PMEM2_MEMHOLD2_5               (0x00200000UL)        /*!<Bit 5 */
#define  FMC_PMEM2_MEMHOLD2_6               (0x00400000UL)        /*!<Bit 6 */
#define  FMC_PMEM2_MEMHOLD2_7               (0x00800000UL)        /*!<Bit 7 */

#define  FMC_PMEM2_MEMHIZ2                  (0xFF000000UL)        /*!<MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time) */
#define  FMC_PMEM2_MEMHIZ2_0                (0x01000000UL)        /*!<Bit 0 */
#define  FMC_PMEM2_MEMHIZ2_1                (0x02000000UL)        /*!<Bit 1 */
#define  FMC_PMEM2_MEMHIZ2_2                (0x04000000UL)        /*!<Bit 2 */
#define  FMC_PMEM2_MEMHIZ2_3                (0x08000000UL)        /*!<Bit 3 */
#define  FMC_PMEM2_MEMHIZ2_4                (0x10000000UL)        /*!<Bit 4 */
#define  FMC_PMEM2_MEMHIZ2_5                (0x20000000UL)        /*!<Bit 5 */
#define  FMC_PMEM2_MEMHIZ2_6                (0x40000000UL)        /*!<Bit 6 */
#define  FMC_PMEM2_MEMHIZ2_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FMC_PMEM3 register  ******************/
#define  FMC_PMEM3_MEMSET3                  (0x000000FFUL)        /*!<MEMSET3[7:0] bits (Common memory 3 setup time) */
#define  FMC_PMEM3_MEMSET3_0                (0x00000001UL)        /*!<Bit 0 */
#define  FMC_PMEM3_MEMSET3_1                (0x00000002UL)        /*!<Bit 1 */
#define  FMC_PMEM3_MEMSET3_2                (0x00000004UL)        /*!<Bit 2 */
#define  FMC_PMEM3_MEMSET3_3                (0x00000008UL)        /*!<Bit 3 */
#define  FMC_PMEM3_MEMSET3_4                (0x00000010UL)        /*!<Bit 4 */
#define  FMC_PMEM3_MEMSET3_5                (0x00000020UL)        /*!<Bit 5 */
#define  FMC_PMEM3_MEMSET3_6                (0x00000040UL)        /*!<Bit 6 */
#define  FMC_PMEM3_MEMSET3_7                (0x00000080UL)        /*!<Bit 7 */

#define  FMC_PMEM3_MEMWAIT3                 (0x0000FF00UL)        /*!<MEMWAIT3[7:0] bits (Common memory 3 wait time) */
#define  FMC_PMEM3_MEMWAIT3_0               (0x00000100UL)        /*!<Bit 0 */
#define  FMC_PMEM3_MEMWAIT3_1               (0x00000200UL)        /*!<Bit 1 */
#define  FMC_PMEM3_MEMWAIT3_2               (0x00000400UL)        /*!<Bit 2 */
#define  FMC_PMEM3_MEMWAIT3_3               (0x00000800UL)        /*!<Bit 3 */
#define  FMC_PMEM3_MEMWAIT3_4               (0x00001000UL)        /*!<Bit 4 */
#define  FMC_PMEM3_MEMWAIT3_5               (0x00002000UL)        /*!<Bit 5 */
#define  FMC_PMEM3_MEMWAIT3_6               (0x00004000UL)        /*!<Bit 6 */
#define  FMC_PMEM3_MEMWAIT3_7               (0x00008000UL)        /*!<Bit 7 */

#define  FMC_PMEM3_MEMHOLD3                 (0x00FF0000UL)        /*!<MEMHOLD3[7:0] bits (Common memory 3 hold time) */
#define  FMC_PMEM3_MEMHOLD3_0               (0x00010000UL)        /*!<Bit 0 */
#define  FMC_PMEM3_MEMHOLD3_1               (0x00020000UL)        /*!<Bit 1 */
#define  FMC_PMEM3_MEMHOLD3_2               (0x00040000UL)        /*!<Bit 2 */
#define  FMC_PMEM3_MEMHOLD3_3               (0x00080000UL)        /*!<Bit 3 */
#define  FMC_PMEM3_MEMHOLD3_4               (0x00100000UL)        /*!<Bit 4 */
#define  FMC_PMEM3_MEMHOLD3_5               (0x00200000UL)        /*!<Bit 5 */
#define  FMC_PMEM3_MEMHOLD3_6               (0x00400000UL)        /*!<Bit 6 */
#define  FMC_PMEM3_MEMHOLD3_7               (0x00800000UL)        /*!<Bit 7 */

#define  FMC_PMEM3_MEMHIZ3                  (0xFF000000UL)        /*!<MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time) */
#define  FMC_PMEM3_MEMHIZ3_0                (0x01000000UL)        /*!<Bit 0 */
#define  FMC_PMEM3_MEMHIZ3_1                (0x02000000UL)        /*!<Bit 1 */
#define  FMC_PMEM3_MEMHIZ3_2                (0x04000000UL)        /*!<Bit 2 */
#define  FMC_PMEM3_MEMHIZ3_3                (0x08000000UL)        /*!<Bit 3 */
#define  FMC_PMEM3_MEMHIZ3_4                (0x10000000UL)        /*!<Bit 4 */
#define  FMC_PMEM3_MEMHIZ3_5                (0x20000000UL)        /*!<Bit 5 */
#define  FMC_PMEM3_MEMHIZ3_6                (0x40000000UL)        /*!<Bit 6 */
#define  FMC_PMEM3_MEMHIZ3_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FMC_PMEM4 register  ******************/
#define  FMC_PMEM4_MEMSET4                  (0x000000FFUL)        /*!<MEMSET4[7:0] bits (Common memory 4 setup time) */
#define  FMC_PMEM4_MEMSET4_0                (0x00000001UL)        /*!<Bit 0 */
#define  FMC_PMEM4_MEMSET4_1                (0x00000002UL)        /*!<Bit 1 */
#define  FMC_PMEM4_MEMSET4_2                (0x00000004UL)        /*!<Bit 2 */
#define  FMC_PMEM4_MEMSET4_3                (0x00000008UL)        /*!<Bit 3 */
#define  FMC_PMEM4_MEMSET4_4                (0x00000010UL)        /*!<Bit 4 */
#define  FMC_PMEM4_MEMSET4_5                (0x00000020UL)        /*!<Bit 5 */
#define  FMC_PMEM4_MEMSET4_6                (0x00000040UL)        /*!<Bit 6 */
#define  FMC_PMEM4_MEMSET4_7                (0x00000080UL)        /*!<Bit 7 */

#define  FMC_PMEM4_MEMWAIT4                 (0x0000FF00UL)        /*!<MEMWAIT4[7:0] bits (Common memory 4 wait time) */
#define  FMC_PMEM4_MEMWAIT4_0               (0x00000100UL)        /*!<Bit 0 */
#define  FMC_PMEM4_MEMWAIT4_1               (0x00000200UL)        /*!<Bit 1 */
#define  FMC_PMEM4_MEMWAIT4_2               (0x00000400UL)        /*!<Bit 2 */
#define  FMC_PMEM4_MEMWAIT4_3               (0x00000800UL)        /*!<Bit 3 */
#define  FMC_PMEM4_MEMWAIT4_4               (0x00001000UL)        /*!<Bit 4 */
#define  FMC_PMEM4_MEMWAIT4_5               (0x00002000UL)        /*!<Bit 5 */
#define  FMC_PMEM4_MEMWAIT4_6               (0x00004000UL)        /*!<Bit 6 */
#define  FMC_PMEM4_MEMWAIT4_7               (0x00008000UL)        /*!<Bit 7 */

#define  FMC_PMEM4_MEMHOLD4                 (0x00FF0000UL)        /*!<MEMHOLD4[7:0] bits (Common memory 4 hold time) */
#define  FMC_PMEM4_MEMHOLD4_0               (0x00010000UL)        /*!<Bit 0 */
#define  FMC_PMEM4_MEMHOLD4_1               (0x00020000UL)        /*!<Bit 1 */
#define  FMC_PMEM4_MEMHOLD4_2               (0x00040000UL)        /*!<Bit 2 */
#define  FMC_PMEM4_MEMHOLD4_3               (0x00080000UL)        /*!<Bit 3 */
#define  FMC_PMEM4_MEMHOLD4_4               (0x00100000UL)        /*!<Bit 4 */
#define  FMC_PMEM4_MEMHOLD4_5               (0x00200000UL)        /*!<Bit 5 */
#define  FMC_PMEM4_MEMHOLD4_6               (0x00400000UL)        /*!<Bit 6 */
#define  FMC_PMEM4_MEMHOLD4_7               (0x00800000UL)        /*!<Bit 7 */

#define  FMC_PMEM4_MEMHIZ4                  (0xFF000000UL)        /*!<MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time) */
#define  FMC_PMEM4_MEMHIZ4_0                (0x01000000UL)        /*!<Bit 0 */
#define  FMC_PMEM4_MEMHIZ4_1                (0x02000000UL)        /*!<Bit 1 */
#define  FMC_PMEM4_MEMHIZ4_2                (0x04000000UL)        /*!<Bit 2 */
#define  FMC_PMEM4_MEMHIZ4_3                (0x08000000UL)        /*!<Bit 3 */
#define  FMC_PMEM4_MEMHIZ4_4                (0x10000000UL)        /*!<Bit 4 */
#define  FMC_PMEM4_MEMHIZ4_5                (0x20000000UL)        /*!<Bit 5 */
#define  FMC_PMEM4_MEMHIZ4_6                (0x40000000UL)        /*!<Bit 6 */
#define  FMC_PMEM4_MEMHIZ4_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FMC_PATT2 register  ******************/
#define  FMC_PATT2_ATTSET2                  (0x000000FFUL)        /*!<ATTSET2[7:0] bits (Attribute memory 2 setup time) */
#define  FMC_PATT2_ATTSET2_0                (0x00000001UL)        /*!<Bit 0 */
#define  FMC_PATT2_ATTSET2_1                (0x00000002UL)        /*!<Bit 1 */
#define  FMC_PATT2_ATTSET2_2                (0x00000004UL)        /*!<Bit 2 */
#define  FMC_PATT2_ATTSET2_3                (0x00000008UL)        /*!<Bit 3 */
#define  FMC_PATT2_ATTSET2_4                (0x00000010UL)        /*!<Bit 4 */
#define  FMC_PATT2_ATTSET2_5                (0x00000020UL)        /*!<Bit 5 */
#define  FMC_PATT2_ATTSET2_6                (0x00000040UL)        /*!<Bit 6 */
#define  FMC_PATT2_ATTSET2_7                (0x00000080UL)        /*!<Bit 7 */

#define  FMC_PATT2_ATTWAIT2                 (0x0000FF00UL)        /*!<ATTWAIT2[7:0] bits (Attribute memory 2 wait time) */
#define  FMC_PATT2_ATTWAIT2_0               (0x00000100UL)        /*!<Bit 0 */
#define  FMC_PATT2_ATTWAIT2_1               (0x00000200UL)        /*!<Bit 1 */
#define  FMC_PATT2_ATTWAIT2_2               (0x00000400UL)        /*!<Bit 2 */
#define  FMC_PATT2_ATTWAIT2_3               (0x00000800UL)        /*!<Bit 3 */
#define  FMC_PATT2_ATTWAIT2_4               (0x00001000UL)        /*!<Bit 4 */
#define  FMC_PATT2_ATTWAIT2_5               (0x00002000UL)        /*!<Bit 5 */
#define  FMC_PATT2_ATTWAIT2_6               (0x00004000UL)        /*!<Bit 6 */
#define  FMC_PATT2_ATTWAIT2_7               (0x00008000UL)        /*!<Bit 7 */

#define  FMC_PATT2_ATTHOLD2                 (0x00FF0000UL)        /*!<ATTHOLD2[7:0] bits (Attribute memory 2 hold time) */
#define  FMC_PATT2_ATTHOLD2_0               (0x00010000UL)        /*!<Bit 0 */
#define  FMC_PATT2_ATTHOLD2_1               (0x00020000UL)        /*!<Bit 1 */
#define  FMC_PATT2_ATTHOLD2_2               (0x00040000UL)        /*!<Bit 2 */
#define  FMC_PATT2_ATTHOLD2_3               (0x00080000UL)        /*!<Bit 3 */
#define  FMC_PATT2_ATTHOLD2_4               (0x00100000UL)        /*!<Bit 4 */
#define  FMC_PATT2_ATTHOLD2_5               (0x00200000UL)        /*!<Bit 5 */
#define  FMC_PATT2_ATTHOLD2_6               (0x00400000UL)        /*!<Bit 6 */
#define  FMC_PATT2_ATTHOLD2_7               (0x00800000UL)        /*!<Bit 7 */

#define  FMC_PATT2_ATTHIZ2                  (0xFF000000UL)        /*!<ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time) */
#define  FMC_PATT2_ATTHIZ2_0                (0x01000000UL)        /*!<Bit 0 */
#define  FMC_PATT2_ATTHIZ2_1                (0x02000000UL)        /*!<Bit 1 */
#define  FMC_PATT2_ATTHIZ2_2                (0x04000000UL)        /*!<Bit 2 */
#define  FMC_PATT2_ATTHIZ2_3                (0x08000000UL)        /*!<Bit 3 */
#define  FMC_PATT2_ATTHIZ2_4                (0x10000000UL)        /*!<Bit 4 */
#define  FMC_PATT2_ATTHIZ2_5                (0x20000000UL)        /*!<Bit 5 */
#define  FMC_PATT2_ATTHIZ2_6                (0x40000000UL)        /*!<Bit 6 */
#define  FMC_PATT2_ATTHIZ2_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FMC_PATT3 register  ******************/
#define  FMC_PATT3_ATTSET3                  (0x000000FFUL)        /*!<ATTSET3[7:0] bits (Attribute memory 3 setup time) */
#define  FMC_PATT3_ATTSET3_0                (0x00000001UL)        /*!<Bit 0 */
#define  FMC_PATT3_ATTSET3_1                (0x00000002UL)        /*!<Bit 1 */
#define  FMC_PATT3_ATTSET3_2                (0x00000004UL)        /*!<Bit 2 */
#define  FMC_PATT3_ATTSET3_3                (0x00000008UL)        /*!<Bit 3 */
#define  FMC_PATT3_ATTSET3_4                (0x00000010UL)        /*!<Bit 4 */
#define  FMC_PATT3_ATTSET3_5                (0x00000020UL)        /*!<Bit 5 */
#define  FMC_PATT3_ATTSET3_6                (0x00000040UL)        /*!<Bit 6 */
#define  FMC_PATT3_ATTSET3_7                (0x00000080UL)        /*!<Bit 7 */

#define  FMC_PATT3_ATTWAIT3                 (0x0000FF00UL)        /*!<ATTWAIT3[7:0] bits (Attribute memory 3 wait time) */
#define  FMC_PATT3_ATTWAIT3_0               (0x00000100UL)        /*!<Bit 0 */
#define  FMC_PATT3_ATTWAIT3_1               (0x00000200UL)        /*!<Bit 1 */
#define  FMC_PATT3_ATTWAIT3_2               (0x00000400UL)        /*!<Bit 2 */
#define  FMC_PATT3_ATTWAIT3_3               (0x00000800UL)        /*!<Bit 3 */
#define  FMC_PATT3_ATTWAIT3_4               (0x00001000UL)        /*!<Bit 4 */
#define  FMC_PATT3_ATTWAIT3_5               (0x00002000UL)        /*!<Bit 5 */
#define  FMC_PATT3_ATTWAIT3_6               (0x00004000UL)        /*!<Bit 6 */
#define  FMC_PATT3_ATTWAIT3_7               (0x00008000UL)        /*!<Bit 7 */

#define  FMC_PATT3_ATTHOLD3                 (0x00FF0000UL)        /*!<ATTHOLD3[7:0] bits (Attribute memory 3 hold time) */
#define  FMC_PATT3_ATTHOLD3_0               (0x00010000UL)        /*!<Bit 0 */
#define  FMC_PATT3_ATTHOLD3_1               (0x00020000UL)        /*!<Bit 1 */
#define  FMC_PATT3_ATTHOLD3_2               (0x00040000UL)        /*!<Bit 2 */
#define  FMC_PATT3_ATTHOLD3_3               (0x00080000UL)        /*!<Bit 3 */
#define  FMC_PATT3_ATTHOLD3_4               (0x00100000UL)        /*!<Bit 4 */
#define  FMC_PATT3_ATTHOLD3_5               (0x00200000UL)        /*!<Bit 5 */
#define  FMC_PATT3_ATTHOLD3_6               (0x00400000UL)        /*!<Bit 6 */
#define  FMC_PATT3_ATTHOLD3_7               (0x00800000UL)        /*!<Bit 7 */

#define  FMC_PATT3_ATTHIZ3                  (0xFF000000UL)        /*!<ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time) */
#define  FMC_PATT3_ATTHIZ3_0                (0x01000000UL)        /*!<Bit 0 */
#define  FMC_PATT3_ATTHIZ3_1                (0x02000000UL)        /*!<Bit 1 */
#define  FMC_PATT3_ATTHIZ3_2                (0x04000000UL)        /*!<Bit 2 */
#define  FMC_PATT3_ATTHIZ3_3                (0x08000000UL)        /*!<Bit 3 */
#define  FMC_PATT3_ATTHIZ3_4                (0x10000000UL)        /*!<Bit 4 */
#define  FMC_PATT3_ATTHIZ3_5                (0x20000000UL)        /*!<Bit 5 */
#define  FMC_PATT3_ATTHIZ3_6                (0x40000000UL)        /*!<Bit 6 */
#define  FMC_PATT3_ATTHIZ3_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FMC_PATT4 register  ******************/
#define  FMC_PATT4_ATTSET4                  (0x000000FFUL)        /*!<ATTSET4[7:0] bits (Attribute memory 4 setup time) */
#define  FMC_PATT4_ATTSET4_0                (0x00000001UL)        /*!<Bit 0 */
#define  FMC_PATT4_ATTSET4_1                (0x00000002UL)        /*!<Bit 1 */
#define  FMC_PATT4_ATTSET4_2                (0x00000004UL)        /*!<Bit 2 */
#define  FMC_PATT4_ATTSET4_3                (0x00000008UL)        /*!<Bit 3 */
#define  FMC_PATT4_ATTSET4_4                (0x00000010UL)        /*!<Bit 4 */
#define  FMC_PATT4_ATTSET4_5                (0x00000020UL)        /*!<Bit 5 */
#define  FMC_PATT4_ATTSET4_6                (0x00000040UL)        /*!<Bit 6 */
#define  FMC_PATT4_ATTSET4_7                (0x00000080UL)        /*!<Bit 7 */

#define  FMC_PATT4_ATTWAIT4                 (0x0000FF00UL)        /*!<ATTWAIT4[7:0] bits (Attribute memory 4 wait time) */
#define  FMC_PATT4_ATTWAIT4_0               (0x00000100UL)        /*!<Bit 0 */
#define  FMC_PATT4_ATTWAIT4_1               (0x00000200UL)        /*!<Bit 1 */
#define  FMC_PATT4_ATTWAIT4_2               (0x00000400UL)        /*!<Bit 2 */
#define  FMC_PATT4_ATTWAIT4_3               (0x00000800UL)        /*!<Bit 3 */
#define  FMC_PATT4_ATTWAIT4_4               (0x00001000UL)        /*!<Bit 4 */
#define  FMC_PATT4_ATTWAIT4_5               (0x00002000UL)        /*!<Bit 5 */
#define  FMC_PATT4_ATTWAIT4_6               (0x00004000UL)        /*!<Bit 6 */
#define  FMC_PATT4_ATTWAIT4_7               (0x00008000UL)        /*!<Bit 7 */

#define  FMC_PATT4_ATTHOLD4                 (0x00FF0000UL)        /*!<ATTHOLD4[7:0] bits (Attribute memory 4 hold time) */
#define  FMC_PATT4_ATTHOLD4_0               (0x00010000UL)        /*!<Bit 0 */
#define  FMC_PATT4_ATTHOLD4_1               (0x00020000UL)        /*!<Bit 1 */
#define  FMC_PATT4_ATTHOLD4_2               (0x00040000UL)        /*!<Bit 2 */
#define  FMC_PATT4_ATTHOLD4_3               (0x00080000UL)        /*!<Bit 3 */
#define  FMC_PATT4_ATTHOLD4_4               (0x00100000UL)        /*!<Bit 4 */
#define  FMC_PATT4_ATTHOLD4_5               (0x00200000UL)        /*!<Bit 5 */
#define  FMC_PATT4_ATTHOLD4_6               (0x00400000UL)        /*!<Bit 6 */
#define  FMC_PATT4_ATTHOLD4_7               (0x00800000UL)        /*!<Bit 7 */

#define  FMC_PATT4_ATTHIZ4                  (0xFF000000UL)        /*!<ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time) */
#define  FMC_PATT4_ATTHIZ4_0                (0x01000000UL)        /*!<Bit 0 */
#define  FMC_PATT4_ATTHIZ4_1                (0x02000000UL)        /*!<Bit 1 */
#define  FMC_PATT4_ATTHIZ4_2                (0x04000000UL)        /*!<Bit 2 */
#define  FMC_PATT4_ATTHIZ4_3                (0x08000000UL)        /*!<Bit 3 */
#define  FMC_PATT4_ATTHIZ4_4                (0x10000000UL)        /*!<Bit 4 */
#define  FMC_PATT4_ATTHIZ4_5                (0x20000000UL)        /*!<Bit 5 */
#define  FMC_PATT4_ATTHIZ4_6                (0x40000000UL)        /*!<Bit 6 */
#define  FMC_PATT4_ATTHIZ4_7                (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FMC_PIO4 register  *******************/
#define  FMC_PIO4_IOSET4                    (0x000000FFUL)        /*!<IOSET4[7:0] bits (I/O 4 setup time) */
#define  FMC_PIO4_IOSET4_0                  (0x00000001UL)        /*!<Bit 0 */
#define  FMC_PIO4_IOSET4_1                  (0x00000002UL)        /*!<Bit 1 */
#define  FMC_PIO4_IOSET4_2                  (0x00000004UL)        /*!<Bit 2 */
#define  FMC_PIO4_IOSET4_3                  (0x00000008UL)        /*!<Bit 3 */
#define  FMC_PIO4_IOSET4_4                  (0x00000010UL)        /*!<Bit 4 */
#define  FMC_PIO4_IOSET4_5                  (0x00000020UL)        /*!<Bit 5 */
#define  FMC_PIO4_IOSET4_6                  (0x00000040UL)        /*!<Bit 6 */
#define  FMC_PIO4_IOSET4_7                  (0x00000080UL)        /*!<Bit 7 */

#define  FMC_PIO4_IOWAIT4                   (0x0000FF00UL)        /*!<IOWAIT4[7:0] bits (I/O 4 wait time) */
#define  FMC_PIO4_IOWAIT4_0                 (0x00000100UL)        /*!<Bit 0 */
#define  FMC_PIO4_IOWAIT4_1                 (0x00000200UL)        /*!<Bit 1 */
#define  FMC_PIO4_IOWAIT4_2                 (0x00000400UL)        /*!<Bit 2 */
#define  FMC_PIO4_IOWAIT4_3                 (0x00000800UL)        /*!<Bit 3 */
#define  FMC_PIO4_IOWAIT4_4                 (0x00001000UL)        /*!<Bit 4 */
#define  FMC_PIO4_IOWAIT4_5                 (0x00002000UL)        /*!<Bit 5 */
#define  FMC_PIO4_IOWAIT4_6                 (0x00004000UL)        /*!<Bit 6 */
#define  FMC_PIO4_IOWAIT4_7                 (0x00008000UL)        /*!<Bit 7 */

#define  FMC_PIO4_IOHOLD4                   (0x00FF0000UL)        /*!<IOHOLD4[7:0] bits (I/O 4 hold time) */
#define  FMC_PIO4_IOHOLD4_0                 (0x00010000UL)        /*!<Bit 0 */
#define  FMC_PIO4_IOHOLD4_1                 (0x00020000UL)        /*!<Bit 1 */
#define  FMC_PIO4_IOHOLD4_2                 (0x00040000UL)        /*!<Bit 2 */
#define  FMC_PIO4_IOHOLD4_3                 (0x00080000UL)        /*!<Bit 3 */
#define  FMC_PIO4_IOHOLD4_4                 (0x00100000UL)        /*!<Bit 4 */
#define  FMC_PIO4_IOHOLD4_5                 (0x00200000UL)        /*!<Bit 5 */
#define  FMC_PIO4_IOHOLD4_6                 (0x00400000UL)        /*!<Bit 6 */
#define  FMC_PIO4_IOHOLD4_7                 (0x00800000UL)        /*!<Bit 7 */

#define  FMC_PIO4_IOHIZ4                    (0xFF000000UL)        /*!<IOHIZ4[7:0] bits (I/O 4 databus HiZ time) */
#define  FMC_PIO4_IOHIZ4_0                  (0x01000000UL)        /*!<Bit 0 */
#define  FMC_PIO4_IOHIZ4_1                  (0x02000000UL)        /*!<Bit 1 */
#define  FMC_PIO4_IOHIZ4_2                  (0x04000000UL)        /*!<Bit 2 */
#define  FMC_PIO4_IOHIZ4_3                  (0x08000000UL)        /*!<Bit 3 */
#define  FMC_PIO4_IOHIZ4_4                  (0x10000000UL)        /*!<Bit 4 */
#define  FMC_PIO4_IOHIZ4_5                  (0x20000000UL)        /*!<Bit 5 */
#define  FMC_PIO4_IOHIZ4_6                  (0x40000000UL)        /*!<Bit 6 */
#define  FMC_PIO4_IOHIZ4_7                  (0x80000000UL)        /*!<Bit 7 */

/******************  Bit definition for FMC_ECCR2 register  ******************/
#define  FMC_ECCR2_ECC2                     (0xFFFFFFFFUL)        /*!<ECC result */

/******************  Bit definition for FMC_ECCR3 register  ******************/
#define  FMC_ECCR3_ECC3                     (0xFFFFFFFFUL)        /*!<ECC result */

/******************  Bit definition for FMC_SDCR1 register  ******************/
#define  FMC_SDCR1_NC                       (0x00000003UL)        /*!<NC[1:0] bits (Number of column bits) */
#define  FMC_SDCR1_NC_0                     (0x00000001UL)        /*!<Bit 0 */
#define  FMC_SDCR1_NC_1                     (0x00000002UL)        /*!<Bit 1 */

#define  FMC_SDCR1_NR                       (0x0000000CUL)        /*!<NR[1:0] bits (Number of row bits) */
#define  FMC_SDCR1_NR_0                     (0x00000004UL)        /*!<Bit 0 */
#define  FMC_SDCR1_NR_1                     (0x00000008UL)        /*!<Bit 1 */

#define  FMC_SDCR1_MWID                     (0x00000030UL)        /*!<NR[1:0] bits (Number of row bits) */
#define  FMC_SDCR1_MWID_0                   (0x00000010UL)        /*!<Bit 0 */
#define  FMC_SDCR1_MWID_1                   (0x00000020UL)        /*!<Bit 1 */

#define  FMC_SDCR1_NB                       (0x00000040UL)        /*!<Number of internal bank */

#define  FMC_SDCR1_CAS                      (0x00000180UL)        /*!<CAS[1:0] bits (CAS latency) */
#define  FMC_SDCR1_CAS_0                    (0x00000080UL)        /*!<Bit 0 */
#define  FMC_SDCR1_CAS_1                    (0x00000100UL)        /*!<Bit 1 */

#define  FMC_SDCR1_WP                       (0x00000200UL)        /*!<Write protection */

#define  FMC_SDCR1_SDCLK                    (0x00000C00UL)        /*!<SDRAM clock configuration */
#define  FMC_SDCR1_SDCLK_0                  (0x00000400UL)        /*!<Bit 0 */
#define  FMC_SDCR1_SDCLK_1                  (0x00000800UL)        /*!<Bit 1 */

#define  FMC_SDCR1_RBURST                   (0x00001000UL)        /*!<Read burst */

#define  FMC_SDCR1_RPIPE                    (0x00006000UL)        /*!<Write protection */
#define  FMC_SDCR1_RPIPE_0                  (0x00002000UL)        /*!<Bit 0 */
#define  FMC_SDCR1_RPIPE_1                  (0x00004000UL)        /*!<Bit 1 */

/******************  Bit definition for FMC_SDCR2 register  ******************/
#define  FMC_SDCR2_NC                       (0x00000003UL)        /*!<NC[1:0] bits (Number of column bits) */
#define  FMC_SDCR2_NC_0                     (0x00000001UL)        /*!<Bit 0 */
#define  FMC_SDCR2_NC_1                     (0x00000002UL)        /*!<Bit 1 */

#define  FMC_SDCR2_NR                       (0x0000000CUL)        /*!<NR[1:0] bits (Number of row bits) */
#define  FMC_SDCR2_NR_0                     (0x00000004UL)        /*!<Bit 0 */
#define  FMC_SDCR2_NR_1                     (0x00000008UL)        /*!<Bit 1 */

#define  FMC_SDCR2_MWID                     (0x00000030UL)        /*!<NR[1:0] bits (Number of row bits) */
#define  FMC_SDCR2_MWID_0                   (0x00000010UL)        /*!<Bit 0 */
#define  FMC_SDCR2_MWID_1                   (0x00000020UL)        /*!<Bit 1 */

#define  FMC_SDCR2_NB                       (0x00000040UL)        /*!<Number of internal bank */

#define  FMC_SDCR2_CAS                      (0x00000180UL)        /*!<CAS[1:0] bits (CAS latency) */
#define  FMC_SDCR2_CAS_0                    (0x00000080UL)        /*!<Bit 0 */
#define  FMC_SDCR2_CAS_1                    (0x00000100UL)        /*!<Bit 1 */

#define  FMC_SDCR2_WP                       (0x00000200UL)        /*!<Write protection */

#define  FMC_SDCR2_SDCLK                    (0x00000C00UL)        /*!<SDCLK[1:0] (SDRAM clock configuration) */
#define  FMC_SDCR2_SDCLK_0                  (0x00000400UL)        /*!<Bit 0 */
#define  FMC_SDCR2_SDCLK_1                  (0x00000800UL)        /*!<Bit 1 */

#define  FMC_SDCR2_RBURST                   (0x00001000UL)        /*!<Read burst */

#define  FMC_SDCR2_RPIPE                    (0x00006000UL)        /*!<RPIPE[1:0](Read pipe) */
#define  FMC_SDCR2_RPIPE_0                  (0x00002000UL)        /*!<Bit 0 */
#define  FMC_SDCR2_RPIPE_1                  (0x00004000UL)        /*!<Bit 1 */

/******************  Bit definition for FMC_SDTR1 register  ******************/
#define  FMC_SDTR1_TMRD                     (0x0000000FUL)        /*!<TMRD[3:0] bits (Load mode register to active) */
#define  FMC_SDTR1_TMRD_0                   (0x00000001UL)        /*!<Bit 0 */
#define  FMC_SDTR1_TMRD_1                   (0x00000002UL)        /*!<Bit 1 */
#define  FMC_SDTR1_TMRD_2                   (0x00000004UL)        /*!<Bit 2 */
#define  FMC_SDTR1_TMRD_3                   (0x00000008UL)        /*!<Bit 3 */

#define  FMC_SDTR1_TXSR                     (0x000000F0UL)        /*!<TXSR[3:0] bits (Exit self refresh) */
#define  FMC_SDTR1_TXSR_0                   (0x00000010UL)        /*!<Bit 0 */
#define  FMC_SDTR1_TXSR_1                   (0x00000020UL)        /*!<Bit 1 */
#define  FMC_SDTR1_TXSR_2                   (0x00000040UL)        /*!<Bit 2 */
#define  FMC_SDTR1_TXSR_3                   (0x00000080UL)        /*!<Bit 3 */

#define  FMC_SDTR1_TRAS                     (0x00000F00UL)        /*!<TRAS[3:0] bits (Self refresh time) */
#define  FMC_SDTR1_TRAS_0                   (0x00000100UL)        /*!<Bit 0 */
#define  FMC_SDTR1_TRAS_1                   (0x00000200UL)        /*!<Bit 1 */
#define  FMC_SDTR1_TRAS_2                   (0x00000400UL)        /*!<Bit 2 */
#define  FMC_SDTR1_TRAS_3                   (0x00000800UL)        /*!<Bit 3 */

#define  FMC_SDTR1_TRC                      (0x0000F000UL)        /*!<TRC[2:0] bits (Row cycle delay) */
#define  FMC_SDTR1_TRC_0                    (0x00001000UL)        /*!<Bit 0 */
#define  FMC_SDTR1_TRC_1                    (0x00002000UL)        /*!<Bit 1 */
#define  FMC_SDTR1_TRC_2                    (0x00004000UL)        /*!<Bit 2 */

#define  FMC_SDTR1_TWR                      (0x000F0000UL)        /*!<TRC[2:0] bits (Write recovery delay) */
#define  FMC_SDTR1_TWR_0                    (0x00010000UL)        /*!<Bit 0 */
#define  FMC_SDTR1_TWR_1                    (0x00020000UL)        /*!<Bit 1 */
#define  FMC_SDTR1_TWR_2                    (0x00040000UL)        /*!<Bit 2 */

#define  FMC_SDTR1_TRP                      (0x00F00000UL)        /*!<TRP[2:0] bits (Row precharge delay) */
#define  FMC_SDTR1_TRP_0                    (0x00100000UL)        /*!<Bit 0 */
#define  FMC_SDTR1_TRP_1                    (0x00200000UL)        /*!<Bit 1 */
#define  FMC_SDTR1_TRP_2                    (0x00400000UL)        /*!<Bit 2 */

#define  FMC_SDTR1_TRCD                     (0x0F000000UL)        /*!<TRP[2:0] bits (Row to column delay) */
#define  FMC_SDTR1_TRCD_0                   (0x01000000UL)        /*!<Bit 0 */
#define  FMC_SDTR1_TRCD_1                   (0x02000000UL)        /*!<Bit 1 */
#define  FMC_SDTR1_TRCD_2                   (0x04000000UL)        /*!<Bit 2 */

/******************  Bit definition for FMC_SDTR2 register  ******************/
#define  FMC_SDTR2_TMRD                     (0x0000000FUL)        /*!<TMRD[3:0] bits (Load mode register to active) */
#define  FMC_SDTR2_TMRD_0                   (0x00000001UL)        /*!<Bit 0 */
#define  FMC_SDTR2_TMRD_1                   (0x00000002UL)        /*!<Bit 1 */
#define  FMC_SDTR2_TMRD_2                   (0x00000004UL)        /*!<Bit 2 */
#define  FMC_SDTR2_TMRD_3                   (0x00000008UL)        /*!<Bit 3 */

#define  FMC_SDTR2_TXSR                     (0x000000F0UL)        /*!<TXSR[3:0] bits (Exit self refresh) */
#define  FMC_SDTR2_TXSR_0                   (0x00000010UL)        /*!<Bit 0 */
#define  FMC_SDTR2_TXSR_1                   (0x00000020UL)        /*!<Bit 1 */
#define  FMC_SDTR2_TXSR_2                   (0x00000040UL)        /*!<Bit 2 */
#define  FMC_SDTR2_TXSR_3                   (0x00000080UL)        /*!<Bit 3 */

#define  FMC_SDTR2_TRAS                     (0x00000F00UL)        /*!<TRAS[3:0] bits (Self refresh time) */
#define  FMC_SDTR2_TRAS_0                   (0x00000100UL)        /*!<Bit 0 */
#define  FMC_SDTR2_TRAS_1                   (0x00000200UL)        /*!<Bit 1 */
#define  FMC_SDTR2_TRAS_2                   (0x00000400UL)        /*!<Bit 2 */
#define  FMC_SDTR2_TRAS_3                   (0x00000800UL)        /*!<Bit 3 */

#define  FMC_SDTR2_TRC                      (0x0000F000UL)        /*!<TRC[2:0] bits (Row cycle delay) */
#define  FMC_SDTR2_TRC_0                    (0x00001000UL)        /*!<Bit 0 */
#define  FMC_SDTR2_TRC_1                    (0x00002000UL)        /*!<Bit 1 */
#define  FMC_SDTR2_TRC_2                    (0x00004000UL)        /*!<Bit 2 */

#define  FMC_SDTR2_TWR                      (0x000F0000UL)        /*!<TRC[2:0] bits (Write recovery delay) */
#define  FMC_SDTR2_TWR_0                    (0x00010000UL)        /*!<Bit 0 */
#define  FMC_SDTR2_TWR_1                    (0x00020000UL)        /*!<Bit 1 */
#define  FMC_SDTR2_TWR_2                    (0x00040000UL)        /*!<Bit 2 */

#define  FMC_SDTR2_TRP                      (0x00F00000UL)        /*!<TRP[2:0] bits (Row precharge delay) */
#define  FMC_SDTR2_TRP_0                    (0x00100000UL)        /*!<Bit 0 */
#define  FMC_SDTR2_TRP_1                    (0x00200000UL)        /*!<Bit 1 */
#define  FMC_SDTR2_TRP_2                    (0x00400000UL)        /*!<Bit 2 */

#define  FMC_SDTR2_TRCD                     (0x0F000000UL)        /*!<TRP[2:0] bits (Row to column delay) */
#define  FMC_SDTR2_TRCD_0                   (0x01000000UL)        /*!<Bit 0 */
#define  FMC_SDTR2_TRCD_1                   (0x02000000UL)        /*!<Bit 1 */
#define  FMC_SDTR2_TRCD_2                   (0x04000000UL)        /*!<Bit 2 */

/******************  Bit definition for FMC_SDCMR register  ******************/
#define  FMC_SDCMR_MODE                     (0x00000007UL)        /*!<MODE[2:0] bits (Command mode) */
#define  FMC_SDCMR_MODE_0                   (0x00000001UL)        /*!<Bit 0 */
#define  FMC_SDCMR_MODE_1                   (0x00000002UL)        /*!<Bit 1 */
#define  FMC_SDCMR_MODE_2                   (0x00000003UL)        /*!<Bit 2 */

#define  FMC_SDCMR_CTB2                     (0x00000008UL)        /*!<Command target 2 */

#define  FMC_SDCMR_CTB1                     (0x00000010UL)        /*!<Command target 1 */

#define  FMC_SDCMR_NRFS                     (0x000001E0UL)        /*!<NRFS[3:0] bits (Number of auto-refresh) */
#define  FMC_SDCMR_NRFS_0                   (0x00000020UL)        /*!<Bit 0 */
#define  FMC_SDCMR_NRFS_1                   (0x00000040UL)        /*!<Bit 1 */
#define  FMC_SDCMR_NRFS_2                   (0x00000080UL)        /*!<Bit 2 */
#define  FMC_SDCMR_NRFS_3                   (0x00000100UL)        /*!<Bit 3 */

#define  FMC_SDCMR_MRD                      (0x003FFE00UL)        /*!<MRD[12:0] bits (Mode register definition) */

/******************  Bit definition for FMC_SDRTR register  ******************/
#define  FMC_SDRTR_CRE                      (0x00000001UL)        /*!<Clear refresh error flag */

#define  FMC_SDRTR_COUNT                    (0x00003FFEUL)        /*!<COUNT[12:0] bits (Refresh timer count) */

#define  FMC_SDRTR_REIE                     (0x00004000UL)        /*!<RES interupt enable */

/******************  Bit definition for FMC_SDSR register  ******************/
#define  FMC_SDSR_RE                        (0x00000001UL)        /*!<Refresh error flag */

#define  FMC_SDSR_MODES1                    (0x00000006UL)        /*!<MODES1[1:0]bits (Status mode for bank 1) */
#define  FMC_SDSR_MODES1_0                  (0x00000002UL)        /*!<Bit 0 */
#define  FMC_SDSR_MODES1_1                  (0x00000004UL)        /*!<Bit 1 */

#define  FMC_SDSR_MODES2                    (0x00000018UL)        /*!<MODES2[1:0]bits (Status mode for bank 2) */
#define  FMC_SDSR_MODES2_0                  (0x00000008UL)        /*!<Bit 0 */
#define  FMC_SDSR_MODES2_1                  (0x00000010UL)        /*!<Bit 1 */

#define  FMC_SDSR_BUSY                      (0x00000020UL)        /*!<Busy status */

#endif /* STM32F427_437xx ||  STM32F429_439xx || STM32F446xx || STM32F469_479xx */

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

/******************************************************************************/
/*                                                                            */
/*                                    HASH                                    */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for HASH_CR register  ********************/
#define HASH_CR_INIT                         (0x00000004UL)
#define HASH_CR_DMAE                         (0x00000008UL)
#define HASH_CR_DATATYPE                     (0x00000030UL)
#define HASH_CR_DATATYPE_0                   (0x00000010UL)
#define HASH_CR_DATATYPE_1                   (0x00000020UL)
#define HASH_CR_MODE                         (0x00000040UL)
#define HASH_CR_ALGO                         (0x00040080UL)
#define HASH_CR_ALGO_0                       (0x00000080UL)
#define HASH_CR_ALGO_1                       (0x00040000UL)
#define HASH_CR_NBW                          (0x00000F00UL)
#define HASH_CR_NBW_0                        (0x00000100UL)
#define HASH_CR_NBW_1                        (0x00000200UL)
#define HASH_CR_NBW_2                        (0x00000400UL)
#define HASH_CR_NBW_3                        (0x00000800UL)
#define HASH_CR_DINNE                        (0x00001000UL)
#define HASH_CR_MDMAT                        (0x00002000UL)
#define HASH_CR_LKEY                         (0x00010000UL)

/******************  Bits definition for HASH_STR register  *******************/
#define HASH_STR_NBW                         (0x0000001FUL)
#define HASH_STR_NBW_0                       (0x00000001UL)
#define HASH_STR_NBW_1                       (0x00000002UL)
#define HASH_STR_NBW_2                       (0x00000004UL)
#define HASH_STR_NBW_3                       (0x00000008UL)
#define HASH_STR_NBW_4                       (0x00000010UL)
#define HASH_STR_DCAL                        (0x00000100UL)

/******************  Bits definition for HASH_IMR register  *******************/
#define HASH_IMR_DINIM                       (0x00000001UL)
#define HASH_IMR_DCIM                        (0x00000002UL)

/******************  Bits definition for HASH_SR register  ********************/
#define HASH_SR_DINIS                        (0x00000001UL)
#define HASH_SR_DCIS                         (0x00000002UL)
#define HASH_SR_DMAS                         (0x00000004UL)
#define HASH_SR_BUSY                         (0x00000008UL)

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  ********************/
#define  I2C_CR1_PE                          (0x0001U)            /*!<Peripheral Enable                             */
#define  I2C_CR1_SMBUS                       (0x0002U)            /*!<SMBus Mode                                    */
#define  I2C_CR1_SMBTYPE                     (0x0008U)            /*!<SMBus Type                                    */
#define  I2C_CR1_ENARP                       (0x0010U)            /*!<ARP Enable                                    */
#define  I2C_CR1_ENPEC                       (0x0020U)            /*!<PEC Enable                                    */
#define  I2C_CR1_ENGC                        (0x0040U)            /*!<General Call Enable                           */
#define  I2C_CR1_NOSTRETCH                   (0x0080U)            /*!<Clock Stretching Disable (Slave mode)         */
#define  I2C_CR1_START                       (0x0100U)            /*!<Start Generation                              */
#define  I2C_CR1_STOP                        (0x0200U)            /*!<Stop Generation                               */
#define  I2C_CR1_ACK                         (0x0400U)            /*!<Acknowledge Enable                            */
#define  I2C_CR1_POS                         (0x0800U)            /*!<Acknowledge/PEC Position (for data reception) */
#define  I2C_CR1_PEC                         (0x1000U)            /*!<Packet Error Checking                         */
#define  I2C_CR1_ALERT                       (0x2000U)            /*!<SMBus Alert                                   */
#define  I2C_CR1_SWRST                       (0x8000U)            /*!<Software Reset                                */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_FREQ                        (0x003FU)            /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
#define  I2C_CR2_FREQ_0                      (0x0001U)            /*!<Bit 0 */
#define  I2C_CR2_FREQ_1                      (0x0002U)            /*!<Bit 1 */
#define  I2C_CR2_FREQ_2                      (0x0004U)            /*!<Bit 2 */
#define  I2C_CR2_FREQ_3                      (0x0008U)            /*!<Bit 3 */
#define  I2C_CR2_FREQ_4                      (0x0010U)            /*!<Bit 4 */
#define  I2C_CR2_FREQ_5                      (0x0020U)            /*!<Bit 5 */

#define  I2C_CR2_ITERREN                     (0x0100U)            /*!<Error Interrupt Enable  */
#define  I2C_CR2_ITEVTEN                     (0x0200U)            /*!<Event Interrupt Enable  */
#define  I2C_CR2_ITBUFEN                     (0x0400U)            /*!<Buffer Interrupt Enable */
#define  I2C_CR2_DMAEN                       (0x0800U)            /*!<DMA Requests Enable     */
#define  I2C_CR2_LAST                        (0x1000U)            /*!<DMA Last Transfer       */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define  I2C_OAR1_ADD1_7                     (0x00FEU)            /*!<Interface Address */
#define  I2C_OAR1_ADD8_9                     (0x0300U)            /*!<Interface Address */

#define  I2C_OAR1_ADD0                       (0x0001U)            /*!<Bit 0 */
#define  I2C_OAR1_ADD1                       (0x0002U)            /*!<Bit 1 */
#define  I2C_OAR1_ADD2                       (0x0004U)            /*!<Bit 2 */
#define  I2C_OAR1_ADD3                       (0x0008U)            /*!<Bit 3 */
#define  I2C_OAR1_ADD4                       (0x0010U)            /*!<Bit 4 */
#define  I2C_OAR1_ADD5                       (0x0020U)            /*!<Bit 5 */
#define  I2C_OAR1_ADD6                       (0x0040U)            /*!<Bit 6 */
#define  I2C_OAR1_ADD7                       (0x0080U)            /*!<Bit 7 */
#define  I2C_OAR1_ADD8                       (0x0100U)            /*!<Bit 8 */
#define  I2C_OAR1_ADD9                       (0x0200U)            /*!<Bit 9 */

#define  I2C_OAR1_ADDMODE                    (0x8000U)            /*!<Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  I2C_OAR2_ENDUAL                     (0x01U)               /*!<Dual addressing mode enable */
#define  I2C_OAR2_ADD2                       (0xFEU)               /*!<Interface address           */

/********************  Bit definition for I2C_DR register  ********************/
#define  I2C_DR_DR                           (0xFFU)               /*!<8-bit Data Register         */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define  I2C_SR1_SB                          (0x0001U)            /*!<Start Bit (Master mode)                         */
#define  I2C_SR1_ADDR                        (0x0002U)            /*!<Address sent (master mode)/matched (slave mode) */
#define  I2C_SR1_BTF                         (0x0004U)            /*!<Byte Transfer Finished                          */
#define  I2C_SR1_ADD10                       (0x0008U)            /*!<10-bit header sent (Master mode)                */
#define  I2C_SR1_STOPF                       (0x0010U)            /*!<Stop detection (Slave mode)                     */
#define  I2C_SR1_RXNE                        (0x0040U)            /*!<Data Register not Empty (receivers)             */
#define  I2C_SR1_TXE                         (0x0080U)            /*!<Data Register Empty (transmitters)              */
#define  I2C_SR1_BERR                        (0x0100U)            /*!<Bus Error                                       */
#define  I2C_SR1_ARLO                        (0x0200U)            /*!<Arbitration Lost (master mode)                  */
#define  I2C_SR1_AF                          (0x0400U)            /*!<Acknowledge Failure                             */
#define  I2C_SR1_OVR                         (0x0800U)            /*!<Overrun/Underrun                                */
#define  I2C_SR1_PECERR                      (0x1000U)            /*!<PEC Error in reception                          */
#define  I2C_SR1_TIMEOUT                     (0x4000U)            /*!<Timeout or Tlow Error                           */
#define  I2C_SR1_SMBALERT                    (0x8000U)            /*!<SMBus Alert                                     */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define  I2C_SR2_MSL                         (0x0001U)            /*!<Master/Slave                              */
#define  I2C_SR2_BUSY                        (0x0002U)            /*!<Bus Busy                                  */
#define  I2C_SR2_TRA                         (0x0004U)            /*!<Transmitter/Receiver                      */
#define  I2C_SR2_GENCALL                     (0x0010U)            /*!<General Call Address (Slave mode)         */
#define  I2C_SR2_SMBDEFAULT                  (0x0020U)            /*!<SMBus Device Default Address (Slave mode) */
#define  I2C_SR2_SMBHOST                     (0x0040U)            /*!<SMBus Host Header (Slave mode)            */
#define  I2C_SR2_DUALF                       (0x0080U)            /*!<Dual Flag (Slave mode)                    */
#define  I2C_SR2_PEC                         (0xFF00U)            /*!<Packet Error Checking Register            */

/*******************  Bit definition for I2C_CCR register  ********************/
#define  I2C_CCR_CCR                         (0x0FFFU)            /*!<Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CCR_DUTY                        (0x4000U)            /*!<Fast Mode Duty Cycle                                       */
#define  I2C_CCR_FS                          (0x8000U)            /*!<I2C Master Mode Selection                                  */

/******************  Bit definition for I2C_TRISE register  *******************/
#define  I2C_TRISE_TRISE                     (0x3FU)               /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************  Bit definition for I2C_FLTR register  *******************/
#define  I2C_FLTR_DNF                     (0x0FU)                  /*!<Digital Noise Filter */
#define  I2C_FLTR_ANOFF                   (0x10U)                  /*!<Analog Noise Filter OFF */

#if defined(STM32F410xx) || defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
/******************************************************************************/
/*                                                                            */
/*              Fast-mode Plus Inter-integrated circuit (FMPI2C)              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  *******************/
#define  FMPI2C_CR1_PE                          (0x00000001UL)        /*!< Peripheral enable                   */
#define  FMPI2C_CR1_TXIE                        (0x00000002UL)        /*!< TX interrupt enable                 */
#define  FMPI2C_CR1_RXIE                        (0x00000004UL)        /*!< RX interrupt enable                 */
#define  FMPI2C_CR1_ADDRIE                      (0x00000008UL)        /*!< Address match interrupt enable      */
#define  FMPI2C_CR1_NACKIE                      (0x00000010UL)        /*!< NACK received interrupt enable      */
#define  FMPI2C_CR1_STOPIE                      (0x00000020UL)        /*!< STOP detection interrupt enable     */
#define  FMPI2C_CR1_TCIE                        (0x00000040UL)        /*!< Transfer complete interrupt enable  */
#define  FMPI2C_CR1_ERRIE                       (0x00000080UL)        /*!< Errors interrupt enable             */
#define  FMPI2C_CR1_DFN                         (0x00000F00UL)        /*!< Digital noise filter                */
#define  FMPI2C_CR1_ANFOFF                      (0x00001000UL)        /*!< Analog noise filter OFF             */
#define  FMPI2C_CR1_TXDMAEN                     (0x00004000UL)        /*!< DMA transmission requests enable    */
#define  FMPI2C_CR1_RXDMAEN                     (0x00008000UL)        /*!< DMA reception requests enable       */
#define  FMPI2C_CR1_SBC                         (0x00010000UL)        /*!< Slave byte control                  */
#define  FMPI2C_CR1_NOSTRETCH                   (0x00020000UL)        /*!< Clock stretching disable            */
#define  FMPI2C_CR1_GCEN                        (0x00080000UL)        /*!< General call enable                 */
#define  FMPI2C_CR1_SMBHEN                      (0x00100000UL)        /*!< SMBus host address enable           */
#define  FMPI2C_CR1_SMBDEN                      (0x00200000UL)        /*!< SMBus device default address enable */
#define  FMPI2C_CR1_ALERTEN                     (0x00400000UL)        /*!< SMBus alert enable                  */
#define  FMPI2C_CR1_PECEN                       (0x00800000UL)        /*!< PEC enable                          */

/******************  Bit definition for I2C_CR2 register  ********************/
#define  FMPI2C_CR2_SADD                        (0x000003FFUL)        /*!< Slave address (master mode)                             */
#define  FMPI2C_CR2_RD_WRN                      (0x00000400UL)        /*!< Transfer direction (master mode)                        */
#define  FMPI2C_CR2_ADD10                       (0x00000800UL)        /*!< 10-bit addressing mode (master mode)                    */
#define  FMPI2C_CR2_HEAD10R                     (0x00001000UL)        /*!< 10-bit address header only read direction (master mode) */
#define  FMPI2C_CR2_START                       (0x00002000UL)        /*!< START generation                                        */
#define  FMPI2C_CR2_STOP                        (0x00004000UL)        /*!< STOP generation (master mode)                           */
#define  FMPI2C_CR2_NACK                        (0x00008000UL)        /*!< NACK generation (slave mode)                            */
#define  FMPI2C_CR2_NBYTES                      (0x00FF0000UL)        /*!< Number of bytes                                         */
#define  FMPI2C_CR2_RELOAD                      (0x01000000UL)        /*!< NBYTES reload mode                                      */
#define  FMPI2C_CR2_AUTOEND                     (0x02000000UL)        /*!< Automatic end mode (master mode)                        */
#define  FMPI2C_CR2_PECBYTE                     (0x04000000UL)        /*!< Packet error checking byte                              */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define  FMPI2C_OAR1_OA1                        (0x000003FFUL)        /*!< Interface own address 1   */
#define  FMPI2C_OAR1_OA1MODE                    (0x00000400UL)        /*!< Own address 1 10-bit mode */
#define  FMPI2C_OAR1_OA1EN                      (0x00008000UL)        /*!< Own address 1 enable     */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  FMPI2C_OAR2_OA2                        (0x000000FEUL)        /*!< Interface own address 2 */
#define  FMPI2C_OAR2_OA2MSK                     (0x00000700UL)        /*!< Own address 2 masks     */
#define  FMPI2C_OAR2_OA2EN                      (0x00008000UL)        /*!< Own address 2 enable    */

/*******************  Bit definition for I2C_TIMINGR register *****************/
#define  FMPI2C_TIMINGR_SCLL                    (0x000000FFUL)        /*!< SCL low period (master mode)  */
#define  FMPI2C_TIMINGR_SCLH                    (0x0000FF00UL)        /*!< SCL high period (master mode) */
#define  FMPI2C_TIMINGR_SDADEL                  (0x000F0000UL)        /*!< Data hold time                */
#define  FMPI2C_TIMINGR_SCLDEL                  (0x00F00000UL)        /*!< Data setup time               */
#define  FMPI2C_TIMINGR_PRESC                   (0xF0000000UL)        /*!< Timings prescaler             */

/******************* Bit definition for I2C_TIMEOUTR register *****************/
#define  FMPI2C_TIMEOUTR_TIMEOUTA               (0x00000FFFUL)        /*!< Bus timeout A                 */
#define  FMPI2C_TIMEOUTR_TIDLE                  (0x00001000UL)        /*!< Idle clock timeout detection  */
#define  FMPI2C_TIMEOUTR_TIMOUTEN               (0x00008000UL)        /*!< Clock timeout enable          */
#define  FMPI2C_TIMEOUTR_TIMEOUTB               (0x0FFF0000UL)        /*!< Bus timeout B                 */
#define  FMPI2C_TIMEOUTR_TEXTEN                 (0x80000000UL)        /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  *********************/
#define  FMPI2C_ISR_TXE                         (0x00000001UL)        /*!< Transmit data register empty    */
#define  FMPI2C_ISR_TXIS                        (0x00000002UL)        /*!< Transmit interrupt status       */
#define  FMPI2C_ISR_RXNE                        (0x00000004UL)        /*!< Receive data register not empty */
#define  FMPI2C_ISR_ADDR                        (0x00000008UL)        /*!< Address matched (slave mode)    */
#define  FMPI2C_ISR_NACKF                       (0x00000010UL)        /*!< NACK received flag              */
#define  FMPI2C_ISR_STOPF                       (0x00000020UL)        /*!< STOP detection flag             */
#define  FMPI2C_ISR_TC                          (0x00000040UL)        /*!< Transfer complete (master mode) */
#define  FMPI2C_ISR_TCR                         (0x00000080UL)        /*!< Transfer complete reload        */
#define  FMPI2C_ISR_BERR                        (0x00000100UL)        /*!< Bus error                       */
#define  FMPI2C_ISR_ARLO                        (0x00000200UL)        /*!< Arbitration lost                */
#define  FMPI2C_ISR_OVR                         (0x00000400UL)        /*!< Overrun/Underrun                */
#define  FMPI2C_ISR_PECERR                      (0x00000800UL)        /*!< PEC error in reception          */
#define  FMPI2C_ISR_TIMEOUT                     (0x00001000UL)        /*!< Timeout or Tlow detection flag  */
#define  FMPI2C_ISR_ALERT                       (0x00002000UL)        /*!< SMBus alert                     */
#define  FMPI2C_ISR_BUSY                        (0x00008000UL)        /*!< Bus busy                        */
#define  FMPI2C_ISR_DIR                         (0x00010000UL)        /*!< Transfer direction (slave mode) */
#define  FMPI2C_ISR_ADDCODE                     (0x00FE0000UL)        /*!< Address match code (slave mode) */

/******************  Bit definition for I2C_ICR register  *********************/
#define  FMPI2C_ICR_ADDRCF                      (0x00000008UL)        /*!< Address matched clear flag  */
#define  FMPI2C_ICR_NACKCF                      (0x00000010UL)        /*!< NACK clear flag             */
#define  FMPI2C_ICR_STOPCF                      (0x00000020UL)        /*!< STOP detection clear flag   */
#define  FMPI2C_ICR_BERRCF                      (0x00000100UL)        /*!< Bus error clear flag */
#define  FMPI2C_ICR_ARLOCF                      (0x00000200UL)        /*!< Arbitration lost clear flag */
#define  FMPI2C_ICR_OVRCF                       (0x00000400UL)        /*!< Overrun/Underrun clear flag */
#define  FMPI2C_ICR_PECCF                       (0x00000800UL)        /*!< PAC error clear flag        */
#define  FMPI2C_ICR_TIMOUTCF                    (0x00001000UL)        /*!< Timeout clear flag          */
#define  FMPI2C_ICR_ALERTCF                     (0x00002000UL)        /*!< Alert clear flag            */

/******************  Bit definition for I2C_PECR register  ********************/
#define  FMPI2C_PECR_PEC                        (0x000000FFUL)        /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define  FMPI2C_RXDR_RXDATA                     (0x000000FFUL)        /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *********************/
#define  FMPI2C_TXDR_TXDATA                     (0x000000FFUL)        /*!< 8-bit transmit data */
#endif /* STM32F410xx || STM32F412xG || STM32F413_423xx || STM32F446xx */
/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         (0xFFFFU)            /*!<Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          (0x07U)               /*!<PR[2:0] (Prescaler divider)         */
#define  IWDG_PR_PR_0                        (0x01U)               /*!<Bit 0 */
#define  IWDG_PR_PR_1                        (0x02U)               /*!<Bit 1 */
#define  IWDG_PR_PR_2                        (0x04U)               /*!<Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         (0x0FFFU)            /*!<Watchdog counter reload value        */

/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         (0x01U)               /*!<Watchdog prescaler value update      */
#define  IWDG_SR_RVU                         (0x02U)               /*!<Watchdog counter reload value update */

/******************************************************************************/
/*                                                                            */
/*                      LCD-TFT Display Controller (LTDC)                     */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for LTDC_SSCR register  *****************/

#define LTDC_SSCR_VSH                       (0x000007FFUL)              /*!< Vertical Synchronization Height */
#define LTDC_SSCR_HSW                       (0x0FFF0000UL)              /*!< Horizontal Synchronization Width */

/********************  Bit definition for LTDC_BPCR register  *****************/

#define LTDC_BPCR_AVBP                      (0x000007FFUL)              /*!< Accumulated Vertical Back Porch */
#define LTDC_BPCR_AHBP                      (0x0FFF0000UL)              /*!< Accumulated Horizontal Back Porch */

/********************  Bit definition for LTDC_AWCR register  *****************/

#define LTDC_AWCR_AAH                       (0x000007FFUL)              /*!< Accumulated Active heigh */
#define LTDC_AWCR_AAW                       (0x0FFF0000UL)              /*!< Accumulated Active Width */

/********************  Bit definition for LTDC_TWCR register  *****************/

#define LTDC_TWCR_TOTALH                    (0x000007FFUL)              /*!< Total Heigh */
#define LTDC_TWCR_TOTALW                    (0x0FFF0000UL)              /*!< Total Width */

/********************  Bit definition for LTDC_GCR register  ******************/

#define LTDC_GCR_LTDCEN                     (0x00000001UL)              /*!< LCD-TFT controller enable bit */
#define LTDC_GCR_DBW                        (0x00000070UL)              /*!< Dither Blue Width */
#define LTDC_GCR_DGW                        (0x00000700UL)              /*!< Dither Green Width */
#define LTDC_GCR_DRW                        (0x00007000UL)              /*!< Dither Red Width */
#define LTDC_GCR_DEN                        (0x00010000UL)              /*!< Dither Enable */
#define LTDC_GCR_PCPOL                      (0x10000000UL)              /*!< Pixel Clock Polarity */
#define LTDC_GCR_DEPOL                      (0x20000000UL)              /*!< Data Enable Polarity */
#define LTDC_GCR_VSPOL                      (0x40000000UL)              /*!< Vertical Synchronization Polarity */
#define LTDC_GCR_HSPOL                      (0x80000000UL)              /*!< Horizontal Synchronization Polarity */

/* Legacy defines */
#define LTDC_GCR_DTEN                       LTDC_GCR_DEN

/********************  Bit definition for LTDC_SRCR register  *****************/

#define LTDC_SRCR_IMR                      (0x00000001UL)               /*!< Immediate Reload */
#define LTDC_SRCR_VBR                      (0x00000002UL)               /*!< Vertical Blanking Reload */

/********************  Bit definition for LTDC_BCCR register  *****************/

#define LTDC_BCCR_BCBLUE                    (0x000000FFUL)              /*!< Background Blue value */
#define LTDC_BCCR_BCGREEN                   (0x0000FF00UL)              /*!< Background Green value */
#define LTDC_BCCR_BCRED                     (0x00FF0000UL)              /*!< Background Red value */

/********************  Bit definition for LTDC_IER register  ******************/

#define LTDC_IER_LIE                        (0x00000001UL)              /*!< Line Interrupt Enable */
#define LTDC_IER_FUIE                       (0x00000002UL)              /*!< FIFO Underrun Interrupt Enable */
#define LTDC_IER_TERRIE                     (0x00000004UL)              /*!< Transfer Error Interrupt Enable */
#define LTDC_IER_RRIE                       (0x00000008UL)              /*!< Register Reload interrupt enable */

/********************  Bit definition for LTDC_ISR register  ******************/

#define LTDC_ISR_LIF                        (0x00000001UL)              /*!< Line Interrupt Flag */
#define LTDC_ISR_FUIF                       (0x00000002UL)              /*!< FIFO Underrun Interrupt Flag */
#define LTDC_ISR_TERRIF                     (0x00000004UL)              /*!< Transfer Error Interrupt Flag */
#define LTDC_ISR_RRIF                       (0x00000008UL)              /*!< Register Reload interrupt Flag */

/********************  Bit definition for LTDC_ICR register  ******************/

#define LTDC_ICR_CLIF                       (0x00000001UL)              /*!< Clears the Line Interrupt Flag */
#define LTDC_ICR_CFUIF                      (0x00000002UL)              /*!< Clears the FIFO Underrun Interrupt Flag */
#define LTDC_ICR_CTERRIF                    (0x00000004UL)              /*!< Clears the Transfer Error Interrupt Flag */
#define LTDC_ICR_CRRIF                      (0x00000008UL)              /*!< Clears Register Reload interrupt Flag */

/********************  Bit definition for LTDC_LIPCR register  ****************/

#define LTDC_LIPCR_LIPOS                    (0x000007FFUL)              /*!< Line Interrupt Position */

/********************  Bit definition for LTDC_CPSR register  *****************/

#define LTDC_CPSR_CYPOS                     (0x0000FFFFUL)              /*!< Current Y Position */
#define LTDC_CPSR_CXPOS                     (0xFFFF0000UL)              /*!< Current X Position */

/********************  Bit definition for LTDC_CDSR register  *****************/

#define LTDC_CDSR_VDES                      (0x00000001UL)              /*!< Vertical Data Enable Status */
#define LTDC_CDSR_HDES                      (0x00000002UL)              /*!< Horizontal Data Enable Status */
#define LTDC_CDSR_VSYNCS                    (0x00000004UL)              /*!< Vertical Synchronization Status */
#define LTDC_CDSR_HSYNCS                    (0x00000008UL)              /*!< Horizontal Synchronization Status */

/********************  Bit definition for LTDC_LxCR register  *****************/

#define LTDC_LxCR_LEN                       (0x00000001UL)              /*!< Layer Enable */
#define LTDC_LxCR_COLKEN                    (0x00000002UL)              /*!< Color Keying Enable */
#define LTDC_LxCR_CLUTEN                    (0x00000010UL)              /*!< Color Lockup Table Enable */

/********************  Bit definition for LTDC_LxWHPCR register  **************/

#define LTDC_LxWHPCR_WHSTPOS                (0x00000FFFUL)              /*!< Window Horizontal Start Position */
#define LTDC_LxWHPCR_WHSPPOS                (0xFFFF0000UL)              /*!< Window Horizontal Stop Position */

/********************  Bit definition for LTDC_LxWVPCR register  **************/

#define LTDC_LxWVPCR_WVSTPOS                (0x00000FFFUL)              /*!< Window Vertical Start Position */
#define LTDC_LxWVPCR_WVSPPOS                (0xFFFF0000UL)              /*!< Window Vertical Stop Position */

/********************  Bit definition for LTDC_LxCKCR register  ***************/

#define LTDC_LxCKCR_CKBLUE                  (0x000000FFUL)              /*!< Color Key Blue value */
#define LTDC_LxCKCR_CKGREEN                 (0x0000FF00UL)              /*!< Color Key Green value */
#define LTDC_LxCKCR_CKRED                   (0x00FF0000UL)              /*!< Color Key Red value */

/********************  Bit definition for LTDC_LxPFCR register  ***************/

#define LTDC_LxPFCR_PF                      (0x00000007UL)              /*!< Pixel Format */

/********************  Bit definition for LTDC_LxCACR register  ***************/

#define LTDC_LxCACR_CONSTA                  (0x000000FFUL)              /*!< Constant Alpha */

/********************  Bit definition for LTDC_LxDCCR register  ***************/

#define LTDC_LxDCCR_DCBLUE                  (0x000000FFUL)              /*!< Default Color Blue */
#define LTDC_LxDCCR_DCGREEN                 (0x0000FF00UL)              /*!< Default Color Green */
#define LTDC_LxDCCR_DCRED                   (0x00FF0000UL)              /*!< Default Color Red */
#define LTDC_LxDCCR_DCALPHA                 (0xFF000000UL)              /*!< Default Color Alpha */

/********************  Bit definition for LTDC_LxBFCR register  ***************/

#define LTDC_LxBFCR_BF2                     (0x00000007UL)              /*!< Blending Factor 2 */
#define LTDC_LxBFCR_BF1                     (0x00000700UL)              /*!< Blending Factor 1 */

/********************  Bit definition for LTDC_LxCFBAR register  **************/

#define LTDC_LxCFBAR_CFBADD                 (0xFFFFFFFFUL)              /*!< Color Frame Buffer Start Address */

/********************  Bit definition for LTDC_LxCFBLR register  **************/

#define LTDC_LxCFBLR_CFBLL                  (0x00001FFFUL)              /*!< Color Frame Buffer Line Length */
#define LTDC_LxCFBLR_CFBP                   (0x1FFF0000UL)              /*!< Color Frame Buffer Pitch in bytes */

/********************  Bit definition for LTDC_LxCFBLNR register  *************/

#define LTDC_LxCFBLNR_CFBLNBR               (0x000007FFUL)              /*!< Frame Buffer Line Number */

/********************  Bit definition for LTDC_LxCLUTWR register  *************/

#define LTDC_LxCLUTWR_BLUE                  (0x000000FFUL)              /*!< Blue value */
#define LTDC_LxCLUTWR_GREEN                 (0x0000FF00UL)              /*!< Green value */
#define LTDC_LxCLUTWR_RED                   (0x00FF0000UL)              /*!< Red value */
#define LTDC_LxCLUTWR_CLUTADD               (0xFF000000UL)              /*!< CLUT address */

#if defined(STM32F469_479xx)
/******************************************************************************/
/*                                                                            */
/*                                    DSI                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for DSI_VR register  *****************/
#define DSI_VR                         (0x3133302AUL)               /*!< DSI Host Version */

/*******************  Bit definition for DSI_CR register  *****************/
#define DSI_CR_EN                      (0x00000001UL)               /*!< DSI Host power up and reset */

/*******************  Bit definition for DSI_CCR register  ****************/
#define DSI_CCR_TXECKDIV               (0x000000FFUL)               /*!< TX Escape Clock Division */
#define DSI_CCR_TXECKDIV0              (0x00000001UL)
#define DSI_CCR_TXECKDIV1              (0x00000002UL)
#define DSI_CCR_TXECKDIV2              (0x00000004UL)
#define DSI_CCR_TXECKDIV3              (0x00000008UL)
#define DSI_CCR_TXECKDIV4              (0x00000010UL)
#define DSI_CCR_TXECKDIV5              (0x00000020UL)
#define DSI_CCR_TXECKDIV6              (0x00000040UL)
#define DSI_CCR_TXECKDIV7              (0x00000080UL)

#define DSI_CCR_TOCKDIV                (0x0000FF00UL)               /*!< Timeout Clock Division */
#define DSI_CCR_TOCKDIV0               (0x00000100UL)
#define DSI_CCR_TOCKDIV1               (0x00000200UL)
#define DSI_CCR_TOCKDIV2               (0x00000400UL)
#define DSI_CCR_TOCKDIV3               (0x00000800UL)
#define DSI_CCR_TOCKDIV4               (0x00001000UL)
#define DSI_CCR_TOCKDIV5               (0x00002000UL)
#define DSI_CCR_TOCKDIV6               (0x00004000UL)
#define DSI_CCR_TOCKDIV7               (0x00008000UL)

/*******************  Bit definition for DSI_LVCIDR register  *************/
#define DSI_LVCIDR_VCID                (0x00000003UL)               /*!< Virtual Channel ID */
#define DSI_LVCIDR_VCID0               (0x00000001UL)
#define DSI_LVCIDR_VCID1               (0x00000002UL)

/*******************  Bit definition for DSI_LCOLCR register  *************/
#define DSI_LCOLCR_COLC                (0x0000000FUL)               /*!< Color Coding */
#define DSI_LCOLCR_COLC0               (0x00000001UL)
#define DSI_LCOLCR_COLC1               (0x00000020UL)
#define DSI_LCOLCR_COLC2               (0x00000040UL)
#define DSI_LCOLCR_COLC3               (0x00000080UL)

#define DSI_LCOLCR_LPE                 (0x00000100UL)               /*!< Loosly Packet Enable */

/*******************  Bit definition for DSI_LPCR register  ***************/
#define DSI_LPCR_DEP                   (0x00000001UL)               /*!< Data Enable Polarity */
#define DSI_LPCR_VSP                   (0x00000002UL)               /*!< VSYNC Polarity */
#define DSI_LPCR_HSP                   (0x00000004UL)               /*!< HSYNC Polarity */

/*******************  Bit definition for DSI_LPMCR register  **************/
#define DSI_LPMCR_VLPSIZE              (0x000000FFUL)               /*!< VACT Largest Packet Size */
#define DSI_LPMCR_VLPSIZE0             (0x00000001UL)
#define DSI_LPMCR_VLPSIZE1             (0x00000002UL)
#define DSI_LPMCR_VLPSIZE2             (0x00000004UL)
#define DSI_LPMCR_VLPSIZE3             (0x00000008UL)
#define DSI_LPMCR_VLPSIZE4             (0x00000010UL)
#define DSI_LPMCR_VLPSIZE5             (0x00000020UL)
#define DSI_LPMCR_VLPSIZE6             (0x00000040UL)
#define DSI_LPMCR_VLPSIZE7             (0x00000080UL)

#define DSI_LPMCR_LPSIZE               (0x00FF0000UL)               /*!< Largest Packet Size */
#define DSI_LPMCR_LPSIZE0              (0x00010000UL)
#define DSI_LPMCR_LPSIZE1              (0x00020000UL)
#define DSI_LPMCR_LPSIZE2              (0x00040000UL)
#define DSI_LPMCR_LPSIZE3              (0x00080000UL)
#define DSI_LPMCR_LPSIZE4              (0x00100000UL)
#define DSI_LPMCR_LPSIZE5              (0x00200000UL)
#define DSI_LPMCR_LPSIZE6              (0x00400000UL)
#define DSI_LPMCR_LPSIZE7              (0x00800000UL)

/*******************  Bit definition for DSI_PCR register  ****************/
#define DSI_PCR_ETTXE                  (0x00000001UL)               /*!< EoTp Transmission Enable */
#define DSI_PCR_ETRXE                  (0x00000002UL)               /*!< EoTp Reception Enable */
#define DSI_PCR_BTAE                   (0x00000004UL)               /*!< Bus Turn Around Enable */
#define DSI_PCR_ECCRXE                 (0x00000008UL)               /*!< ECC Reception Enable */
#define DSI_PCR_CRCRXE                 (0x00000010UL)               /*!< CRC Reception Enable */

/*******************  Bit definition for DSI_GVCIDR register  *************/
#define DSI_GVCIDR_VCID                (0x00000003UL)               /*!< Virtual Channel ID */
#define DSI_GVCIDR_VCID0               (0x00000001UL)
#define DSI_GVCIDR_VCID1               (0x00000002UL)

/*******************  Bit definition for DSI_MCR register  ****************/
#define DSI_MCR_CMDM                   (0x00000001UL)               /*!< Command Mode */

/*******************  Bit definition for DSI_VMCR register  ***************/
#define DSI_VMCR_VMT                   (0x00000003UL)               /*!< Video Mode Type */
#define DSI_VMCR_VMT0                  (0x00000001UL)
#define DSI_VMCR_VMT1                  (0x00000002UL)

#define DSI_VMCR_LPVSAE                (0x00000100UL)               /*!< Low-Power Vertical Sync Active Enable */
#define DSI_VMCR_LPVBPE                (0x00000200UL)               /*!< Low-power Vertical Back-Porch Enable */
#define DSI_VMCR_LPVFPE                (0x00000400UL)               /*!< Low-power Vertical Front-porch Enable */
#define DSI_VMCR_LPVAE                 (0x00000800UL)               /*!< Low-Power Vertical Active Enable */
#define DSI_VMCR_LPHBPE                (0x00001000UL)               /*!< Low-Power Horizontal Back-Porch Enable */
#define DSI_VMCR_LPHFPE                (0x00002000UL)               /*!< Low-Power Horizontal Front-Porch Enable */
#define DSI_VMCR_FBTAAE                (0x00004000UL)               /*!< Frame Bus-Turn-Around Acknowledge Enable */
#define DSI_VMCR_LPCE                  (0x00008000UL)               /*!< Low-Power Command Enable */
#define DSI_VMCR_PGE                   (0x00010000UL)               /*!< Pattern Generator Enable */
#define DSI_VMCR_PGM                   (0x00100000UL)               /*!< Pattern Generator Mode */
#define DSI_VMCR_PGO                   (0x01000000UL)               /*!< Pattern Generator Orientation */

/*******************  Bit definition for DSI_VPCR register  ***************/
#define DSI_VPCR_VPSIZE                (0x00003FFFUL)               /*!< Video Packet Size */
#define DSI_VPCR_VPSIZE0               (0x00000001UL)
#define DSI_VPCR_VPSIZE1               (0x00000002UL)
#define DSI_VPCR_VPSIZE2               (0x00000004UL)
#define DSI_VPCR_VPSIZE3               (0x00000008UL)
#define DSI_VPCR_VPSIZE4               (0x00000010UL)
#define DSI_VPCR_VPSIZE5               (0x00000020UL)
#define DSI_VPCR_VPSIZE6               (0x00000040UL)
#define DSI_VPCR_VPSIZE7               (0x00000080UL)
#define DSI_VPCR_VPSIZE8               (0x00000100UL)
#define DSI_VPCR_VPSIZE9               (0x00000200UL)
#define DSI_VPCR_VPSIZE10              (0x00000400UL)
#define DSI_VPCR_VPSIZE11              (0x00000800UL)
#define DSI_VPCR_VPSIZE12              (0x00001000UL)
#define DSI_VPCR_VPSIZE13              (0x00002000UL)

/*******************  Bit definition for DSI_VCCR register  ***************/
#define DSI_VCCR_NUMC                  (0x00001FFFUL)               /*!< Number of Chunks */
#define DSI_VCCR_NUMC0                 (0x00000001UL)
#define DSI_VCCR_NUMC1                 (0x00000002UL)
#define DSI_VCCR_NUMC2                 (0x00000004UL)
#define DSI_VCCR_NUMC3                 (0x00000008UL)
#define DSI_VCCR_NUMC4                 (0x00000010UL)
#define DSI_VCCR_NUMC5                 (0x00000020UL)
#define DSI_VCCR_NUMC6                 (0x00000040UL)
#define DSI_VCCR_NUMC7                 (0x00000080UL)
#define DSI_VCCR_NUMC8                 (0x00000100UL)
#define DSI_VCCR_NUMC9                 (0x00000200UL)
#define DSI_VCCR_NUMC10                (0x00000400UL)
#define DSI_VCCR_NUMC11                (0x00000800UL)
#define DSI_VCCR_NUMC12                (0x00001000UL)

/*******************  Bit definition for DSI_VNPCR register  **************/
#define DSI_VNPCR_NPSIZE               (0x00001FFFUL)               /*!< Null Packet Size */
#define DSI_VNPCR_NPSIZE0              (0x00000001UL)
#define DSI_VNPCR_NPSIZE1              (0x00000002UL)
#define DSI_VNPCR_NPSIZE2              (0x00000004UL)
#define DSI_VNPCR_NPSIZE3              (0x00000008UL)
#define DSI_VNPCR_NPSIZE4              (0x00000010UL)
#define DSI_VNPCR_NPSIZE5              (0x00000020UL)
#define DSI_VNPCR_NPSIZE6              (0x00000040UL)
#define DSI_VNPCR_NPSIZE7              (0x00000080UL)
#define DSI_VNPCR_NPSIZE8              (0x00000100UL)
#define DSI_VNPCR_NPSIZE9              (0x00000200UL)
#define DSI_VNPCR_NPSIZE10             (0x00000400UL)
#define DSI_VNPCR_NPSIZE11             (0x00000800UL)
#define DSI_VNPCR_NPSIZE12             (0x00001000UL)

/*******************  Bit definition for DSI_VHSACR register  *************/
#define DSI_VHSACR_HSA                 (0x00000FFFUL)               /*!< Horizontal Synchronism Active duration */
#define DSI_VHSACR_HSA0                (0x00000001UL)
#define DSI_VHSACR_HSA1                (0x00000002UL)
#define DSI_VHSACR_HSA2                (0x00000004UL)
#define DSI_VHSACR_HSA3                (0x00000008UL)
#define DSI_VHSACR_HSA4                (0x00000010UL)
#define DSI_VHSACR_HSA5                (0x00000020UL)
#define DSI_VHSACR_HSA6                (0x00000040UL)
#define DSI_VHSACR_HSA7                (0x00000080UL)
#define DSI_VHSACR_HSA8                (0x00000100UL)
#define DSI_VHSACR_HSA9                (0x00000200UL)
#define DSI_VHSACR_HSA10               (0x00000400UL)
#define DSI_VHSACR_HSA11               (0x00000800UL)

/*******************  Bit definition for DSI_VHBPCR register  *************/
#define DSI_VHBPCR_HBP                 (0x00000FFFUL)               /*!< Horizontal Back-Porch duration */
#define DSI_VHBPCR_HBP0                (0x00000001UL)
#define DSI_VHBPCR_HBP1                (0x00000002UL)
#define DSI_VHBPCR_HBP2                (0x00000004UL)
#define DSI_VHBPCR_HBP3                (0x00000008UL)
#define DSI_VHBPCR_HBP4                (0x00000010UL)
#define DSI_VHBPCR_HBP5                (0x00000020UL)
#define DSI_VHBPCR_HBP6                (0x00000040UL)
#define DSI_VHBPCR_HBP7                (0x00000080UL)
#define DSI_VHBPCR_HBP8                (0x00000100UL)
#define DSI_VHBPCR_HBP9                (0x00000200UL)
#define DSI_VHBPCR_HBP10               (0x00000400UL)
#define DSI_VHBPCR_HBP11               (0x00000800UL)

/*******************  Bit definition for DSI_VLCR register  ***************/
#define DSI_VLCR_HLINE                 (0x00007FFFUL)               /*!< Horizontal Line duration */
#define DSI_VLCR_HLINE0                (0x00000001UL)
#define DSI_VLCR_HLINE1                (0x00000002UL)
#define DSI_VLCR_HLINE2                (0x00000004UL)
#define DSI_VLCR_HLINE3                (0x00000008UL)
#define DSI_VLCR_HLINE4                (0x00000010UL)
#define DSI_VLCR_HLINE5                (0x00000020UL)
#define DSI_VLCR_HLINE6                (0x00000040UL)
#define DSI_VLCR_HLINE7                (0x00000080UL)
#define DSI_VLCR_HLINE8                (0x00000100UL)
#define DSI_VLCR_HLINE9                (0x00000200UL)
#define DSI_VLCR_HLINE10               (0x00000400UL)
#define DSI_VLCR_HLINE11               (0x00000800UL)
#define DSI_VLCR_HLINE12               (0x00001000UL)
#define DSI_VLCR_HLINE13               (0x00002000UL)
#define DSI_VLCR_HLINE14               (0x00004000UL)

/*******************  Bit definition for DSI_VVSACR register  *************/
#define DSI_VVSACR_VSA                 (0x000003FFUL)               /*!< Vertical Synchronism Active duration */
#define DSI_VVSACR_VSA0                (0x00000001UL)
#define DSI_VVSACR_VSA1                (0x00000002UL)
#define DSI_VVSACR_VSA2                (0x00000004UL)
#define DSI_VVSACR_VSA3                (0x00000008UL)
#define DSI_VVSACR_VSA4                (0x00000010UL)
#define DSI_VVSACR_VSA5                (0x00000020UL)
#define DSI_VVSACR_VSA6                (0x00000040UL)
#define DSI_VVSACR_VSA7                (0x00000080UL)
#define DSI_VVSACR_VSA8                (0x00000100UL)
#define DSI_VVSACR_VSA9                (0x00000200UL)

/*******************  Bit definition for DSI_VVBPCR register  *************/
#define DSI_VVBPCR_VBP                 (0x000003FFUL)               /*!< Vertical Back-Porch duration */
#define DSI_VVBPCR_VBP0                (0x00000001UL)
#define DSI_VVBPCR_VBP1                (0x00000002UL)
#define DSI_VVBPCR_VBP2                (0x00000004UL)
#define DSI_VVBPCR_VBP3                (0x00000008UL)
#define DSI_VVBPCR_VBP4                (0x00000010UL)
#define DSI_VVBPCR_VBP5                (0x00000020UL)
#define DSI_VVBPCR_VBP6                (0x00000040UL)
#define DSI_VVBPCR_VBP7                (0x00000080UL)
#define DSI_VVBPCR_VBP8                (0x00000100UL)
#define DSI_VVBPCR_VBP9                (0x00000200UL)

/*******************  Bit definition for DSI_VVFPCR register  *************/
#define DSI_VVFPCR_VFP                 (0x000003FFUL)               /*!< Vertical Front-Porch duration */
#define DSI_VVFPCR_VFP0                (0x00000001UL)
#define DSI_VVFPCR_VFP1                (0x00000002UL)
#define DSI_VVFPCR_VFP2                (0x00000004UL)
#define DSI_VVFPCR_VFP3                (0x00000008UL)
#define DSI_VVFPCR_VFP4                (0x00000010UL)
#define DSI_VVFPCR_VFP5                (0x00000020UL)
#define DSI_VVFPCR_VFP6                (0x00000040UL)
#define DSI_VVFPCR_VFP7                (0x00000080UL)
#define DSI_VVFPCR_VFP8                (0x00000100UL)
#define DSI_VVFPCR_VFP9                (0x00000200UL)

/*******************  Bit definition for DSI_VVACR register  **************/
#define DSI_VVACR_VA                   (0x00003FFFUL)               /*!< Vertical Active duration */
#define DSI_VVACR_VA0                  (0x00000001UL)
#define DSI_VVACR_VA1                  (0x00000002UL)
#define DSI_VVACR_VA2                  (0x00000004UL)
#define DSI_VVACR_VA3                  (0x00000008UL)
#define DSI_VVACR_VA4                  (0x00000010UL)
#define DSI_VVACR_VA5                  (0x00000020UL)
#define DSI_VVACR_VA6                  (0x00000040UL)
#define DSI_VVACR_VA7                  (0x00000080UL)
#define DSI_VVACR_VA8                  (0x00000100UL)
#define DSI_VVACR_VA9                  (0x00000200UL)
#define DSI_VVACR_VA10                 (0x00000400UL)
#define DSI_VVACR_VA11                 (0x00000800UL)
#define DSI_VVACR_VA12                 (0x00001000UL)
#define DSI_VVACR_VA13                 (0x00002000UL)

/*******************  Bit definition for DSI_LCCR register  ***************/
#define DSI_LCCR_CMDSIZE               (0x0000FFFFUL)               /*!< Command Size */
#define DSI_LCCR_CMDSIZE0              (0x00000001UL)
#define DSI_LCCR_CMDSIZE1              (0x00000002UL)
#define DSI_LCCR_CMDSIZE2              (0x00000004UL)
#define DSI_LCCR_CMDSIZE3              (0x00000008UL)
#define DSI_LCCR_CMDSIZE4              (0x00000010UL)
#define DSI_LCCR_CMDSIZE5              (0x00000020UL)
#define DSI_LCCR_CMDSIZE6              (0x00000040UL)
#define DSI_LCCR_CMDSIZE7              (0x00000080UL)
#define DSI_LCCR_CMDSIZE8              (0x00000100UL)
#define DSI_LCCR_CMDSIZE9              (0x00000200UL)
#define DSI_LCCR_CMDSIZE10             (0x00000400UL)
#define DSI_LCCR_CMDSIZE11             (0x00000800UL)
#define DSI_LCCR_CMDSIZE12             (0x00001000UL)
#define DSI_LCCR_CMDSIZE13             (0x00002000UL)
#define DSI_LCCR_CMDSIZE14             (0x00004000UL)
#define DSI_LCCR_CMDSIZE15             (0x00008000UL)

/*******************  Bit definition for DSI_CMCR register  ***************/
#define DSI_CMCR_TEARE                 (0x00000001UL)               /*!< Tearing Effect Acknowledge Request Enable */
#define DSI_CMCR_ARE                   (0x00000002UL)               /*!< Acknowledge Request Enable */
#define DSI_CMCR_GSW0TX                (0x00000100UL)               /*!< Generic Short Write Zero parameters Transmission */
#define DSI_CMCR_GSW1TX                (0x00000200UL)               /*!< Generic Short Write One parameters Transmission */
#define DSI_CMCR_GSW2TX                (0x00000400UL)               /*!< Generic Short Write Two parameters Transmission */
#define DSI_CMCR_GSR0TX                (0x00000800UL)               /*!< Generic Short Read Zero parameters Transmission */
#define DSI_CMCR_GSR1TX                (0x00001000UL)               /*!< Generic Short Read One parameters Transmission */
#define DSI_CMCR_GSR2TX                (0x00002000UL)               /*!< Generic Short Read Two parameters Transmission */
#define DSI_CMCR_GLWTX                 (0x00004000UL)               /*!< Generic Long Write Transmission */
#define DSI_CMCR_DSW0TX                (0x00010000UL)               /*!< DCS Short Write Zero parameter Transmission */
#define DSI_CMCR_DSW1TX                (0x00020000UL)               /*!< DCS Short Read One parameter Transmission */
#define DSI_CMCR_DSR0TX                (0x00040000UL)               /*!< DCS Short Read Zero parameter Transmission */
#define DSI_CMCR_DLWTX                 (0x00080000UL)               /*!< DCS Long Write Transmission */
#define DSI_CMCR_MRDPS                 (0x01000000UL)               /*!< Maximum Read Packet Size */

/*******************  Bit definition for DSI_GHCR register  ***************/
#define DSI_GHCR_DT                    (0x0000003FUL)               /*!< Type */
#define DSI_GHCR_DT0                   (0x00000001UL)
#define DSI_GHCR_DT1                   (0x00000002UL)
#define DSI_GHCR_DT2                   (0x00000004UL)
#define DSI_GHCR_DT3                   (0x00000008UL)
#define DSI_GHCR_DT4                   (0x00000010UL)
#define DSI_GHCR_DT5                   (0x00000020UL)

#define DSI_GHCR_VCID                  (0x000000C0UL)               /*!< Channel */
#define DSI_GHCR_VCID0                 (0x00000040UL)
#define DSI_GHCR_VCID1                 (0x00000080UL)

#define DSI_GHCR_WCLSB                 (0x0000FF00UL)               /*!< WordCount LSB */
#define DSI_GHCR_WCLSB0                (0x00000100UL)
#define DSI_GHCR_WCLSB1                (0x00000200UL)
#define DSI_GHCR_WCLSB2                (0x00000400UL)
#define DSI_GHCR_WCLSB3                (0x00000800UL)
#define DSI_GHCR_WCLSB4                (0x00001000UL)
#define DSI_GHCR_WCLSB5                (0x00002000UL)
#define DSI_GHCR_WCLSB6                (0x00004000UL)
#define DSI_GHCR_WCLSB7                (0x00008000UL)

#define DSI_GHCR_WCMSB                 (0x00FF0000UL)               /*!< WordCount MSB */
#define DSI_GHCR_WCMSB0                (0x00010000UL)
#define DSI_GHCR_WCMSB1                (0x00020000UL)
#define DSI_GHCR_WCMSB2                (0x00040000UL)
#define DSI_GHCR_WCMSB3                (0x00080000UL)
#define DSI_GHCR_WCMSB4                (0x00100000UL)
#define DSI_GHCR_WCMSB5                (0x00200000UL)
#define DSI_GHCR_WCMSB6                (0x00400000UL)
#define DSI_GHCR_WCMSB7                (0x00800000UL)

/*******************  Bit definition for DSI_GPDR register  ***************/
#define DSI_GPDR_DATA1                 (0x000000FFUL)               /*!< Payload Byte 1 */
#define DSI_GPDR_DATA1_0               (0x00000001UL)
#define DSI_GPDR_DATA1_1               (0x00000002UL)
#define DSI_GPDR_DATA1_2               (0x00000004UL)
#define DSI_GPDR_DATA1_3               (0x00000008UL)
#define DSI_GPDR_DATA1_4               (0x00000010UL)
#define DSI_GPDR_DATA1_5               (0x00000020UL)
#define DSI_GPDR_DATA1_6               (0x00000040UL)
#define DSI_GPDR_DATA1_7               (0x00000080UL)

#define DSI_GPDR_DATA2                 (0x0000FF00UL)               /*!< Payload Byte 2 */
#define DSI_GPDR_DATA2_0               (0x00000100UL)
#define DSI_GPDR_DATA2_1               (0x00000200UL)
#define DSI_GPDR_DATA2_2               (0x00000400UL)
#define DSI_GPDR_DATA2_3               (0x00000800UL)
#define DSI_GPDR_DATA2_4               (0x00001000UL)
#define DSI_GPDR_DATA2_5               (0x00002000UL)
#define DSI_GPDR_DATA2_6               (0x00004000UL)
#define DSI_GPDR_DATA2_7               (0x00008000UL)

#define DSI_GPDR_DATA3                 (0x00FF0000UL)               /*!< Payload Byte 3 */
#define DSI_GPDR_DATA3_0               (0x00010000UL)
#define DSI_GPDR_DATA3_1               (0x00020000UL)
#define DSI_GPDR_DATA3_2               (0x00040000UL)
#define DSI_GPDR_DATA3_3               (0x00080000UL)
#define DSI_GPDR_DATA3_4               (0x00100000UL)
#define DSI_GPDR_DATA3_5               (0x00200000UL)
#define DSI_GPDR_DATA3_6               (0x00400000UL)
#define DSI_GPDR_DATA3_7               (0x00800000UL)

#define DSI_GPDR_DATA4                 (0xFF000000UL)               /*!< Payload Byte 4 */
#define DSI_GPDR_DATA4_0               (0x01000000UL)
#define DSI_GPDR_DATA4_1               (0x02000000UL)
#define DSI_GPDR_DATA4_2               (0x04000000UL)
#define DSI_GPDR_DATA4_3               (0x08000000UL)
#define DSI_GPDR_DATA4_4               (0x10000000UL)
#define DSI_GPDR_DATA4_5               (0x20000000UL)
#define DSI_GPDR_DATA4_6               (0x40000000UL)
#define DSI_GPDR_DATA4_7               (0x80000000UL)

/*******************  Bit definition for DSI_GPSR register  ***************/
#define DSI_GPSR_CMDFE                 (0x00000001UL)               /*!< Command FIFO Empty */
#define DSI_GPSR_CMDFF                 (0x00000002UL)               /*!< Command FIFO Full */
#define DSI_GPSR_PWRFE                 (0x00000004UL)               /*!< Payload Write FIFO Empty */
#define DSI_GPSR_PWRFF                 (0x00000008UL)               /*!< Payload Write FIFO Full */
#define DSI_GPSR_PRDFE                 (0x00000010UL)               /*!< Payload Read FIFO Empty */
#define DSI_GPSR_PRDFF                 (0x00000020UL)               /*!< Payload Read FIFO Full */
#define DSI_GPSR_RCB                   (0x00000040UL)               /*!< Read Command Busy */

/*******************  Bit definition for DSI_TCCR0 register  **************/
#define DSI_TCCR0_LPRX_TOCNT           (0x0000FFFFUL)               /*!< Low-power Reception Timeout Counter */
#define DSI_TCCR0_LPRX_TOCNT0          (0x00000001UL)
#define DSI_TCCR0_LPRX_TOCNT1          (0x00000002UL)
#define DSI_TCCR0_LPRX_TOCNT2          (0x00000004UL)
#define DSI_TCCR0_LPRX_TOCNT3          (0x00000008UL)
#define DSI_TCCR0_LPRX_TOCNT4          (0x00000010UL)
#define DSI_TCCR0_LPRX_TOCNT5          (0x00000020UL)
#define DSI_TCCR0_LPRX_TOCNT6          (0x00000040UL)
#define DSI_TCCR0_LPRX_TOCNT7          (0x00000080UL)
#define DSI_TCCR0_LPRX_TOCNT8          (0x00000100UL)
#define DSI_TCCR0_LPRX_TOCNT9          (0x00000200UL)
#define DSI_TCCR0_LPRX_TOCNT10         (0x00000400UL)
#define DSI_TCCR0_LPRX_TOCNT11         (0x00000800UL)
#define DSI_TCCR0_LPRX_TOCNT12         (0x00001000UL)
#define DSI_TCCR0_LPRX_TOCNT13         (0x00002000UL)
#define DSI_TCCR0_LPRX_TOCNT14         (0x00004000UL)
#define DSI_TCCR0_LPRX_TOCNT15         (0x00008000UL)

#define DSI_TCCR0_HSTX_TOCNT           (0xFFFF0000UL)               /*!< High-Speed Transmission Timeout Counter */
#define DSI_TCCR0_HSTX_TOCNT0          (0x00010000UL)
#define DSI_TCCR0_HSTX_TOCNT1          (0x00020000UL)
#define DSI_TCCR0_HSTX_TOCNT2          (0x00040000UL)
#define DSI_TCCR0_HSTX_TOCNT3          (0x00080000UL)
#define DSI_TCCR0_HSTX_TOCNT4          (0x00100000UL)
#define DSI_TCCR0_HSTX_TOCNT5          (0x00200000UL)
#define DSI_TCCR0_HSTX_TOCNT6          (0x00400000UL)
#define DSI_TCCR0_HSTX_TOCNT7          (0x00800000UL)
#define DSI_TCCR0_HSTX_TOCNT8          (0x01000000UL)
#define DSI_TCCR0_HSTX_TOCNT9          (0x02000000UL)
#define DSI_TCCR0_HSTX_TOCNT10         (0x04000000UL)
#define DSI_TCCR0_HSTX_TOCNT11         (0x08000000UL)
#define DSI_TCCR0_HSTX_TOCNT12         (0x10000000UL)
#define DSI_TCCR0_HSTX_TOCNT13         (0x20000000UL)
#define DSI_TCCR0_HSTX_TOCNT14         (0x40000000UL)
#define DSI_TCCR0_HSTX_TOCNT15         (0x80000000UL)

/*******************  Bit definition for DSI_TCCR1 register  **************/
#define DSI_TCCR1_HSRD_TOCNT           (0x0000FFFFUL)               /*!< High-Speed Read Timeout Counter */
#define DSI_TCCR1_HSRD_TOCNT0          (0x00000001UL)
#define DSI_TCCR1_HSRD_TOCNT1          (0x00000002UL)
#define DSI_TCCR1_HSRD_TOCNT2          (0x00000004UL)
#define DSI_TCCR1_HSRD_TOCNT3          (0x00000008UL)
#define DSI_TCCR1_HSRD_TOCNT4          (0x00000010UL)
#define DSI_TCCR1_HSRD_TOCNT5          (0x00000020UL)
#define DSI_TCCR1_HSRD_TOCNT6          (0x00000040UL)
#define DSI_TCCR1_HSRD_TOCNT7          (0x00000080UL)
#define DSI_TCCR1_HSRD_TOCNT8          (0x00000100UL)
#define DSI_TCCR1_HSRD_TOCNT9          (0x00000200UL)
#define DSI_TCCR1_HSRD_TOCNT10         (0x00000400UL)
#define DSI_TCCR1_HSRD_TOCNT11         (0x00000800UL)
#define DSI_TCCR1_HSRD_TOCNT12         (0x00001000UL)
#define DSI_TCCR1_HSRD_TOCNT13         (0x00002000UL)
#define DSI_TCCR1_HSRD_TOCNT14         (0x00004000UL)
#define DSI_TCCR1_HSRD_TOCNT15         (0x00008000UL)

/*******************  Bit definition for DSI_TCCR2 register  **************/
#define DSI_TCCR2_LPRD_TOCNT           (0x0000FFFFUL)               /*!< Low-Power Read Timeout Counter */
#define DSI_TCCR2_LPRD_TOCNT0          (0x00000001UL)
#define DSI_TCCR2_LPRD_TOCNT1          (0x00000002UL)
#define DSI_TCCR2_LPRD_TOCNT2          (0x00000004UL)
#define DSI_TCCR2_LPRD_TOCNT3          (0x00000008UL)
#define DSI_TCCR2_LPRD_TOCNT4          (0x00000010UL)
#define DSI_TCCR2_LPRD_TOCNT5          (0x00000020UL)
#define DSI_TCCR2_LPRD_TOCNT6          (0x00000040UL)
#define DSI_TCCR2_LPRD_TOCNT7          (0x00000080UL)
#define DSI_TCCR2_LPRD_TOCNT8          (0x00000100UL)
#define DSI_TCCR2_LPRD_TOCNT9          (0x00000200UL)
#define DSI_TCCR2_LPRD_TOCNT10         (0x00000400UL)
#define DSI_TCCR2_LPRD_TOCNT11         (0x00000800UL)
#define DSI_TCCR2_LPRD_TOCNT12         (0x00001000UL)
#define DSI_TCCR2_LPRD_TOCNT13         (0x00002000UL)
#define DSI_TCCR2_LPRD_TOCNT14         (0x00004000UL)
#define DSI_TCCR2_LPRD_TOCNT15         (0x00008000UL)

/*******************  Bit definition for DSI_TCCR3 register  **************/
#define DSI_TCCR3_HSWR_TOCNT           (0x0000FFFFUL)               /*!< High-Speed Write Timeout Counter */
#define DSI_TCCR3_HSWR_TOCNT0          (0x00000001UL)
#define DSI_TCCR3_HSWR_TOCNT1          (0x00000002UL)
#define DSI_TCCR3_HSWR_TOCNT2          (0x00000004UL)
#define DSI_TCCR3_HSWR_TOCNT3          (0x00000008UL)
#define DSI_TCCR3_HSWR_TOCNT4          (0x00000010UL)
#define DSI_TCCR3_HSWR_TOCNT5          (0x00000020UL)
#define DSI_TCCR3_HSWR_TOCNT6          (0x00000040UL)
#define DSI_TCCR3_HSWR_TOCNT7          (0x00000080UL)
#define DSI_TCCR3_HSWR_TOCNT8          (0x00000100UL)
#define DSI_TCCR3_HSWR_TOCNT9          (0x00000200UL)
#define DSI_TCCR3_HSWR_TOCNT10         (0x00000400UL)
#define DSI_TCCR3_HSWR_TOCNT11         (0x00000800UL)
#define DSI_TCCR3_HSWR_TOCNT12         (0x00001000UL)
#define DSI_TCCR3_HSWR_TOCNT13         (0x00002000UL)
#define DSI_TCCR3_HSWR_TOCNT14         (0x00004000UL)
#define DSI_TCCR3_HSWR_TOCNT15         (0x00008000UL)

#define DSI_TCCR3_PM                   (0x01000000UL)               /*!< Presp Mode */

/*******************  Bit definition for DSI_TCCR4 register  **************/
#define DSI_TCCR4_LPWR_TOCNT           (0x0000FFFFUL)               /*!< Low-Power Write Timeout Counter */
#define DSI_TCCR4_LPWR_TOCNT0          (0x00000001UL)
#define DSI_TCCR4_LPWR_TOCNT1          (0x00000002UL)
#define DSI_TCCR4_LPWR_TOCNT2          (0x00000004UL)
#define DSI_TCCR4_LPWR_TOCNT3          (0x00000008UL)
#define DSI_TCCR4_LPWR_TOCNT4          (0x00000010UL)
#define DSI_TCCR4_LPWR_TOCNT5          (0x00000020UL)
#define DSI_TCCR4_LPWR_TOCNT6          (0x00000040UL)
#define DSI_TCCR4_LPWR_TOCNT7          (0x00000080UL)
#define DSI_TCCR4_LPWR_TOCNT8          (0x00000100UL)
#define DSI_TCCR4_LPWR_TOCNT9          (0x00000200UL)
#define DSI_TCCR4_LPWR_TOCNT10         (0x00000400UL)
#define DSI_TCCR4_LPWR_TOCNT11         (0x00000800UL)
#define DSI_TCCR4_LPWR_TOCNT12         (0x00001000UL)
#define DSI_TCCR4_LPWR_TOCNT13         (0x00002000UL)
#define DSI_TCCR4_LPWR_TOCNT14         (0x00004000UL)
#define DSI_TCCR4_LPWR_TOCNT15         (0x00008000UL)

/*******************  Bit definition for DSI_TCCR5 register  **************/
#define DSI_TCCR5_BTA_TOCNT            (0x0000FFFFUL)               /*!< Bus-Turn-Around Timeout Counter */
#define DSI_TCCR5_BTA_TOCNT0           (0x00000001UL)
#define DSI_TCCR5_BTA_TOCNT1           (0x00000002UL)
#define DSI_TCCR5_BTA_TOCNT2           (0x00000004UL)
#define DSI_TCCR5_BTA_TOCNT3           (0x00000008UL)
#define DSI_TCCR5_BTA_TOCNT4           (0x00000010UL)
#define DSI_TCCR5_BTA_TOCNT5           (0x00000020UL)
#define DSI_TCCR5_BTA_TOCNT6           (0x00000040UL)
#define DSI_TCCR5_BTA_TOCNT7           (0x00000080UL)
#define DSI_TCCR5_BTA_TOCNT8           (0x00000100UL)
#define DSI_TCCR5_BTA_TOCNT9           (0x00000200UL)
#define DSI_TCCR5_BTA_TOCNT10          (0x00000400UL)
#define DSI_TCCR5_BTA_TOCNT11          (0x00000800UL)
#define DSI_TCCR5_BTA_TOCNT12          (0x00001000UL)
#define DSI_TCCR5_BTA_TOCNT13          (0x00002000UL)
#define DSI_TCCR5_BTA_TOCNT14          (0x00004000UL)
#define DSI_TCCR5_BTA_TOCNT15          (0x00008000UL)

/*******************  Bit definition for DSI_TDCR register  ***************/
#define DSI_TDCR_3DM                   (0x00000003UL)               /*!< 3D Mode */
#define DSI_TDCR_3DM0                  (0x00000001UL)
#define DSI_TDCR_3DM1                  (0x00000002UL)

#define DSI_TDCR_3DF                   (0x0000000CUL)               /*!< 3D Format */
#define DSI_TDCR_3DF0                  (0x00000004UL)
#define DSI_TDCR_3DF1                  (0x00000008UL)

#define DSI_TDCR_SVS                   (0x00000010UL)               /*!< Second VSYNC */
#define DSI_TDCR_RF                    (0x00000020UL)               /*!< Right First */
#define DSI_TDCR_S3DC                  (0x00010000UL)               /*!< Send 3D Control */

/*******************  Bit definition for DSI_CLCR register  ***************/
#define DSI_CLCR_DPCC                  (0x00000001UL)               /*!< D-PHY Clock Control */
#define DSI_CLCR_ACR                   (0x00000002UL)               /*!< Automatic Clocklane Control */

/*******************  Bit definition for DSI_CLTCR register  **************/
#define DSI_CLTCR_LP2HS_TIME           (0x000003FFUL)               /*!< Low-Power to High-Speed Time */
#define DSI_CLTCR_LP2HS_TIME0          (0x00000001UL)
#define DSI_CLTCR_LP2HS_TIME1          (0x00000002UL)
#define DSI_CLTCR_LP2HS_TIME2          (0x00000004UL)
#define DSI_CLTCR_LP2HS_TIME3          (0x00000008UL)
#define DSI_CLTCR_LP2HS_TIME4          (0x00000010UL)
#define DSI_CLTCR_LP2HS_TIME5          (0x00000020UL)
#define DSI_CLTCR_LP2HS_TIME6          (0x00000040UL)
#define DSI_CLTCR_LP2HS_TIME7          (0x00000080UL)
#define DSI_CLTCR_LP2HS_TIME8          (0x00000100UL)
#define DSI_CLTCR_LP2HS_TIME9          (0x00000200UL)

#define DSI_CLTCR_HS2LP_TIME           (0x03FF0000UL)               /*!< High-Speed to Low-Power Time */
#define DSI_CLTCR_HS2LP_TIME0          (0x00010000UL)
#define DSI_CLTCR_HS2LP_TIME1          (0x00020000UL)
#define DSI_CLTCR_HS2LP_TIME2          (0x00040000UL)
#define DSI_CLTCR_HS2LP_TIME3          (0x00080000UL)
#define DSI_CLTCR_HS2LP_TIME4          (0x00100000UL)
#define DSI_CLTCR_HS2LP_TIME5          (0x00200000UL)
#define DSI_CLTCR_HS2LP_TIME6          (0x00400000UL)
#define DSI_CLTCR_HS2LP_TIME7          (0x00800000UL)
#define DSI_CLTCR_HS2LP_TIME8          (0x01000000UL)
#define DSI_CLTCR_HS2LP_TIME9          (0x02000000UL)

/*******************  Bit definition for DSI_DLTCR register  **************/
#define DSI_DLTCR_MRD_TIME             (0x00007FFFUL)               /*!< Maximum Read Time */
#define DSI_DLTCR_MRD_TIME0            (0x00000001UL)
#define DSI_DLTCR_MRD_TIME1            (0x00000002UL)
#define DSI_DLTCR_MRD_TIME2            (0x00000004UL)
#define DSI_DLTCR_MRD_TIME3            (0x00000008UL)
#define DSI_DLTCR_MRD_TIME4            (0x00000010UL)
#define DSI_DLTCR_MRD_TIME5            (0x00000020UL)
#define DSI_DLTCR_MRD_TIME6            (0x00000040UL)
#define DSI_DLTCR_MRD_TIME7            (0x00000080UL)
#define DSI_DLTCR_MRD_TIME8            (0x00000100UL)
#define DSI_DLTCR_MRD_TIME9            (0x00000200UL)
#define DSI_DLTCR_MRD_TIME10           (0x00000400UL)
#define DSI_DLTCR_MRD_TIME11           (0x00000800UL)
#define DSI_DLTCR_MRD_TIME12           (0x00001000UL)
#define DSI_DLTCR_MRD_TIME13           (0x00002000UL)
#define DSI_DLTCR_MRD_TIME14           (0x00004000UL)

#define DSI_DLTCR_LP2HS_TIME           (0x00FF0000UL)               /*!< Low-Power To High-Speed Time */
#define DSI_DLTCR_LP2HS_TIME0          (0x00010000UL)
#define DSI_DLTCR_LP2HS_TIME1          (0x00020000UL)
#define DSI_DLTCR_LP2HS_TIME2          (0x00040000UL)
#define DSI_DLTCR_LP2HS_TIME3          (0x00080000UL)
#define DSI_DLTCR_LP2HS_TIME4          (0x00100000UL)
#define DSI_DLTCR_LP2HS_TIME5          (0x00200000UL)
#define DSI_DLTCR_LP2HS_TIME6          (0x00400000UL)
#define DSI_DLTCR_LP2HS_TIME7          (0x00800000UL)

#define DSI_DLTCR_HS2LP_TIME           (0xFF000000UL)               /*!< High-Speed To Low-Power Time */
#define DSI_DLTCR_HS2LP_TIME0          (0x01000000UL)
#define DSI_DLTCR_HS2LP_TIME1          (0x02000000UL)
#define DSI_DLTCR_HS2LP_TIME2          (0x04000000UL)
#define DSI_DLTCR_HS2LP_TIME3          (0x08000000UL)
#define DSI_DLTCR_HS2LP_TIME4          (0x10000000UL)
#define DSI_DLTCR_HS2LP_TIME5          (0x20000000UL)
#define DSI_DLTCR_HS2LP_TIME6          (0x40000000UL)
#define DSI_DLTCR_HS2LP_TIME7          (0x80000000UL)

/*******************  Bit definition for DSI_PCTLR register  **************/
#define DSI_PCTLR_DEN                  (0x00000002UL)               /*!< Digital Enable */
#define DSI_PCTLR_CKE                  (0x00000004UL)               /*!< Clock Enable */

/*******************  Bit definition for DSI_PCONFR register  *************/
#define DSI_PCONFR_NL                  (0x00000003UL)               /*!< Number of Lanes */
#define DSI_PCONFR_NL0                 (0x00000001UL)
#define DSI_PCONFR_NL1                 (0x00000002UL)

#define DSI_PCONFR_SW_TIME             (0x0000FF00UL)               /*!< Stop Wait Time */
#define DSI_PCONFR_SW_TIME0            (0x00000100UL)
#define DSI_PCONFR_SW_TIME1            (0x00000200UL)
#define DSI_PCONFR_SW_TIME2            (0x00000400UL)
#define DSI_PCONFR_SW_TIME3            (0x00000800UL)
#define DSI_PCONFR_SW_TIME4            (0x00001000UL)
#define DSI_PCONFR_SW_TIME5            (0x00002000UL)
#define DSI_PCONFR_SW_TIME6            (0x00004000UL)
#define DSI_PCONFR_SW_TIME7            (0x00008000UL)

/*******************  Bit definition for DSI_PUCR register  ***************/
#define DSI_PUCR_URCL                  (0x00000001UL)               /*!< ULPS Request on Clock Lane */
#define DSI_PUCR_UECL                  (0x00000002UL)               /*!< ULPS Exit on Clock Lane */
#define DSI_PUCR_URDL                  (0x00000004UL)               /*!< ULPS Request on Data Lane */
#define DSI_PUCR_UEDL                  (0x00000008UL)               /*!< ULPS Exit on Data Lane */

/*******************  Bit definition for DSI_PTTCR register  **************/
#define DSI_PTTCR_TX_TRIG              (0x0000000FUL)               /*!< Transmission Trigger */
#define DSI_PTTCR_TX_TRIG0             (0x00000001UL)
#define DSI_PTTCR_TX_TRIG1             (0x00000002UL)
#define DSI_PTTCR_TX_TRIG2             (0x00000004UL)
#define DSI_PTTCR_TX_TRIG3             (0x00000008UL)

/*******************  Bit definition for DSI_PSR register  ****************/
#define DSI_PSR_PD                     (0x00000002UL)               /*!< PHY Direction */
#define DSI_PSR_PSSC                   (0x00000004UL)               /*!< PHY Stop State Clock lane */
#define DSI_PSR_UANC                   (0x00000008UL)               /*!< ULPS Active Not Clock lane */
#define DSI_PSR_PSS0                   (0x00000010UL)               /*!< PHY Stop State lane 0 */
#define DSI_PSR_UAN0                   (0x00000020UL)               /*!< ULPS Active Not lane 0 */
#define DSI_PSR_RUE0                   (0x00000040UL)               /*!< RX ULPS Escape lane 0 */
#define DSI_PSR_PSS1                   (0x00000080UL)               /*!< PHY Stop State lane 1 */
#define DSI_PSR_UAN1                   (0x00000100UL)               /*!< ULPS Active Not lane 1 */

/*******************  Bit definition for DSI_ISR0 register  ***************/
#define DSI_ISR0_AE0                   (0x00000001UL)               /*!< Acknowledge Error 0 */
#define DSI_ISR0_AE1                   (0x00000002UL)               /*!< Acknowledge Error 1 */
#define DSI_ISR0_AE2                   (0x00000004UL)               /*!< Acknowledge Error 2 */
#define DSI_ISR0_AE3                   (0x00000008UL)               /*!< Acknowledge Error 3 */
#define DSI_ISR0_AE4                   (0x00000010UL)               /*!< Acknowledge Error 4 */
#define DSI_ISR0_AE5                   (0x00000020UL)               /*!< Acknowledge Error 5 */
#define DSI_ISR0_AE6                   (0x00000040UL)               /*!< Acknowledge Error 6 */
#define DSI_ISR0_AE7                   (0x00000080UL)               /*!< Acknowledge Error 7 */
#define DSI_ISR0_AE8                   (0x00000100UL)               /*!< Acknowledge Error 8 */
#define DSI_ISR0_AE9                   (0x00000200UL)               /*!< Acknowledge Error 9 */
#define DSI_ISR0_AE10                  (0x00000400UL)               /*!< Acknowledge Error 10 */
#define DSI_ISR0_AE11                  (0x00000800UL)               /*!< Acknowledge Error 11 */
#define DSI_ISR0_AE12                  (0x00001000UL)               /*!< Acknowledge Error 12 */
#define DSI_ISR0_AE13                  (0x00002000UL)               /*!< Acknowledge Error 13 */
#define DSI_ISR0_AE14                  (0x00004000UL)               /*!< Acknowledge Error 14 */
#define DSI_ISR0_AE15                  (0x00008000UL)               /*!< Acknowledge Error 15 */
#define DSI_ISR0_PE0                   (0x00010000UL)               /*!< PHY Error 0 */
#define DSI_ISR0_PE1                   (0x00020000UL)               /*!< PHY Error 1 */
#define DSI_ISR0_PE2                   (0x00040000UL)               /*!< PHY Error 2 */
#define DSI_ISR0_PE3                   (0x00080000UL)               /*!< PHY Error 3 */
#define DSI_ISR0_PE4                   (0x00100000UL)               /*!< PHY Error 4 */

/*******************  Bit definition for DSI_ISR1 register  ***************/
#define DSI_ISR1_TOHSTX                (0x00000001UL)               /*!< Timeout High-Speed Transmission */
#define DSI_ISR1_TOLPRX                (0x00000002UL)               /*!< Timeout Low-Power Reception */
#define DSI_ISR1_ECCSE                 (0x00000004UL)               /*!< ECC Single-bit Error */
#define DSI_ISR1_ECCME                 (0x00000008UL)               /*!< ECC Multi-bit Error */
#define DSI_ISR1_CRCE                  (0x00000010UL)               /*!< CRC Error */
#define DSI_ISR1_PSE                   (0x00000020UL)               /*!< Packet Size Error */
#define DSI_ISR1_EOTPE                 (0x00000040UL)               /*!< EoTp Error */
#define DSI_ISR1_LPWRE                 (0x00000080UL)               /*!< LTDC Payload Write Error */
#define DSI_ISR1_GCWRE                 (0x00000100UL)               /*!< Generic Command Write Error */
#define DSI_ISR1_GPWRE                 (0x00000200UL)               /*!< Generic Payload Write Error */
#define DSI_ISR1_GPTXE                 (0x00000400UL)               /*!< Generic Payload Transmit Error */
#define DSI_ISR1_GPRDE                 (0x00000800UL)               /*!< Generic Payload Read Error */
#define DSI_ISR1_GPRXE                 (0x00001000UL)               /*!< Generic Payload Receive Error */

/*******************  Bit definition for DSI_IER0 register  ***************/
#define DSI_IER0_AE0IE                 (0x00000001UL)               /*!< Acknowledge Error 0 Interrupt Enable */
#define DSI_IER0_AE1IE                 (0x00000002UL)               /*!< Acknowledge Error 1 Interrupt Enable */
#define DSI_IER0_AE2IE                 (0x00000004UL)               /*!< Acknowledge Error 2 Interrupt Enable */
#define DSI_IER0_AE3IE                 (0x00000008UL)               /*!< Acknowledge Error 3 Interrupt Enable */
#define DSI_IER0_AE4IE                 (0x00000010UL)               /*!< Acknowledge Error 4 Interrupt Enable */
#define DSI_IER0_AE5IE                 (0x00000020UL)               /*!< Acknowledge Error 5 Interrupt Enable */
#define DSI_IER0_AE6IE                 (0x00000040UL)               /*!< Acknowledge Error 6 Interrupt Enable */
#define DSI_IER0_AE7IE                 (0x00000080UL)               /*!< Acknowledge Error 7 Interrupt Enable */
#define DSI_IER0_AE8IE                 (0x00000100UL)               /*!< Acknowledge Error 8 Interrupt Enable */
#define DSI_IER0_AE9IE                 (0x00000200UL)               /*!< Acknowledge Error 9 Interrupt Enable */
#define DSI_IER0_AE10IE                (0x00000400UL)               /*!< Acknowledge Error 10 Interrupt Enable */
#define DSI_IER0_AE11IE                (0x00000800UL)               /*!< Acknowledge Error 11 Interrupt Enable */
#define DSI_IER0_AE12IE                (0x00001000UL)               /*!< Acknowledge Error 12 Interrupt Enable */
#define DSI_IER0_AE13IE                (0x00002000UL)               /*!< Acknowledge Error 13 Interrupt Enable */
#define DSI_IER0_AE14IE                (0x00004000UL)               /*!< Acknowledge Error 14 Interrupt Enable */
#define DSI_IER0_AE15IE                (0x00008000UL)               /*!< Acknowledge Error 15 Interrupt Enable */
#define DSI_IER0_PE0IE                 (0x00010000UL)               /*!< PHY Error 0 Interrupt Enable */
#define DSI_IER0_PE1IE                 (0x00020000UL)               /*!< PHY Error 1 Interrupt Enable */
#define DSI_IER0_PE2IE                 (0x00040000UL)               /*!< PHY Error 2 Interrupt Enable */
#define DSI_IER0_PE3IE                 (0x00080000UL)               /*!< PHY Error 3 Interrupt Enable */
#define DSI_IER0_PE4IE                 (0x00100000UL)               /*!< PHY Error 4 Interrupt Enable */

/*******************  Bit definition for DSI_IER1 register  ***************/
#define DSI_IER1_TOHSTXIE              (0x00000001UL)               /*!< Timeout High-Speed Transmission Interrupt Enable */
#define DSI_IER1_TOLPRXIE              (0x00000002UL)               /*!< Timeout Low-Power Reception Interrupt Enable */
#define DSI_IER1_ECCSEIE               (0x00000004UL)               /*!< ECC Single-bit Error Interrupt Enable */
#define DSI_IER1_ECCMEIE               (0x00000008UL)               /*!< ECC Multi-bit Error Interrupt Enable */
#define DSI_IER1_CRCEIE                (0x00000010UL)               /*!< CRC Error Interrupt Enable */
#define DSI_IER1_PSEIE                 (0x00000020UL)               /*!< Packet Size Error Interrupt Enable */
#define DSI_IER1_EOTPEIE               (0x00000040UL)               /*!< EoTp Error Interrupt Enable */
#define DSI_IER1_LPWREIE               (0x00000080UL)               /*!< LTDC Payload Write Error Interrupt Enable */
#define DSI_IER1_GCWREIE               (0x00000100UL)               /*!< Generic Command Write Error Interrupt Enable */
#define DSI_IER1_GPWREIE               (0x00000200UL)               /*!< Generic Payload Write Error Interrupt Enable */
#define DSI_IER1_GPTXEIE               (0x00000400UL)               /*!< Generic Payload Transmit Error Interrupt Enable */
#define DSI_IER1_GPRDEIE               (0x00000800UL)               /*!< Generic Payload Read Error Interrupt Enable */
#define DSI_IER1_GPRXEIE               (0x00001000UL)               /*!< Generic Payload Receive Error Interrupt Enable */

/*******************  Bit definition for DSI_FIR0 register  ***************/
#define DSI_FIR0_FAE0                  (0x00000001UL)               /*!< Force Acknowledge Error 0 */
#define DSI_FIR0_FAE1                  (0x00000002UL)               /*!< Force Acknowledge Error 1 */
#define DSI_FIR0_FAE2                  (0x00000004UL)               /*!< Force Acknowledge Error 2 */
#define DSI_FIR0_FAE3                  (0x00000008UL)               /*!< Force Acknowledge Error 3 */
#define DSI_FIR0_FAE4                  (0x00000010UL)               /*!< Force Acknowledge Error 4 */
#define DSI_FIR0_FAE5                  (0x00000020UL)               /*!< Force Acknowledge Error 5 */
#define DSI_FIR0_FAE6                  (0x00000040UL)               /*!< Force Acknowledge Error 6 */
#define DSI_FIR0_FAE7                  (0x00000080UL)               /*!< Force Acknowledge Error 7 */
#define DSI_FIR0_FAE8                  (0x00000100UL)               /*!< Force Acknowledge Error 8 */
#define DSI_FIR0_FAE9                  (0x00000200UL)               /*!< Force Acknowledge Error 9 */
#define DSI_FIR0_FAE10                 (0x00000400UL)               /*!< Force Acknowledge Error 10 */
#define DSI_FIR0_FAE11                 (0x00000800UL)               /*!< Force Acknowledge Error 11 */
#define DSI_FIR0_FAE12                 (0x00001000UL)               /*!< Force Acknowledge Error 12 */
#define DSI_FIR0_FAE13                 (0x00002000UL)               /*!< Force Acknowledge Error 13 */
#define DSI_FIR0_FAE14                 (0x00004000UL)               /*!< Force Acknowledge Error 14 */
#define DSI_FIR0_FAE15                 (0x00008000UL)               /*!< Force Acknowledge Error 15 */
#define DSI_FIR0_FPE0                  (0x00010000UL)               /*!< Force PHY Error 0 */
#define DSI_FIR0_FPE1                  (0x00020000UL)               /*!< Force PHY Error 1 */
#define DSI_FIR0_FPE2                  (0x00040000UL)               /*!< Force PHY Error 2 */
#define DSI_FIR0_FPE3                  (0x00080000UL)               /*!< Force PHY Error 3 */
#define DSI_FIR0_FPE4                  (0x00100000UL)               /*!< Force PHY Error 4 */

/*******************  Bit definition for DSI_FIR1 register  ***************/
#define DSI_FIR1_FTOHSTX               (0x00000001UL)               /*!< Force Timeout High-Speed Transmission */
#define DSI_FIR1_FTOLPRX               (0x00000002UL)               /*!< Force Timeout Low-Power Reception */
#define DSI_FIR1_FECCSE                (0x00000004UL)               /*!< Force ECC Single-bit Error */
#define DSI_FIR1_FECCME                (0x00000008UL)               /*!< Force ECC Multi-bit Error */
#define DSI_FIR1_FCRCE                 (0x00000010UL)               /*!< Force CRC Error */
#define DSI_FIR1_FPSE                  (0x00000020UL)               /*!< Force Packet Size Error */
#define DSI_FIR1_FEOTPE                (0x00000040UL)               /*!< Force EoTp Error */
#define DSI_FIR1_FLPWRE                (0x00000080UL)               /*!< Force LTDC Payload Write Error */
#define DSI_FIR1_FGCWRE                (0x00000100UL)               /*!< Force Generic Command Write Error */
#define DSI_FIR1_FGPWRE                (0x00000200UL)               /*!< Force Generic Payload Write Error */
#define DSI_FIR1_FGPTXE                (0x00000400UL)               /*!< Force Generic Payload Transmit Error */
#define DSI_FIR1_FGPRDE                (0x00000800UL)               /*!< Force Generic Payload Read Error */
#define DSI_FIR1_FGPRXE                (0x00001000UL)               /*!< Force Generic Payload Receive Error */

/*******************  Bit definition for DSI_VSCR register  ***************/
#define DSI_VSCR_EN                    (0x00000001UL)               /*!< Enable */
#define DSI_VSCR_UR                    (0x00000100UL)               /*!< Update Register */

/*******************  Bit definition for DSI_LCVCIDR register  ************/
#define DSI_LCVCIDR_VCID               (0x00000003UL)               /*!< Virtual Channel ID */
#define DSI_LCVCIDR_VCID0              (0x00000001UL)
#define DSI_LCVCIDR_VCID1              (0x00000002UL)

/*******************  Bit definition for DSI_LCCCR register  **************/
#define DSI_LCCCR_COLC                 (0x0000000FUL)               /*!< Color Coding */
#define DSI_LCCCR_COLC0                (0x00000001UL)
#define DSI_LCCCR_COLC1                (0x00000002UL)
#define DSI_LCCCR_COLC2                (0x00000004UL)
#define DSI_LCCCR_COLC3                (0x00000008UL)

#define DSI_LCCCR_LPE                  (0x00000100UL)               /*!< Loosely Packed Enable */

/*******************  Bit definition for DSI_LPMCCR register  *************/
#define DSI_LPMCCR_VLPSIZE             (0x000000FFUL)               /*!< VACT Largest Packet Size */
#define DSI_LPMCCR_VLPSIZE0            (0x00000001UL)
#define DSI_LPMCCR_VLPSIZE1            (0x00000002UL)
#define DSI_LPMCCR_VLPSIZE2            (0x00000004UL)
#define DSI_LPMCCR_VLPSIZE3            (0x00000008UL)
#define DSI_LPMCCR_VLPSIZE4            (0x00000010UL)
#define DSI_LPMCCR_VLPSIZE5            (0x00000020UL)
#define DSI_LPMCCR_VLPSIZE6            (0x00000040UL)
#define DSI_LPMCCR_VLPSIZE7            (0x00000080UL)

#define DSI_LPMCCR_LPSIZE              (0x00FF0000UL)               /*!< Largest Packet Size */
#define DSI_LPMCCR_LPSIZE0             (0x00010000UL)
#define DSI_LPMCCR_LPSIZE1             (0x00020000UL)
#define DSI_LPMCCR_LPSIZE2             (0x00040000UL)
#define DSI_LPMCCR_LPSIZE3             (0x00080000UL)
#define DSI_LPMCCR_LPSIZE4             (0x00100000UL)
#define DSI_LPMCCR_LPSIZE5             (0x00200000UL)
#define DSI_LPMCCR_LPSIZE6             (0x00400000UL)
#define DSI_LPMCCR_LPSIZE7             (0x00800000UL)

/*******************  Bit definition for DSI_VMCCR register  **************/
#define DSI_VMCCR_VMT                  (0x00000003UL)               /*!< Video Mode Type */
#define DSI_VMCCR_VMT0                 (0x00000001UL)
#define DSI_VMCCR_VMT1                 (0x00000002UL)

#define DSI_VMCCR_LPVSAE               (0x00000100UL)               /*!< Low-power Vertical Sync time Enable */
#define DSI_VMCCR_LPVBPE               (0x00000200UL)               /*!< Low-power Vertical Back-porch Enable */
#define DSI_VMCCR_LPVFPE               (0x00000400UL)               /*!< Low-power Vertical Front-porch Enable */
#define DSI_VMCCR_LPVAE                (0x00000800UL)               /*!< Low-power Vertical Active Enable */
#define DSI_VMCCR_LPHBPE               (0x00001000UL)               /*!< Low-power Horizontal Back-porch Enable */
#define DSI_VMCCR_LPHFE                (0x00002000UL)               /*!< Low-power Horizontal Front-porch Enable */
#define DSI_VMCCR_FBTAAE               (0x00004000UL)               /*!< Frame BTA Acknowledge Enable */
#define DSI_VMCCR_LPCE                 (0x00008000UL)               /*!< Low-power Command Enable */

/*******************  Bit definition for DSI_VPCCR register  **************/
#define DSI_VPCCR_VPSIZE               (0x00003FFFUL)               /*!< Video Packet Size */
#define DSI_VPCCR_VPSIZE0              (0x00000001UL)
#define DSI_VPCCR_VPSIZE1              (0x00000002UL)
#define DSI_VPCCR_VPSIZE2              (0x00000004UL)
#define DSI_VPCCR_VPSIZE3              (0x00000008UL)
#define DSI_VPCCR_VPSIZE4              (0x00000010UL)
#define DSI_VPCCR_VPSIZE5              (0x00000020UL)
#define DSI_VPCCR_VPSIZE6              (0x00000040UL)
#define DSI_VPCCR_VPSIZE7              (0x00000080UL)
#define DSI_VPCCR_VPSIZE8              (0x00000100UL)
#define DSI_VPCCR_VPSIZE9              (0x00000200UL)
#define DSI_VPCCR_VPSIZE10             (0x00000400UL)
#define DSI_VPCCR_VPSIZE11             (0x00000800UL)
#define DSI_VPCCR_VPSIZE12             (0x00001000UL)
#define DSI_VPCCR_VPSIZE13             (0x00002000UL)

/*******************  Bit definition for DSI_VCCCR register  **************/
#define DSI_VCCCR_NUMC                 (0x00001FFFUL)               /*!< Number of Chunks */
#define DSI_VCCCR_NUMC0                (0x00000001UL)
#define DSI_VCCCR_NUMC1                (0x00000002UL)
#define DSI_VCCCR_NUMC2                (0x00000004UL)
#define DSI_VCCCR_NUMC3                (0x00000008UL)
#define DSI_VCCCR_NUMC4                (0x00000010UL)
#define DSI_VCCCR_NUMC5                (0x00000020UL)
#define DSI_VCCCR_NUMC6                (0x00000040UL)
#define DSI_VCCCR_NUMC7                (0x00000080UL)
#define DSI_VCCCR_NUMC8                (0x00000100UL)
#define DSI_VCCCR_NUMC9                (0x00000200UL)
#define DSI_VCCCR_NUMC10               (0x00000400UL)
#define DSI_VCCCR_NUMC11               (0x00000800UL)
#define DSI_VCCCR_NUMC12               (0x00001000UL)

/*******************  Bit definition for DSI_VNPCCR register  *************/
#define DSI_VNPCCR_NPSIZE              (0x00001FFFUL)               /*!< Number of Chunks */
#define DSI_VNPCCR_NPSIZE0             (0x00000001UL)
#define DSI_VNPCCR_NPSIZE1             (0x00000002UL)
#define DSI_VNPCCR_NPSIZE2             (0x00000004UL)
#define DSI_VNPCCR_NPSIZE3             (0x00000008UL)
#define DSI_VNPCCR_NPSIZE4             (0x00000010UL)
#define DSI_VNPCCR_NPSIZE5             (0x00000020UL)
#define DSI_VNPCCR_NPSIZE6             (0x00000040UL)
#define DSI_VNPCCR_NPSIZE7             (0x00000080UL)
#define DSI_VNPCCR_NPSIZE8             (0x00000100UL)
#define DSI_VNPCCR_NPSIZE9             (0x00000200UL)
#define DSI_VNPCCR_NPSIZE10            (0x00000400UL)
#define DSI_VNPCCR_NPSIZE11            (0x00000800UL)
#define DSI_VNPCCR_NPSIZE12            (0x00001000UL)

/*******************  Bit definition for DSI_VHSACCR register  ************/
#define DSI_VHSACCR_HSA                (0x00000FFFUL)               /*!< Horizontal Synchronism Active duration */
#define DSI_VHSACCR_HSA0               (0x00000001UL)
#define DSI_VHSACCR_HSA1               (0x00000002UL)
#define DSI_VHSACCR_HSA2               (0x00000004UL)
#define DSI_VHSACCR_HSA3               (0x00000008UL)
#define DSI_VHSACCR_HSA4               (0x00000010UL)
#define DSI_VHSACCR_HSA5               (0x00000020UL)
#define DSI_VHSACCR_HSA6               (0x00000040UL)
#define DSI_VHSACCR_HSA7               (0x00000080UL)
#define DSI_VHSACCR_HSA8               (0x00000100UL)
#define DSI_VHSACCR_HSA9               (0x00000200UL)
#define DSI_VHSACCR_HSA10              (0x00000400UL)
#define DSI_VHSACCR_HSA11              (0x00000800UL)

/*******************  Bit definition for DSI_VHBPCCR register  ************/
#define DSI_VHBPCCR_HBP                (0x00000FFFUL)               /*!< Horizontal Back-Porch duration */
#define DSI_VHBPCCR_HBP0               (0x00000001UL)
#define DSI_VHBPCCR_HBP1               (0x00000002UL)
#define DSI_VHBPCCR_HBP2               (0x00000004UL)
#define DSI_VHBPCCR_HBP3               (0x00000008UL)
#define DSI_VHBPCCR_HBP4               (0x00000010UL)
#define DSI_VHBPCCR_HBP5               (0x00000020UL)
#define DSI_VHBPCCR_HBP6               (0x00000040UL)
#define DSI_VHBPCCR_HBP7               (0x00000080UL)
#define DSI_VHBPCCR_HBP8               (0x00000100UL)
#define DSI_VHBPCCR_HBP9               (0x00000200UL)
#define DSI_VHBPCCR_HBP10              (0x00000400UL)
#define DSI_VHBPCCR_HBP11              (0x00000800UL)

/*******************  Bit definition for DSI_VLCCR register  **************/
#define DSI_VLCCR_HLINE                (0x00007FFFUL)               /*!< Horizontal Line duration */
#define DSI_VLCCR_HLINE0               (0x00000001UL)
#define DSI_VLCCR_HLINE1               (0x00000002UL)
#define DSI_VLCCR_HLINE2               (0x00000004UL)
#define DSI_VLCCR_HLINE3               (0x00000008UL)
#define DSI_VLCCR_HLINE4               (0x00000010UL)
#define DSI_VLCCR_HLINE5               (0x00000020UL)
#define DSI_VLCCR_HLINE6               (0x00000040UL)
#define DSI_VLCCR_HLINE7               (0x00000080UL)
#define DSI_VLCCR_HLINE8               (0x00000100UL)
#define DSI_VLCCR_HLINE9               (0x00000200UL)
#define DSI_VLCCR_HLINE10              (0x00000400UL)
#define DSI_VLCCR_HLINE11              (0x00000800UL)
#define DSI_VLCCR_HLINE12              (0x00001000UL)
#define DSI_VLCCR_HLINE13              (0x00002000UL)
#define DSI_VLCCR_HLINE14              (0x00004000UL)

/*******************  Bit definition for DSI_VVSACCR register  ***************/
#define DSI_VVSACCR_VSA                   (0x000003FFUL)               /*!< Vertical Synchronism Active duration */
#define DSI_VVSACCR_VSA0                  (0x00000001UL)
#define DSI_VVSACCR_VSA1                  (0x00000002UL)
#define DSI_VVSACCR_VSA2                  (0x00000004UL)
#define DSI_VVSACCR_VSA3                  (0x00000008UL)
#define DSI_VVSACCR_VSA4                  (0x00000010UL)
#define DSI_VVSACCR_VSA5                  (0x00000020UL)
#define DSI_VVSACCR_VSA6                  (0x00000040UL)
#define DSI_VVSACCR_VSA7                  (0x00000080UL)
#define DSI_VVSACCR_VSA8                  (0x00000100UL)
#define DSI_VVSACCR_VSA9                  (0x00000200UL)

/*******************  Bit definition for DSI_VVBPCCR register  ************/
#define DSI_VVBPCCR_VBP                (0x000003FFUL)               /*!< Vertical Back-Porch duration */
#define DSI_VVBPCCR_VBP0               (0x00000001UL)
#define DSI_VVBPCCR_VBP1               (0x00000002UL)
#define DSI_VVBPCCR_VBP2               (0x00000004UL)
#define DSI_VVBPCCR_VBP3               (0x00000008UL)
#define DSI_VVBPCCR_VBP4               (0x00000010UL)
#define DSI_VVBPCCR_VBP5               (0x00000020UL)
#define DSI_VVBPCCR_VBP6               (0x00000040UL)
#define DSI_VVBPCCR_VBP7               (0x00000080UL)
#define DSI_VVBPCCR_VBP8               (0x00000100UL)
#define DSI_VVBPCCR_VBP9               (0x00000200UL)

/*******************  Bit definition for DSI_VVFPCCR register  ************/
#define DSI_VVFPCCR_VFP                (0x000003FFUL)               /*!< Vertical Front-Porch duration */
#define DSI_VVFPCCR_VFP0               (0x00000001UL)
#define DSI_VVFPCCR_VFP1               (0x00000002UL)
#define DSI_VVFPCCR_VFP2               (0x00000004UL)
#define DSI_VVFPCCR_VFP3               (0x00000008UL)
#define DSI_VVFPCCR_VFP4               (0x00000010UL)
#define DSI_VVFPCCR_VFP5               (0x00000020UL)
#define DSI_VVFPCCR_VFP6               (0x00000040UL)
#define DSI_VVFPCCR_VFP7               (0x00000080UL)
#define DSI_VVFPCCR_VFP8               (0x00000100UL)
#define DSI_VVFPCCR_VFP9               (0x00000200UL)

/*******************  Bit definition for DSI_VVACCR register  *************/
#define DSI_VVACCR_VA                  (0x00003FFFUL)               /*!< Vertical Active duration */
#define DSI_VVACCR_VA0                 (0x00000001UL)
#define DSI_VVACCR_VA1                 (0x00000002UL)
#define DSI_VVACCR_VA2                 (0x00000004UL)
#define DSI_VVACCR_VA3                 (0x00000008UL)
#define DSI_VVACCR_VA4                 (0x00000010UL)
#define DSI_VVACCR_VA5                 (0x00000020UL)
#define DSI_VVACCR_VA6                 (0x00000040UL)
#define DSI_VVACCR_VA7                 (0x00000080UL)
#define DSI_VVACCR_VA8                 (0x00000100UL)
#define DSI_VVACCR_VA9                 (0x00000200UL)
#define DSI_VVACCR_VA10                (0x00000400UL)
#define DSI_VVACCR_VA11                (0x00000800UL)
#define DSI_VVACCR_VA12                (0x00001000UL)
#define DSI_VVACCR_VA13                (0x00002000UL)

/*******************  Bit definition for DSI_TDCCR register  **************/
#define DSI_TDCCR_3DM                  (0x00000003UL)               /*!< 3D Mode */
#define DSI_TDCCR_3DM0                 (0x00000001UL)
#define DSI_TDCCR_3DM1                 (0x00000002UL)

#define DSI_TDCCR_3DF                  (0x0000000CUL)               /*!< 3D Format */
#define DSI_TDCCR_3DF0                 (0x00000004UL)
#define DSI_TDCCR_3DF1                 (0x00000008UL)

#define DSI_TDCCR_SVS                  (0x00000010UL)               /*!< Second VSYNC */
#define DSI_TDCCR_RF                   (0x00000020UL)               /*!< Right First */
#define DSI_TDCCR_S3DC                 (0x00010000UL)               /*!< Send 3D Control */

/*******************  Bit definition for DSI_WCFGR register  ***************/
#define DSI_WCFGR_DSIM                   (0x00000001UL)              /*!< DSI Mode */
#define DSI_WCFGR_COLMUX                 (0x0000000EUL)              /*!< Color Multiplexing */
#define DSI_WCFGR_COLMUX0                (0x00000002UL)
#define DSI_WCFGR_COLMUX1                (0x00000004UL)
#define DSI_WCFGR_COLMUX2                (0x00000008UL)

#define DSI_WCFGR_TESRC                  (0x00000010UL)              /*!< Tearing Effect Source */
#define DSI_WCFGR_TEPOL                  (0x00000020UL)              /*!< Tearing Effect Polarity */
#define DSI_WCFGR_AR                     (0x00000040UL)              /*!< Automatic Refresh */
#define DSI_WCFGR_VSPOL                  (0x00000080UL)              /*!< VSync Polarity */

/*******************  Bit definition for DSI_WCR register  *****************/
#define DSI_WCR_COLM                     (0x00000001UL)              /*!< Color Mode */
#define DSI_WCR_SHTDN                    (0x00000002UL)              /*!< Shutdown */
#define DSI_WCR_LTDCEN                   (0x00000004UL)              /*!< LTDC Enable */
#define DSI_WCR_DSIEN                    (0x00000008UL)              /*!< DSI Enable */

/*******************  Bit definition for DSI_WIER register  ****************/
#define DSI_WIER_TEIE                    (0x00000001UL)              /*!< Tearing Effect Interrupt Enable */
#define DSI_WIER_ERIE                    (0x00000002UL)              /*!< End of Refresh Interrupt Enable */
#define DSI_WIER_PLLLIE                  (0x00000200UL)              /*!< PLL Lock Interrupt Enable */
#define DSI_WIER_PLLUIE                  (0x00000400UL)              /*!< PLL Unlock Interrupt Enable */
#define DSI_WIER_RRIE                    (0x00002000UL)              /*!< Regulator Ready Interrupt Enable */

/*******************  Bit definition for DSI_WISR register  ****************/
#define DSI_WISR_TEIF                    (0x00000001UL)              /*!< Tearing Effect Interrupt Flag */
#define DSI_WISR_ERIF                    (0x00000002UL)              /*!< End of Refresh Interrupt Flag */
#define DSI_WISR_BUSY                    (0x00000004UL)              /*!< Busy Flag */
#define DSI_WISR_PLLLS                   (0x00000100UL)              /*!< PLL Lock Status */
#define DSI_WISR_PLLLIF                  (0x00000200UL)              /*!< PLL Lock Interrupt Flag */
#define DSI_WISR_PLLUIF                  (0x00000400UL)              /*!< PLL Unlock Interrupt Flag */
#define DSI_WISR_RRS                     (0x00001000UL)              /*!< Regulator Ready Flag */
#define DSI_WISR_RRIF                    (0x00002000UL)              /*!< Regulator Ready Interrupt Flag */

/*******************  Bit definition for DSI_WIFCR register  ***************/
#define DSI_WIFCR_CTEIF                  (0x00000001UL)              /*!< Clear Tearing Effect Interrupt Flag */
#define DSI_WIFCR_CERIF                  (0x00000002UL)              /*!< Clear End of Refresh Interrupt Flag */
#define DSI_WIFCR_CPLLLIF                (0x00000200UL)              /*!< Clear PLL Lock Interrupt Flag */
#define DSI_WIFCR_CPLLUIF                (0x00000400UL)              /*!< Clear PLL Unlock Interrupt Flag */
#define DSI_WIFCR_CRRIF                  (0x00002000UL)              /*!< Clear Regulator Ready Interrupt Flag */

/*******************  Bit definition for DSI_WPCR0 register  ***************/
#define DSI_WPCR0_UIX4                   (0x0000003FUL)              /*!< Unit Interval multiplied by 4 */
#define DSI_WPCR0_UIX4_0                 (0x00000001UL)
#define DSI_WPCR0_UIX4_1                 (0x00000002UL)
#define DSI_WPCR0_UIX4_2                 (0x00000004UL)
#define DSI_WPCR0_UIX4_3                 (0x00000008UL)
#define DSI_WPCR0_UIX4_4                 (0x00000010UL)
#define DSI_WPCR0_UIX4_5                 (0x00000020UL)

#define DSI_WPCR0_SWCL                   (0x00000040UL)              /*!< Swap pins on clock lane */
#define DSI_WPCR0_SWDL0                  (0x00000080UL)              /*!< Swap pins on data lane 1 */
#define DSI_WPCR0_SWDL1                  (0x00000100UL)              /*!< Swap pins on data lane 2 */
#define DSI_WPCR0_HSICL                  (0x00000200UL)              /*!< Invert the high-speed data signal on clock lane */
#define DSI_WPCR0_HSIDL0                 (0x00000400UL)              /*!< Invert the high-speed data signal on lane 1 */
#define DSI_WPCR0_HSIDL1                 (0x00000800UL)              /*!< Invert the high-speed data signal on lane 2 */
#define DSI_WPCR0_FTXSMCL                (0x00001000UL)              /*!< Force clock lane in TX stop mode */
#define DSI_WPCR0_FTXSMDL                (0x00002000UL)              /*!< Force data lanes in TX stop mode */
#define DSI_WPCR0_CDOFFDL                (0x00004000UL)              /*!< Contention detection OFF */
#define DSI_WPCR0_TDDL                   (0x00010000UL)              /*!< Turn Disable Data Lanes */
#define DSI_WPCR0_PDEN                   (0x00040000UL)              /*!< Pull-Down Enable */
#define DSI_WPCR0_TCLKPREPEN             (0x00080000UL)              /*!< Timer for t-CLKPREP Enable */
#define DSI_WPCR0_TCLKZEROEN             (0x00100000UL)              /*!< Timer for t-CLKZERO Enable */
#define DSI_WPCR0_THSPREPEN              (0x00200000UL)              /*!< Timer for t-HSPREP Enable */
#define DSI_WPCR0_THSTRAILEN             (0x00400000UL)              /*!< Timer for t-HSTRAIL Enable */
#define DSI_WPCR0_THSZEROEN              (0x00800000UL)              /*!< Timer for t-HSZERO Enable */
#define DSI_WPCR0_TLPXDEN                (0x01000000UL)              /*!< Timer for t-LPXD Enable */
#define DSI_WPCR0_THSEXITEN              (0x02000000UL)              /*!< Timer for t-HSEXIT Enable */
#define DSI_WPCR0_TLPXCEN                (0x04000000UL)              /*!< Timer for t-LPXC Enable */
#define DSI_WPCR0_TCLKPOSTEN             (0x08000000UL)              /*!< Timer for t-CLKPOST Enable */

/*******************  Bit definition for DSI_WPCR1 register  ***************/
#define DSI_WPCR1_HSTXDCL                (0x00000003UL)              /*!< High-Speed Transmission Delay on Clock Lane */
#define DSI_WPCR1_HSTXDCL0               (0x00000001UL)
#define DSI_WPCR1_HSTXDCL1               (0x00000002UL)

#define DSI_WPCR1_HSTXDDL                (0x0000000CUL)              /*!< High-Speed Transmission Delay on Data Lane */
#define DSI_WPCR1_HSTXDDL0               (0x00000004UL)
#define DSI_WPCR1_HSTXDDL1               (0x00000008UL)

#define DSI_WPCR1_LPSRCCL                (0x000000C0UL)              /*!< Low-Power transmission Slew Rate Compensation on Clock Lane */
#define DSI_WPCR1_LPSRCCL0               (0x00000040UL)
#define DSI_WPCR1_LPSRCCL1               (0x00000080UL)

#define DSI_WPCR1_LPSRCDL                (0x00000300UL)              /*!< Low-Power transmission Slew Rate Compensation on Data Lane */
#define DSI_WPCR1_LPSRCDL0               (0x00000100UL)
#define DSI_WPCR1_LPSRCDL1               (0x00000200UL)

#define DSI_WPCR1_SDDC                   (0x00001000UL)              /*!< SDD Control */

#define DSI_WPCR1_LPRXVCDL               (0x0000C000UL)              /*!< Low-Power Reception V-IL Compensation on Data Lanes */
#define DSI_WPCR1_LPRXVCDL0              (0x00004000UL)
#define DSI_WPCR1_LPRXVCDL1              (0x00008000UL)

#define DSI_WPCR1_HSTXSRCCL              (0x00030000UL)              /*!< High-Speed Transmission Delay on Clock Lane */
#define DSI_WPCR1_HSTXSRCCL0             (0x00010000UL)
#define DSI_WPCR1_HSTXSRCCL1             (0x00020000UL)

#define DSI_WPCR1_HSTXSRCDL              (0x000C0000UL)              /*!< High-Speed Transmission Delay on Data Lane */
#define DSI_WPCR1_HSTXSRCDL0             (0x00040000UL)
#define DSI_WPCR1_HSTXSRCDL1             (0x00080000UL)

#define DSI_WPCR1_FLPRXLPM               (0x00400000UL)              /*!< Forces LP Receiver in Low-Power Mode */

#define DSI_WPCR1_LPRXFT                 (0x06000000UL)              /*!< Low-Power RX low-pass Filtering Tuning */
#define DSI_WPCR1_LPRXFT0                (0x02000000UL)
#define DSI_WPCR1_LPRXFT1                (0x04000000UL)

/*******************  Bit definition for DSI_WPCR2 register  ***************/
#define DSI_WPCR2_TCLKPREP               (0x000000FFUL)              /*!< t-CLKPREP */
#define DSI_WPCR2_TCLKPREP0              (0x00000001UL)
#define DSI_WPCR2_TCLKPREP1              (0x00000002UL)
#define DSI_WPCR2_TCLKPREP2              (0x00000004UL)
#define DSI_WPCR2_TCLKPREP3              (0x00000008UL)
#define DSI_WPCR2_TCLKPREP4              (0x00000010UL)
#define DSI_WPCR2_TCLKPREP5              (0x00000020UL)
#define DSI_WPCR2_TCLKPREP6              (0x00000040UL)
#define DSI_WPCR2_TCLKPREP7              (0x00000080UL)

#define DSI_WPCR2_TCLKZERO               (0x0000FF00UL)              /*!< t-CLKZERO */
#define DSI_WPCR2_TCLKZERO0              (0x00000100UL)
#define DSI_WPCR2_TCLKZERO1              (0x00000200UL)
#define DSI_WPCR2_TCLKZERO2              (0x00000400UL)
#define DSI_WPCR2_TCLKZERO3              (0x00000800UL)
#define DSI_WPCR2_TCLKZERO4              (0x00001000UL)
#define DSI_WPCR2_TCLKZERO5              (0x00002000UL)
#define DSI_WPCR2_TCLKZERO6              (0x00004000UL)
#define DSI_WPCR2_TCLKZERO7              (0x00008000UL)

#define DSI_WPCR2_THSPREP                (0x00FF0000UL)              /*!< t-HSPREP */
#define DSI_WPCR2_THSPREP0               (0x00010000UL)
#define DSI_WPCR2_THSPREP1               (0x00020000UL)
#define DSI_WPCR2_THSPREP2               (0x00040000UL)
#define DSI_WPCR2_THSPREP3               (0x00080000UL)
#define DSI_WPCR2_THSPREP4               (0x00100000UL)
#define DSI_WPCR2_THSPREP5               (0x00200000UL)
#define DSI_WPCR2_THSPREP6               (0x00400000UL)
#define DSI_WPCR2_THSPREP7               (0x00800000UL)

#define DSI_WPCR2_THSTRAIL               (0xFF000000UL)              /*!< t-HSTRAIL */
#define DSI_WPCR2_THSTRAIL0              (0x01000000UL)
#define DSI_WPCR2_THSTRAIL1              (0x02000000UL)
#define DSI_WPCR2_THSTRAIL2              (0x04000000UL)
#define DSI_WPCR2_THSTRAIL3              (0x08000000UL)
#define DSI_WPCR2_THSTRAIL4              (0x10000000UL)
#define DSI_WPCR2_THSTRAIL5              (0x20000000UL)
#define DSI_WPCR2_THSTRAIL6              (0x40000000UL)
#define DSI_WPCR2_THSTRAIL7              (0x80000000UL)

/*******************  Bit definition for DSI_WPCR3 register  ***************/
#define DSI_WPCR3_THSZERO                (0x000000FFUL)              /*!< t-HSZERO */
#define DSI_WPCR3_THSZERO0               (0x00000001UL)
#define DSI_WPCR3_THSZERO1               (0x00000002UL)
#define DSI_WPCR3_THSZERO2               (0x00000004UL)
#define DSI_WPCR3_THSZERO3               (0x00000008UL)
#define DSI_WPCR3_THSZERO4               (0x00000010UL)
#define DSI_WPCR3_THSZERO5               (0x00000020UL)
#define DSI_WPCR3_THSZERO6               (0x00000040UL)
#define DSI_WPCR3_THSZERO7               (0x00000080UL)

#define DSI_WPCR3_TLPXD                  (0x0000FF00UL)              /*!< t-LPXD */
#define DSI_WPCR3_TLPXD0                 (0x00000100UL)
#define DSI_WPCR3_TLPXD1                 (0x00000200UL)
#define DSI_WPCR3_TLPXD2                 (0x00000400UL)
#define DSI_WPCR3_TLPXD3                 (0x00000800UL)
#define DSI_WPCR3_TLPXD4                 (0x00001000UL)
#define DSI_WPCR3_TLPXD5                 (0x00002000UL)
#define DSI_WPCR3_TLPXD6                 (0x00004000UL)
#define DSI_WPCR3_TLPXD7                 (0x00008000UL)

#define DSI_WPCR3_THSEXIT                (0x00FF0000UL)              /*!< t-HSEXIT */
#define DSI_WPCR3_THSEXIT0               (0x00010000UL)
#define DSI_WPCR3_THSEXIT1               (0x00020000UL)
#define DSI_WPCR3_THSEXIT2               (0x00040000UL)
#define DSI_WPCR3_THSEXIT3               (0x00080000UL)
#define DSI_WPCR3_THSEXIT4               (0x00100000UL)
#define DSI_WPCR3_THSEXIT5               (0x00200000UL)
#define DSI_WPCR3_THSEXIT6               (0x00400000UL)
#define DSI_WPCR3_THSEXIT7               (0x00800000UL)

#define DSI_WPCR3_TLPXC                  (0xFF000000UL)              /*!< t-LPXC */
#define DSI_WPCR3_TLPXC0                 (0x01000000UL)
#define DSI_WPCR3_TLPXC1                 (0x02000000UL)
#define DSI_WPCR3_TLPXC2                 (0x04000000UL)
#define DSI_WPCR3_TLPXC3                 (0x08000000UL)
#define DSI_WPCR3_TLPXC4                 (0x10000000UL)
#define DSI_WPCR3_TLPXC5                 (0x20000000UL)
#define DSI_WPCR3_TLPXC6                 (0x40000000UL)
#define DSI_WPCR3_TLPXC7                 (0x80000000UL)

/*******************  Bit definition for DSI_WPCR4 register  ***************/
#define DSI_WPCR4_TCLKPOST               (0x000000FFUL)              /*!< t-CLKPOST */
#define DSI_WPCR4_TCLKPOST0              (0x00000001UL)
#define DSI_WPCR4_TCLKPOST1              (0x00000002UL)
#define DSI_WPCR4_TCLKPOST2              (0x00000004UL)
#define DSI_WPCR4_TCLKPOST3              (0x00000008UL)
#define DSI_WPCR4_TCLKPOST4              (0x00000010UL)
#define DSI_WPCR4_TCLKPOST5              (0x00000020UL)
#define DSI_WPCR4_TCLKPOST6              (0x00000040UL)
#define DSI_WPCR4_TCLKPOST7              (0x00000080UL)

/*******************  Bit definition for DSI_WRPCR register  ***************/
#define DSI_WRPCR_PLLEN                  (0x00000001UL)              /*!< PLL Enable */
#define DSI_WRPCR_PLL_NDIV               (0x000001FCUL)              /*!< PLL Loop Division Factor */
#define DSI_WRPCR_PLL_NDIV0              (0x00000004UL)
#define DSI_WRPCR_PLL_NDIV1              (0x00000008UL)
#define DSI_WRPCR_PLL_NDIV2              (0x00000010UL)
#define DSI_WRPCR_PLL_NDIV3              (0x00000020UL)
#define DSI_WRPCR_PLL_NDIV4              (0x00000040UL)
#define DSI_WRPCR_PLL_NDIV5              (0x00000080UL)
#define DSI_WRPCR_PLL_NDIV6              (0x00000100UL)

#define DSI_WRPCR_PLL_IDF                (0x00007800UL)              /*!< PLL Input Division Factor */
#define DSI_WRPCR_PLL_IDF0               (0x00000800UL)
#define DSI_WRPCR_PLL_IDF1               (0x00001000UL)
#define DSI_WRPCR_PLL_IDF2               (0x00002000UL)
#define DSI_WRPCR_PLL_IDF3               (0x00004000UL)

#define DSI_WRPCR_PLL_ODF                (0x00030000UL)              /*!< PLL Output Division Factor */
#define DSI_WRPCR_PLL_ODF0               (0x00010000UL)
#define DSI_WRPCR_PLL_ODF1               (0x00020000UL)

#define DSI_WRPCR_REGEN                  (0x01000000UL)              /*!< Regulator Enable */
#endif /* STM32F469_479xx */

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
#define  PWR_CR_LPUDS                        (0x00000400UL)     /*!< Low-Power Regulator in Stop under-drive mode               */
#define  PWR_CR_MRUDS                        (0x00000800UL)     /*!< Main regulator in Stop under-drive mode                    */

#define  PWR_CR_LPLVDS                       (0x00000400UL)     /*!< Low-power regulator Low Voltage in Deep Sleep mode         */
#define  PWR_CR_MRLVDS                       (0x00000800UL)     /*!< Main regulator Low Voltage in Deep Sleep mode              */

#define  PWR_CR_ADCDC1                       (0x00002000UL)     /*!< Refer to AN4073 on how to use this bit */

#define  PWR_CR_VOS                          (0x0000C000UL)     /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
#define  PWR_CR_VOS_0                        (0x00004000UL)     /*!< Bit 0 */
#define  PWR_CR_VOS_1                        (0x00008000UL)     /*!< Bit 1 */

#define  PWR_CR_ODEN                         (0x00010000UL)     /*!< Over Drive enable                   */
#define  PWR_CR_ODSWEN                       (0x00020000UL)     /*!< Over Drive switch enabled           */
#define  PWR_CR_UDEN                         (0x000C0000UL)     /*!< Under Drive enable in stop mode     */
#define  PWR_CR_UDEN_0                       (0x00040000UL)     /*!< Bit 0                               */
#define  PWR_CR_UDEN_1                       (0x00080000UL)     /*!< Bit 1                               */

#define  PWR_CR_FMSSR                        (0x00100000UL)     /*!< Flash Memory Sleep System Run        */
#define  PWR_CR_FISSR                        (0x00200000UL)     /*!< Flash Interface Stop while System Run */

/* Legacy define */
#define  PWR_CR_PMODE                        PWR_CR_VOS

/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         (0x00000001UL)     /*!< Wakeup Flag                                      */
#define  PWR_CSR_SBF                         (0x00000002UL)     /*!< Standby Flag                                     */
#define  PWR_CSR_PVDO                        (0x00000004UL)     /*!< PVD Output                                       */
#define  PWR_CSR_BRR                         (0x00000008UL)     /*!< Backup regulator ready                           */
#define  PWR_CSR_WUPP                        (0x00000080UL)     /*!< WKUP pin Polarity                                */
#define  PWR_CSR_EWUP                        (0x00000100UL)     /*!< Enable WKUP pin                                  */
#define  PWR_CSR_BRE                         (0x00000200UL)     /*!< Backup regulator enable                          */
#define  PWR_CSR_VOSRDY                      (0x00004000UL)     /*!< Regulator voltage scaling output selection ready */
#define  PWR_CSR_ODRDY                       (0x00010000UL)     /*!< Over Drive generator ready                       */
#define  PWR_CSR_ODSWRDY                     (0x00020000UL)     /*!< Over Drive Switch ready                          */
#define  PWR_CSR_UDSWRDY                     (0x000C0000UL)     /*!< Under Drive ready                                */

/* Legacy define */
#define  PWR_CSR_REGRDY                      PWR_CSR_VOSRDY

#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
/******************************************************************************/
/*                                                                            */
/*                                    QUADSPI                                 */
/*                                                                            */
/******************************************************************************/
/*****************  Bit definition for QUADSPI_CR register  *******************/
#define  QUADSPI_CR_EN                           (0x00000001UL)            /*!< Enable                             */
#define  QUADSPI_CR_ABORT                        (0x00000002UL)            /*!< Abort request                      */
#define  QUADSPI_CR_DMAEN                        (0x00000004UL)            /*!< DMA Enable                         */
#define  QUADSPI_CR_TCEN                         (0x00000008UL)            /*!< Timeout Counter Enable             */
#define  QUADSPI_CR_SSHIFT                       (0x00000010UL)            /*!< SSHIFT Sample Shift                */
#define  QUADSPI_CR_DFM                          (0x00000040UL)            /*!< Dual Flash Mode                    */
#define  QUADSPI_CR_FSEL                         (0x00000080UL)            /*!< Flash Select                       */
#define  QUADSPI_CR_FTHRES                       (0x00001F00UL)            /*!< FTHRES[3:0] FIFO Level             */
#define  QUADSPI_CR_FTHRES_0                     (0x00000100UL)            /*!< Bit 0 */
#define  QUADSPI_CR_FTHRES_1                     (0x00000200UL)            /*!< Bit 1 */
#define  QUADSPI_CR_FTHRES_2                     (0x00000400UL)            /*!< Bit 2 */
#define  QUADSPI_CR_FTHRES_3                     (0x00000800UL)            /*!< Bit 3 */
#define  QUADSPI_CR_FTHRES_4                     (0x00001000UL)            /*!< Bit 4 */
#define  QUADSPI_CR_TEIE                         (0x00010000UL)            /*!< Transfer Error Interrupt Enable    */
#define  QUADSPI_CR_TCIE                         (0x00020000UL)            /*!< Transfer Complete Interrupt Enable */
#define  QUADSPI_CR_FTIE                         (0x00040000UL)            /*!< FIFO Threshold Interrupt Enable    */
#define  QUADSPI_CR_SMIE                         (0x00080000UL)            /*!< Status Match Interrupt Enable      */
#define  QUADSPI_CR_TOIE                         (0x00100000UL)            /*!< TimeOut Interrupt Enable           */
#define  QUADSPI_CR_APMS                         (0x00400000UL)            /*!< Bit 1                              */
#define  QUADSPI_CR_PMM                          (0x00800000UL)            /*!< Polling Match Mode                 */
#define  QUADSPI_CR_PRESCALER                    (0xFF000000UL)            /*!< PRESCALER[7:0] Clock prescaler     */
#define  QUADSPI_CR_PRESCALER_0                  (0x01000000UL)            /*!< Bit 0 */
#define  QUADSPI_CR_PRESCALER_1                  (0x02000000UL)            /*!< Bit 1 */
#define  QUADSPI_CR_PRESCALER_2                  (0x04000000UL)            /*!< Bit 2 */
#define  QUADSPI_CR_PRESCALER_3                  (0x08000000UL)            /*!< Bit 3 */
#define  QUADSPI_CR_PRESCALER_4                  (0x10000000UL)            /*!< Bit 4 */
#define  QUADSPI_CR_PRESCALER_5                  (0x20000000UL)            /*!< Bit 5 */
#define  QUADSPI_CR_PRESCALER_6                  (0x40000000UL)            /*!< Bit 6 */
#define  QUADSPI_CR_PRESCALER_7                  (0x80000000UL)            /*!< Bit 7 */

/*****************  Bit definition for QUADSPI_DCR register  ******************/
#define  QUADSPI_DCR_CKMODE                      (0x00000001UL)            /*!< Mode 0 / Mode 3                 */
#define  QUADSPI_DCR_CSHT                        (0x00000700UL)            /*!< CSHT[2:0]: ChipSelect High Time */
#define  QUADSPI_DCR_CSHT_0                      (0x00000100UL)            /*!< Bit 0 */
#define  QUADSPI_DCR_CSHT_1                      (0x00000200UL)            /*!< Bit 1 */
#define  QUADSPI_DCR_CSHT_2                      (0x00000400UL)            /*!< Bit 2 */
#define  QUADSPI_DCR_FSIZE                       (0x001F0000UL)            /*!< FSIZE[4:0]: Flash Size          */
#define  QUADSPI_DCR_FSIZE_0                     (0x00010000UL)            /*!< Bit 0 */
#define  QUADSPI_DCR_FSIZE_1                     (0x00020000UL)            /*!< Bit 1 */
#define  QUADSPI_DCR_FSIZE_2                     (0x00040000UL)            /*!< Bit 2 */
#define  QUADSPI_DCR_FSIZE_3                     (0x00080000UL)            /*!< Bit 3 */
#define  QUADSPI_DCR_FSIZE_4                     (0x00100000UL)            /*!< Bit 4 */

/******************  Bit definition for QUADSPI_SR register  *******************/
#define  QUADSPI_SR_TEF                          (0x00000001UL)             /*!< Transfer Error Flag     */
#define  QUADSPI_SR_TCF                          (0x00000002UL)             /*!< Transfer Complete Flag  */
#define  QUADSPI_SR_FTF                          (0x00000004UL)             /*!< FIFO Threshlod Flag     */
#define  QUADSPI_SR_SMF                          (0x00000008UL)             /*!< Status Match Flag       */
#define  QUADSPI_SR_TOF                          (0x00000010UL)             /*!< Timeout Flag            */
#define  QUADSPI_SR_BUSY                         (0x00000020UL)             /*!< Busy                    */
#define  QUADSPI_SR_FLEVEL                       (0x00003F00UL)             /*!< FIFO Level              */
#define  QUADSPI_SR_FLEVEL_0                     (0x00000100UL)             /*!< Bit 0 */
#define  QUADSPI_SR_FLEVEL_1                     (0x00000200UL)             /*!< Bit 1 */
#define  QUADSPI_SR_FLEVEL_2                     (0x00000400UL)             /*!< Bit 2 */
#define  QUADSPI_SR_FLEVEL_3                     (0x00000800UL)             /*!< Bit 3 */
#define  QUADSPI_SR_FLEVEL_4                     (0x00001000UL)             /*!< Bit 4 */
#define  QUADSPI_SR_FLEVEL_5                     (0x00002000UL)             /*!< Bit 5 */

/******************  Bit definition for QUADSPI_FCR register  ******************/
#define  QUADSPI_FCR_CTEF                        (0x00000001UL)             /*!< Clear Transfer Error Flag    */
#define  QUADSPI_FCR_CTCF                        (0x00000002UL)             /*!< Clear Transfer Complete Flag */
#define  QUADSPI_FCR_CSMF                        (0x00000008UL)             /*!< Clear Status Match Flag      */
#define  QUADSPI_FCR_CTOF                        (0x00000010UL)             /*!< Clear Timeout Flag           */

/******************  Bit definition for QUADSPI_DLR register  ******************/
#define  QUADSPI_DLR_DL                        (0xFFFFFFFFUL)               /*!< DL[31:0]: Data Length */

/******************  Bit definition for QUADSPI_CCR register  ******************/
#define  QUADSPI_CCR_INSTRUCTION                  (0x000000FFUL)            /*!< INSTRUCTION[7:0]: Instruction */
#define  QUADSPI_CCR_INSTRUCTION_0                (0x00000001UL)            /*!< Bit 0 */
#define  QUADSPI_CCR_INSTRUCTION_1                (0x00000002UL)            /*!< Bit 1 */
#define  QUADSPI_CCR_INSTRUCTION_2                (0x00000004UL)            /*!< Bit 2 */
#define  QUADSPI_CCR_INSTRUCTION_3                (0x00000008UL)            /*!< Bit 3 */
#define  QUADSPI_CCR_INSTRUCTION_4                (0x00000010UL)            /*!< Bit 4 */
#define  QUADSPI_CCR_INSTRUCTION_5                (0x00000020UL)            /*!< Bit 5 */
#define  QUADSPI_CCR_INSTRUCTION_6                (0x00000040UL)            /*!< Bit 6 */
#define  QUADSPI_CCR_INSTRUCTION_7                (0x00000080UL)            /*!< Bit 7 */
#define  QUADSPI_CCR_IMODE                        (0x00000300UL)            /*!< IMODE[1:0]: Instruction Mode */
#define  QUADSPI_CCR_IMODE_0                      (0x00000100UL)            /*!< Bit 0 */
#define  QUADSPI_CCR_IMODE_1                      (0x00000200UL)            /*!< Bit 1 */
#define  QUADSPI_CCR_ADMODE                       (0x00000C00UL)            /*!< ADMODE[1:0]: Address Mode */
#define  QUADSPI_CCR_ADMODE_0                     (0x00000400UL)            /*!< Bit 0 */
#define  QUADSPI_CCR_ADMODE_1                     (0x00000800UL)            /*!< Bit 1 */
#define  QUADSPI_CCR_ADSIZE                       (0x00003000UL)            /*!< ADSIZE[1:0]: Address Size */
#define  QUADSPI_CCR_ADSIZE_0                     (0x00001000UL)            /*!< Bit 0 */
#define  QUADSPI_CCR_ADSIZE_1                     (0x00002000UL)            /*!< Bit 1 */
#define  QUADSPI_CCR_ABMODE                       (0x0000C000UL)            /*!< ABMODE[1:0]: Alternate Bytes Mode */
#define  QUADSPI_CCR_ABMODE_0                     (0x00004000UL)            /*!< Bit 0 */
#define  QUADSPI_CCR_ABMODE_1                     (0x00008000UL)            /*!< Bit 1 */
#define  QUADSPI_CCR_ABSIZE                       (0x00030000UL)            /*!< ABSIZE[1:0]: Instruction Mode */
#define  QUADSPI_CCR_ABSIZE_0                     (0x00010000UL)            /*!< Bit 0 */
#define  QUADSPI_CCR_ABSIZE_1                     (0x00020000UL)            /*!< Bit 1 */
#define  QUADSPI_CCR_DCYC                         (0x007C0000UL)            /*!< DCYC[4:0]: Dummy Cycles */
#define  QUADSPI_CCR_DCYC_0                       (0x00040000UL)            /*!< Bit 0 */
#define  QUADSPI_CCR_DCYC_1                       (0x00080000UL)            /*!< Bit 1 */
#define  QUADSPI_CCR_DCYC_2                       (0x00100000UL)            /*!< Bit 2 */
#define  QUADSPI_CCR_DCYC_3                       (0x00200000UL)            /*!< Bit 3 */
#define  QUADSPI_CCR_DCYC_4                       (0x00400000UL)            /*!< Bit 4 */
#define  QUADSPI_CCR_DMODE                        (0x03000000UL)            /*!< DMODE[1:0]: Data Mode */
#define  QUADSPI_CCR_DMODE_0                      (0x01000000UL)            /*!< Bit 0 */
#define  QUADSPI_CCR_DMODE_1                      (0x02000000UL)            /*!< Bit 1 */
#define  QUADSPI_CCR_FMODE                        (0x0C000000UL)            /*!< FMODE[1:0]: Functional Mode */
#define  QUADSPI_CCR_FMODE_0                      (0x04000000UL)            /*!< Bit 0 */
#define  QUADSPI_CCR_FMODE_1                      (0x08000000UL)            /*!< Bit 1 */
#define  QUADSPI_CCR_SIOO                         (0x10000000UL)            /*!< SIOO: Send Instruction Only Once Mode */
#define  QUADSPI_CCR_DHHC                         (0x40000000UL)            /*!< DHHC: Delay Half Hclk Cycle */
#define  QUADSPI_CCR_DDRM                         (0x80000000UL)            /*!< DDRM: Double Data Rate Mode */
/******************  Bit definition for QUADSPI_AR register  *******************/
#define  QUADSPI_AR_ADDRESS                       (0xFFFFFFFFUL)            /*!< ADDRESS[31:0]: Address */

/******************  Bit definition for QUADSPI_ABR register  ******************/
#define  QUADSPI_ABR_ALTERNATE                    (0xFFFFFFFFUL)            /*!< ALTERNATE[31:0]: Alternate Bytes */

/******************  Bit definition for QUADSPI_DR register  *******************/
#define  QUADSPI_DR_DATA                          (0xFFFFFFFFUL)            /*!< DATA[31:0]: Data */

/******************  Bit definition for QUADSPI_PSMKR register  ****************/
#define  QUADSPI_PSMKR_MASK                       (0xFFFFFFFFUL)            /*!< MASK[31:0]: Status Mask */

/******************  Bit definition for QUADSPI_PSMAR register  ****************/
#define  QUADSPI_PSMAR_MATCH                      (0xFFFFFFFFUL)            /*!< MATCH[31:0]: Status Match */

/******************  Bit definition for QUADSPI_PIR register  *****************/
#define  QUADSPI_PIR_INTERVAL                     (0x0000FFFFUL)            /*!< INTERVAL[15:0]: Polling Interval */

/******************  Bit definition for QUADSPI_LPTR register  *****************/
#define  QUADSPI_LPTR_TIMEOUT                     (0x0000FFFFUL)            /*!< TIMEOUT[15:0]: Timeout period */
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx || STM32F469_479xx */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        (0x00000001UL)
#define  RCC_CR_HSIRDY                       (0x00000002UL)

#define  RCC_CR_HSITRIM                      (0x000000F8UL)
#define  RCC_CR_HSITRIM_0                    (0x00000008UL)
#define  RCC_CR_HSITRIM_1                    (0x00000010UL)
#define  RCC_CR_HSITRIM_2                    (0x00000020UL)
#define  RCC_CR_HSITRIM_3                    (0x00000040UL)
#define  RCC_CR_HSITRIM_4                    (0x00000080UL)

#define  RCC_CR_HSICAL                       (0x0000FF00UL)
#define  RCC_CR_HSICAL_0                     (0x00000100UL)
#define  RCC_CR_HSICAL_1                     (0x00000200UL)
#define  RCC_CR_HSICAL_2                     (0x00000400UL)
#define  RCC_CR_HSICAL_3                     (0x00000800UL)
#define  RCC_CR_HSICAL_4                     (0x00001000UL)
#define  RCC_CR_HSICAL_5                     (0x00002000UL)
#define  RCC_CR_HSICAL_6                     (0x00004000UL)
#define  RCC_CR_HSICAL_7                     (0x00008000UL)

#define  RCC_CR_HSEON                        (0x00010000UL)
#define  RCC_CR_HSERDY                       (0x00020000UL)
#define  RCC_CR_HSEBYP                       (0x00040000UL)
#define  RCC_CR_CSSON                        (0x00080000UL)
#define  RCC_CR_PLLON                        (0x01000000UL)
#define  RCC_CR_PLLRDY                       (0x02000000UL)
#define  RCC_CR_PLLI2SON                     (0x04000000UL)
#define  RCC_CR_PLLI2SRDY                    (0x08000000UL)
#define  RCC_CR_PLLSAION                     (0x10000000UL)
#define  RCC_CR_PLLSAIRDY                    (0x20000000UL)

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

#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define  RCC_PLLCFGR_PLLR                    (0x70000000UL)
#define  RCC_PLLCFGR_PLLR_0                  (0x10000000UL)
#define  RCC_PLLCFGR_PLLR_1                  (0x20000000UL)
#define  RCC_PLLCFGR_PLLR_2                  (0x40000000UL)
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx || STM32F469_479xx */

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         (0x00000003UL)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       (0x00000001UL)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       (0x00000002UL)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     (0x00000000UL)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     (0x00000001UL)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     (0x00000002UL)        /*!< PLL/PLLP selected as system clock */
#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define  RCC_CFGR_SW_PLLR                    (0x00000003UL)        /*!< PLL/PLLR selected as system clock */
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx || STM32F469_479xx */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        (0x0000000CUL)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      (0x00000004UL)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      (0x00000008UL)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    (0x00000000UL)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    (0x00000004UL)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    (0x00000008UL)        /*!< PLL/PLLP used as system clock       */
#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F469_479xx) || defined(STM32F446xx)
#define  RCC_CFGR_SWS_PLLR                   (0x0000000CUL)        /*!< PLL/PLLR used as system clock       */
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx || STM32F469_479xx */

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

#if defined(STM32F410xx)
/*!< MCO1EN configuration */
#define  RCC_CFGR_MCO1EN                     (0x00000100UL)        /*!< MCO1EN bit */
/*!< MCO1EN configuration */
#define  RCC_CFGR_MCO2EN                     (0x00000200UL)        /*!< MCO2EN bit */
#endif /* STM32F410xx */
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
#define  RCC_CIR_PLLSAIRDYF                  (0x00000040UL)
#define  RCC_CIR_CSSF                        (0x00000080UL)
#define  RCC_CIR_LSIRDYIE                    (0x00000100UL)
#define  RCC_CIR_LSERDYIE                    (0x00000200UL)
#define  RCC_CIR_HSIRDYIE                    (0x00000400UL)
#define  RCC_CIR_HSERDYIE                    (0x00000800UL)
#define  RCC_CIR_PLLRDYIE                    (0x00001000UL)
#define  RCC_CIR_PLLI2SRDYIE                 (0x00002000UL)
#define  RCC_CIR_PLLSAIRDYIE                 (0x00004000UL)
#define  RCC_CIR_LSIRDYC                     (0x00010000UL)
#define  RCC_CIR_LSERDYC                     (0x00020000UL)
#define  RCC_CIR_HSIRDYC                     (0x00040000UL)
#define  RCC_CIR_HSERDYC                     (0x00080000UL)
#define  RCC_CIR_PLLRDYC                     (0x00100000UL)
#define  RCC_CIR_PLLI2SRDYC                  (0x00200000UL)
#define  RCC_CIR_PLLSAIRDYC                  (0x00400000UL)
#define  RCC_CIR_CSSC                        (0x00800000UL)

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define  RCC_AHB1RSTR_GPIOARST               (0x00000001UL)
#define  RCC_AHB1RSTR_GPIOBRST               (0x00000002UL)
#define  RCC_AHB1RSTR_GPIOCRST               (0x00000004UL)
#define  RCC_AHB1RSTR_GPIODRST               (0x00000008UL)
#define  RCC_AHB1RSTR_GPIOERST               (0x00000010UL)
#define  RCC_AHB1RSTR_GPIOFRST               (0x00000020UL)
#define  RCC_AHB1RSTR_GPIOGRST               (0x00000040UL)
#define  RCC_AHB1RSTR_GPIOHRST               (0x00000080UL)
#define  RCC_AHB1RSTR_GPIOIRST               (0x00000100UL)
#define  RCC_AHB1RSTR_GPIOJRST               (0x00000200UL)
#define  RCC_AHB1RSTR_GPIOKRST               (0x00000400UL)
#define  RCC_AHB1RSTR_CRCRST                 (0x00001000UL)
#define  RCC_AHB1RSTR_DMA1RST                (0x00200000UL)
#define  RCC_AHB1RSTR_DMA2RST                (0x00400000UL)
#define  RCC_AHB1RSTR_DMA2DRST               (0x00800000UL)
#define  RCC_AHB1RSTR_ETHMACRST              (0x02000000UL)
#define  RCC_AHB1RSTR_OTGHRST                (0x10000000UL)

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define  RCC_AHB2RSTR_DCMIRST                (0x00000001UL)
#define  RCC_AHB2RSTR_CRYPRST                (0x00000010UL)
#define  RCC_AHB2RSTR_HASHRST                (0x00000020UL)
 /* maintained for legacy purpose */
#define  RCC_AHB2RSTR_HSAHRST                RCC_AHB2RSTR_HASHRST
#define  RCC_AHB2RSTR_RNGRST                 (0x00000040UL)
#define  RCC_AHB2RSTR_OTGFSRST               (0x00000080UL)

/********************  Bit definition for RCC_AHB3RSTR register  **************/
#if defined(STM32F40_41xxx) || defined(STM32F412xG) || defined(STM32F413_423xx)
#define  RCC_AHB3RSTR_FSMCRST                (0x00000001UL)
#endif /* STM32F40_41xxx || STM32F412xG || STM32F413_423xx */

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define  RCC_AHB3RSTR_FMCRST                (0x00000001UL)
#endif /* STM32F427_437xx ||  STM32F429_439xx || STM32F446xx || STM32F469_479xx */
#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define  RCC_AHB3RSTR_QSPIRST               (0x00000002UL)
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx || STM32F469_479xx */

/********************  Bit definition for RCC_APB1RSTR register  **************/
#define  RCC_APB1RSTR_TIM2RST                (0x00000001UL)
#define  RCC_APB1RSTR_TIM3RST                (0x00000002UL)
#define  RCC_APB1RSTR_TIM4RST                (0x00000004UL)
#define  RCC_APB1RSTR_TIM5RST                (0x00000008UL)
#define  RCC_APB1RSTR_TIM6RST                (0x00000010UL)
#define  RCC_APB1RSTR_TIM7RST                (0x00000020UL)
#define  RCC_APB1RSTR_TIM12RST               (0x00000040UL)
#define  RCC_APB1RSTR_TIM13RST               (0x00000080UL)
#define  RCC_APB1RSTR_TIM14RST               (0x00000100UL)
#if defined(STM32F410xx) || defined(STM32F413_423xx)
#define  RCC_APB1RSTR_LPTIM1RST              (0x00000200UL)
#endif /* STM32F410xx || STM32F413_423xx */
#define  RCC_APB1RSTR_WWDGRST                (0x00000800UL)
#define  RCC_APB1RSTR_SPI2RST                (0x00004000UL)
#define  RCC_APB1RSTR_SPI3RST                (0x00008000UL)
#if defined(STM32F446xx)
#define  RCC_APB1RSTR_SPDIFRXRST             (0x00010000UL)
#endif /* STM32F446xx */
#define  RCC_APB1RSTR_USART2RST              (0x00020000UL)
#define  RCC_APB1RSTR_USART3RST              (0x00040000UL)
#define  RCC_APB1RSTR_UART4RST               (0x00080000UL)
#define  RCC_APB1RSTR_UART5RST               (0x00100000UL)
#define  RCC_APB1RSTR_I2C1RST                (0x00200000UL)
#define  RCC_APB1RSTR_I2C2RST                (0x00400000UL)
#define  RCC_APB1RSTR_I2C3RST                (0x00800000UL)
#if defined(STM32F410xx) || defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
#define  RCC_APB1RSTR_FMPI2C1RST             (0x01000000UL)
#endif /* STM32F410xx || STM32F412xG || STM32F413_423xx || STM32F446xx */
#define  RCC_APB1RSTR_CAN1RST                (0x02000000UL)
#define  RCC_APB1RSTR_CAN2RST                (0x04000000UL)
#if defined(STM32F446xx)
#define  RCC_APB1RSTR_CECRST                 (0x08000000UL)
#endif /* STM32F446xx */
#define  RCC_APB1RSTR_PWRRST                 (0x10000000UL)
#define  RCC_APB1RSTR_DACRST                 (0x20000000UL)
#define  RCC_APB1RSTR_UART7RST               (0x40000000UL)
#define  RCC_APB1RSTR_UART8RST               (0x80000000UL)

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define  RCC_APB2RSTR_TIM1RST                (0x00000001UL)
#define  RCC_APB2RSTR_TIM8RST                (0x00000002UL)
#define  RCC_APB2RSTR_USART1RST              (0x00000010UL)
#define  RCC_APB2RSTR_USART6RST              (0x00000020UL)
#define  RCC_APB2RSTR_UART9RST               (0x00000040UL)
#define  RCC_APB2RSTR_UART10RST              (0x00000080UL)
#define  RCC_APB2RSTR_ADCRST                 (0x00000100UL)
#define  RCC_APB2RSTR_SDIORST                (0x00000800UL)
#define  RCC_APB2RSTR_SPI1RST                (0x00001000UL)
#define  RCC_APB2RSTR_SPI4RST                (0x00002000UL)
#define  RCC_APB2RSTR_SYSCFGRST              (0x00004000UL)
#define  RCC_APB2RSTR_TIM9RST                (0x00010000UL)
#define  RCC_APB2RSTR_TIM10RST               (0x00020000UL)
#define  RCC_APB2RSTR_TIM11RST               (0x00040000UL)
#define  RCC_APB2RSTR_SPI5RST                (0x00100000UL)
#define  RCC_APB2RSTR_SPI6RST                (0x00200000UL)
#define  RCC_APB2RSTR_SAI1RST                (0x00400000UL)
#if defined(STM32F446xx)
#define  RCC_APB2RSTR_SAI2RST                (0x00800000UL)
#endif /* STM32F446xx */
#define  RCC_APB2RSTR_LTDCRST                (0x04000000UL)
#if defined(STM32F469_479xx)
#define  RCC_APB2RSTR_DSIRST                 (0x08000000UL)
#endif /* STM32F469_479xx */
#if defined(STM32F412xG) || defined(STM32F413_423xx)
#define  RCC_APB2RSTR_DFSDM1RST              (0x01000000UL)
#endif /* STM32F412xG || STM32F413_423xx */

#if defined(STM32F413_423xx)
#define  RCC_APB2RSTR_DFSDM2RST              (0x02000000UL)
#endif /* STM32F413_423xx */
/* Old definitions, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                   RCC_APB2RSTR_SPI1RST
#define  RCC_APB2RSTR_DFSDMRST               RCC_APB2RSTR_DFSDM1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_GPIOAEN                 (0x00000001UL)
#define  RCC_AHB1ENR_GPIOBEN                 (0x00000002UL)
#define  RCC_AHB1ENR_GPIOCEN                 (0x00000004UL)
#define  RCC_AHB1ENR_GPIODEN                 (0x00000008UL)
#define  RCC_AHB1ENR_GPIOEEN                 (0x00000010UL)
#define  RCC_AHB1ENR_GPIOFEN                 (0x00000020UL)
#define  RCC_AHB1ENR_GPIOGEN                 (0x00000040UL)
#define  RCC_AHB1ENR_GPIOHEN                 (0x00000080UL)
#define  RCC_AHB1ENR_GPIOIEN                 (0x00000100UL)
#define  RCC_AHB1ENR_GPIOJEN                 (0x00000200UL)
#define  RCC_AHB1ENR_GPIOKEN                 (0x00000400UL)
#define  RCC_AHB1ENR_CRCEN                   (0x00001000UL)
#define  RCC_AHB1ENR_BKPSRAMEN               (0x00040000UL)
#define  RCC_AHB1ENR_CCMDATARAMEN            (0x00100000UL)
#define  RCC_AHB1ENR_DMA1EN                  (0x00200000UL)
#define  RCC_AHB1ENR_DMA2EN                  (0x00400000UL)
#define  RCC_AHB1ENR_DMA2DEN                 (0x00800000UL)
#define  RCC_AHB1ENR_ETHMACEN                (0x02000000UL)
#define  RCC_AHB1ENR_ETHMACTXEN              (0x04000000UL)
#define  RCC_AHB1ENR_ETHMACRXEN              (0x08000000UL)
#define  RCC_AHB1ENR_ETHMACPTPEN             (0x10000000UL)
#define  RCC_AHB1ENR_OTGHSEN                 (0x20000000UL)
#define  RCC_AHB1ENR_OTGHSULPIEN             (0x40000000UL)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_DCMIEN                  (0x00000001UL)
#define  RCC_AHB2ENR_CRYPEN                  (0x00000010UL)
#define  RCC_AHB2ENR_HASHEN                  (0x00000020UL)
#define  RCC_AHB2ENR_RNGEN                   (0x00000040UL)
#define  RCC_AHB2ENR_OTGFSEN                 (0x00000080UL)

/********************  Bit definition for RCC_AHB3ENR register  ***************/

#if defined(STM32F40_41xxx) || defined(STM32F412xG) || defined(STM32F413_423xx)
#define  RCC_AHB3ENR_FSMCEN                  (0x00000001UL)
#endif /* STM32F40_41xxx || STM32F412xG || STM32F413_423xx */

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define  RCC_AHB3ENR_FMCEN                  (0x00000001UL)
#endif /* STM32F427_437xx ||  STM32F429_439xx || STM32F446xx || STM32F469_479xx */

#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define  RCC_AHB3ENR_QSPIEN                 (0x00000002UL)
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx || STM32F469_479xx */

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define  RCC_APB1ENR_TIM2EN                  (0x00000001UL)
#define  RCC_APB1ENR_TIM3EN                  (0x00000002UL)
#define  RCC_APB1ENR_TIM4EN                  (0x00000004UL)
#define  RCC_APB1ENR_TIM5EN                  (0x00000008UL)
#define  RCC_APB1ENR_TIM6EN                  (0x00000010UL)
#define  RCC_APB1ENR_TIM7EN                  (0x00000020UL)
#define  RCC_APB1ENR_TIM12EN                 (0x00000040UL)
#define  RCC_APB1ENR_TIM13EN                 (0x00000080UL)
#define  RCC_APB1ENR_TIM14EN                 (0x00000100UL)
#if defined(STM32F410xx) || defined(STM32F413_423xx)
#define  RCC_APB1ENR_LPTIM1EN                (0x00000200UL)
#endif /* STM32F410xx || STM32F413_423xx */
#define  RCC_APB1ENR_WWDGEN                  (0x00000800UL)
#define  RCC_APB1ENR_SPI2EN                  (0x00004000UL)
#define  RCC_APB1ENR_SPI3EN                  (0x00008000UL)
#if defined(STM32F446xx)
#define  RCC_APB1ENR_SPDIFRXEN               (0x00010000UL)
#endif /* STM32F446xx */
#define  RCC_APB1ENR_USART2EN                (0x00020000UL)
#define  RCC_APB1ENR_USART3EN                (0x00040000UL)
#define  RCC_APB1ENR_UART4EN                 (0x00080000UL)
#define  RCC_APB1ENR_UART5EN                 (0x00100000UL)
#define  RCC_APB1ENR_I2C1EN                  (0x00200000UL)
#define  RCC_APB1ENR_I2C2EN                  (0x00400000UL)
#define  RCC_APB1ENR_I2C3EN                  (0x00800000UL)
#if defined(STM32F410xx) || defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
#define  RCC_APB1ENR_FMPI2C1EN               (0x01000000UL)
#endif /* STM32F410xx || STM32F412xG || STM32F413_423xx || STM32F446xx */
#define  RCC_APB1ENR_CAN1EN                  (0x02000000UL)
#define  RCC_APB1ENR_CAN2EN                  (0x04000000UL)
#if defined(STM32F446xx)
#define  RCC_APB1ENR_CECEN                   (0x08000000UL)
#endif /* STM32F446xx */
#define  RCC_APB1ENR_PWREN                   (0x10000000UL)
#define  RCC_APB1ENR_DACEN                   (0x20000000UL)
#define  RCC_APB1ENR_UART7EN                 (0x40000000UL)
#define  RCC_APB1ENR_UART8EN                 (0x80000000UL)

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define  RCC_APB2ENR_TIM1EN                  (0x00000001UL)
#define  RCC_APB2ENR_TIM8EN                  (0x00000002UL)
#define  RCC_APB2ENR_USART1EN                (0x00000010UL)
#define  RCC_APB2ENR_USART6EN                (0x00000020UL)
#define  RCC_APB2ENR_UART9EN                 (0x00000040UL)
#define  RCC_APB2ENR_UART10EN                (0x00000080UL)
#define  RCC_APB2ENR_ADC1EN                  (0x00000100UL)
#define  RCC_APB2ENR_ADC2EN                  (0x00000200UL)
#define  RCC_APB2ENR_ADC3EN                  (0x00000400UL)
#define  RCC_APB2ENR_SDIOEN                  (0x00000800UL)
#define  RCC_APB2ENR_SPI1EN                  (0x00001000UL)
#define  RCC_APB2ENR_SPI4EN                  (0x00002000UL)
#define  RCC_APB2ENR_SYSCFGEN                (0x00004000UL)
#define  RCC_APB2ENR_EXTIEN                  (0x00008000UL)
#define  RCC_APB2ENR_TIM9EN                  (0x00010000UL)
#define  RCC_APB2ENR_TIM10EN                 (0x00020000UL)
#define  RCC_APB2ENR_TIM11EN                 (0x00040000UL)
#define  RCC_APB2ENR_SPI5EN                  (0x00100000UL)
#define  RCC_APB2ENR_SPI6EN                  (0x00200000UL)
#define  RCC_APB2ENR_SAI1EN                  (0x00400000UL)
#if defined(STM32F446xx)
#define  RCC_APB2ENR_SAI2EN                  (0x00800000UL)
#endif /* STM32F446xx */
#define  RCC_APB2ENR_LTDCEN                  (0x04000000UL)
#if defined(STM32F469_479xx)
#define  RCC_APB2ENR_DSIEN                   (0x08000000UL)
#endif /* STM32F469_479xx */
#if defined(STM32F412xG) || defined(STM32F413_423xx)
#define  RCC_APB2ENR_DFSDM1EN                (0x01000000UL)
#endif /* STM32F412xG || STM32F413_423xx */
#if defined(STM32F413_423xx)
#define  RCC_APB2ENR_DFSDM2EN                (0x02000000UL)
#endif /* STM32F413_423xx */
/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define  RCC_AHB1LPENR_GPIOALPEN             (0x00000001UL)
#define  RCC_AHB1LPENR_GPIOBLPEN             (0x00000002UL)
#define  RCC_AHB1LPENR_GPIOCLPEN             (0x00000004UL)
#define  RCC_AHB1LPENR_GPIODLPEN             (0x00000008UL)
#define  RCC_AHB1LPENR_GPIOELPEN             (0x00000010UL)
#define  RCC_AHB1LPENR_GPIOFLPEN             (0x00000020UL)
#define  RCC_AHB1LPENR_GPIOGLPEN             (0x00000040UL)
#define  RCC_AHB1LPENR_GPIOHLPEN             (0x00000080UL)
#define  RCC_AHB1LPENR_GPIOILPEN             (0x00000100UL)
#define  RCC_AHB1LPENR_GPIOJLPEN             (0x00000200UL)
#define  RCC_AHB1LPENR_GPIOKLPEN             (0x00000400UL)
#define  RCC_AHB1LPENR_CRCLPEN               (0x00001000UL)
#define  RCC_AHB1LPENR_FLITFLPEN             (0x00008000UL)
#define  RCC_AHB1LPENR_SRAM1LPEN             (0x00010000UL)
#define  RCC_AHB1LPENR_SRAM2LPEN             (0x00020000UL)
#define  RCC_AHB1LPENR_BKPSRAMLPEN           (0x00040000UL)
#define  RCC_AHB1LPENR_SRAM3LPEN             (0x00080000UL)
#define  RCC_AHB1LPENR_DMA1LPEN              (0x00200000UL)
#define  RCC_AHB1LPENR_DMA2LPEN              (0x00400000UL)
#define  RCC_AHB1LPENR_DMA2DLPEN             (0x00800000UL)
#define  RCC_AHB1LPENR_ETHMACLPEN            (0x02000000UL)
#define  RCC_AHB1LPENR_ETHMACTXLPEN          (0x04000000UL)
#define  RCC_AHB1LPENR_ETHMACRXLPEN          (0x08000000UL)
#define  RCC_AHB1LPENR_ETHMACPTPLPEN         (0x10000000UL)
#define  RCC_AHB1LPENR_OTGHSLPEN             (0x20000000UL)
#define  RCC_AHB1LPENR_OTGHSULPILPEN         (0x40000000UL)

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define  RCC_AHB2LPENR_DCMILPEN              (0x00000001UL)
#define  RCC_AHB2LPENR_CRYPLPEN              (0x00000010UL)
#define  RCC_AHB2LPENR_HASHLPEN              (0x00000020UL)
#define  RCC_AHB2LPENR_RNGLPEN               (0x00000040UL)
#define  RCC_AHB2LPENR_OTGFSLPEN             (0x00000080UL)

/********************  Bit definition for RCC_AHB3LPENR register  *************/
#if defined(STM32F40_41xxx) || defined(STM32F412xG) || defined(STM32F413_423xx)
#define  RCC_AHB3LPENR_FSMCLPEN              (0x00000001UL)
#endif /* STM32F40_41xxx || STM32F412xG || STM32F413_423xx */

#if defined(STM32F427_437xx) || defined(STM32F429_439xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define  RCC_AHB3LPENR_FMCLPEN              (0x00000001UL)
#endif /* STM32F427_437xx ||  STM32F429_439xx  || STM32F446xx || STM32F469_479xx */
#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx) || defined(STM32F469_479xx)
#define  RCC_AHB3LPENR_QSPILPEN             (0x00000002UL)
#endif /* STM32F412xG || STM32F413_423xx || STM32F469_479xx || STM32F446xx */

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define  RCC_APB1LPENR_TIM2LPEN              (0x00000001UL)
#define  RCC_APB1LPENR_TIM3LPEN              (0x00000002UL)
#define  RCC_APB1LPENR_TIM4LPEN              (0x00000004UL)
#define  RCC_APB1LPENR_TIM5LPEN              (0x00000008UL)
#define  RCC_APB1LPENR_TIM6LPEN              (0x00000010UL)
#define  RCC_APB1LPENR_TIM7LPEN              (0x00000020UL)
#define  RCC_APB1LPENR_TIM12LPEN             (0x00000040UL)
#define  RCC_APB1LPENR_TIM13LPEN             (0x00000080UL)
#define  RCC_APB1LPENR_TIM14LPEN             (0x00000100UL)
#if defined(STM32F410xx) || defined(STM32F413_423xx)
#define  RCC_APB1LPENR_LPTIM1LPEN            (0x00000200UL)
#endif /* STM32F410xx || STM32F413_423xx */
#define  RCC_APB1LPENR_WWDGLPEN              (0x00000800UL)
#define  RCC_APB1LPENR_SPI2LPEN              (0x00004000UL)
#define  RCC_APB1LPENR_SPI3LPEN              (0x00008000UL)
#if defined(STM32F446xx)
#define  RCC_APB1LPENR_SPDIFRXLPEN           (0x00010000UL)
#endif /* STM32F446xx */
#define  RCC_APB1LPENR_USART2LPEN            (0x00020000UL)
#define  RCC_APB1LPENR_USART3LPEN            (0x00040000UL)
#define  RCC_APB1LPENR_UART4LPEN             (0x00080000UL)
#define  RCC_APB1LPENR_UART5LPEN             (0x00100000UL)
#define  RCC_APB1LPENR_I2C1LPEN              (0x00200000UL)
#define  RCC_APB1LPENR_I2C2LPEN              (0x00400000UL)
#define  RCC_APB1LPENR_I2C3LPEN              (0x00800000UL)
#if defined(STM32F410xx) || defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
#define  RCC_APB1LPENR_FMPI2C1LPEN           (0x01000000UL)
#endif /* STM32F410xx || STM32F412xG || STM32F413_423xx || STM32F446xx */
#define  RCC_APB1LPENR_CAN1LPEN              (0x02000000UL)
#define  RCC_APB1LPENR_CAN2LPEN              (0x04000000UL)
#if defined(STM32F446xx)
#define  RCC_APB1LPENR_CECLPEN               (0x08000000UL)
#endif /* STM32F446xx */
#define  RCC_APB1LPENR_PWRLPEN               (0x10000000UL)
#define  RCC_APB1LPENR_DACLPEN               (0x20000000UL)
#define  RCC_APB1LPENR_UART7LPEN             (0x40000000UL)
#define  RCC_APB1LPENR_UART8LPEN             (0x80000000UL)

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define  RCC_APB2LPENR_TIM1LPEN              (0x00000001UL)
#define  RCC_APB2LPENR_TIM8LPEN              (0x00000002UL)
#define  RCC_APB2LPENR_USART1LPEN            (0x00000010UL)
#define  RCC_APB2LPENR_USART6LPEN            (0x00000020UL)
#define  RCC_APB2LPENR_UART9LPEN             (0x00000040UL)
#define  RCC_APB2LPENR_UART10LPEN            (0x00000080UL)
#define  RCC_APB2LPENR_ADC1LPEN              (0x00000100UL)
#define  RCC_APB2LPENR_ADC2PEN               (0x00000200UL)
#define  RCC_APB2LPENR_ADC3LPEN              (0x00000400UL)
#define  RCC_APB2LPENR_SDIOLPEN              (0x00000800UL)
#define  RCC_APB2LPENR_SPI1LPEN              (0x00001000UL)
#define  RCC_APB2LPENR_SPI4LPEN              (0x00002000UL)
#define  RCC_APB2LPENR_SYSCFGLPEN            (0x00004000UL)
#define  RCC_APB2LPENR_TIM9LPEN              (0x00010000UL)
#define  RCC_APB2LPENR_TIM10LPEN             (0x00020000UL)
#define  RCC_APB2LPENR_TIM11LPEN             (0x00040000UL)
#define  RCC_APB2LPENR_SPI5LPEN              (0x00100000UL)
#define  RCC_APB2LPENR_SPI6LPEN              (0x00200000UL)
#define  RCC_APB2LPENR_SAI1LPEN              (0x00400000UL)
#if defined(STM32F446xx)
#define  RCC_APB2LPENR_SAI2LPEN              (0x00800000UL)
#endif /* STM32F446xx */
#define  RCC_APB2LPENR_LTDCLPEN              (0x04000000UL)
#if defined(STM32F469_479xx)
#define  RCC_APB2LPENR_DSILPEN               (0x08000000UL)
#endif /* STM32F469_479xx */
#if defined(STM32F412xG) || defined(STM32F413_423xx)
#define  RCC_APB2LPENR_DFSDM1LPEN            (0x01000000UL)
#endif /* STM32F412xG || STM32F413_423xx */
#if defined(STM32F413_423xx)
#define  RCC_APB2LPENR_DFSDM2LPEN            (0x02000000UL)
#endif /* STM32F413_423xx */

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

#if defined(STM32F412xG) || defined(STM32F413_423xx)
#define  RCC_PLLI2SCFGR_PLLI2SSRC            (0x00400000UL)
#endif /* STM32F412xG || STM32F413_423xx */

#if defined(STM32F446xx)
#define  RCC_PLLI2SCFGR_PLLI2SP              (0x00030000UL)
#define  RCC_PLLI2SCFGR_PLLI2SP_0            (0x00010000UL)
#define  RCC_PLLI2SCFGR_PLLI2SP_1            (0x00020000UL)
#endif /* STM32F446xx */

#define  RCC_PLLI2SCFGR_PLLI2SQ              (0x0F000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SQ_0            (0x01000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SQ_1            (0x02000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SQ_2            (0x04000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SQ_3            (0x08000000UL)

#define  RCC_PLLI2SCFGR_PLLI2SR              (0x70000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SR_0            (0x10000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SR_1            (0x20000000UL)
#define  RCC_PLLI2SCFGR_PLLI2SR_2            (0x40000000UL)

/********************  Bit definition for RCC_PLLSAICFGR register  ************/
#if defined(STM32F446xx)
#define  RCC_PLLSAICFGR_PLLSAIM              (0x0000003FUL)
#define  RCC_PLLSAICFGR_PLLSAIM_0            (0x00000001UL)
#define  RCC_PLLSAICFGR_PLLSAIM_1            (0x00000002UL)
#define  RCC_PLLSAICFGR_PLLSAIM_2            (0x00000004UL)
#define  RCC_PLLSAICFGR_PLLSAIM_3            (0x00000008UL)
#define  RCC_PLLSAICFGR_PLLSAIM_4            (0x00000010UL)
#define  RCC_PLLSAICFGR_PLLSAIM_5            (0x00000020UL)
#endif /* STM32F446xx */

#define  RCC_PLLSAICFGR_PLLSAIN              (0x00007FC0UL)
#define  RCC_PLLSAICFGR_PLLSAIN_0            (0x00000040UL)
#define  RCC_PLLSAICFGR_PLLSAIN_1            (0x00000080UL)
#define  RCC_PLLSAICFGR_PLLSAIN_2            (0x00000100UL)
#define  RCC_PLLSAICFGR_PLLSAIN_3            (0x00000200UL)
#define  RCC_PLLSAICFGR_PLLSAIN_4            (0x00000400UL)
#define  RCC_PLLSAICFGR_PLLSAIN_5            (0x00000800UL)
#define  RCC_PLLSAICFGR_PLLSAIN_6            (0x00001000UL)
#define  RCC_PLLSAICFGR_PLLSAIN_7            (0x00002000UL)
#define  RCC_PLLSAICFGR_PLLSAIN_8            (0x00004000UL)

#if defined(STM32F446xx) || defined(STM32F469_479xx)
#define  RCC_PLLSAICFGR_PLLSAIP              (0x00030000UL)
#define  RCC_PLLSAICFGR_PLLSAIP_0            (0x00010000UL)
#define  RCC_PLLSAICFGR_PLLSAIP_1            (0x00020000UL)
#endif /* STM32F446xx || STM32F469_479xx */

#define  RCC_PLLSAICFGR_PLLSAIQ              (0x0F000000UL)
#define  RCC_PLLSAICFGR_PLLSAIQ_0            (0x01000000UL)
#define  RCC_PLLSAICFGR_PLLSAIQ_1            (0x02000000UL)
#define  RCC_PLLSAICFGR_PLLSAIQ_2            (0x04000000UL)
#define  RCC_PLLSAICFGR_PLLSAIQ_3            (0x08000000UL)

#define  RCC_PLLSAICFGR_PLLSAIR              (0x70000000UL)
#define  RCC_PLLSAICFGR_PLLSAIR_0            (0x10000000UL)
#define  RCC_PLLSAICFGR_PLLSAIR_1            (0x20000000UL)
#define  RCC_PLLSAICFGR_PLLSAIR_2            (0x40000000UL)

/********************  Bit definition for RCC_DCKCFGR register  ***************/
#define  RCC_DCKCFGR_PLLI2SDIVQ              (0x0000001FUL)
#define  RCC_DCKCFGR_PLLSAIDIVQ              (0x00001F00UL)
#define  RCC_DCKCFGR_PLLSAIDIVR              (0x00030000UL)

#if defined(STM32F412xG) || defined(STM32F413_423xx)
#define  RCC_DCKCFGR_CKDFSDM1SEL             (0x80000000UL)
#define  RCC_DCKCFGR_CKDFSDM1ASEL            (0x00008000UL)
#endif /* STM32F412xG || STM32F413_423xx */

#if defined(STM32F413_423xx)
#define  RCC_DCKCFGR_PLLI2SDIVR              (0x0000001FUL)
#define  RCC_DCKCFGR_PLLI2SDIVR_0            (0x00000001UL)
#define  RCC_DCKCFGR_PLLI2SDIVR_1            (0x00000002UL)
#define  RCC_DCKCFGR_PLLI2SDIVR_2            (0x00000004UL)
#define  RCC_DCKCFGR_PLLI2SDIVR_3            (0x00000008UL)
#define  RCC_DCKCFGR_PLLI2SDIVR_4            (0x00000010UL)

#define  RCC_DCKCFGR_PLLDIVR                 (0x00001F00UL)
#define  RCC_DCKCFGR_PLLDIVR_0               (0x00000100UL)
#define  RCC_DCKCFGR_PLLDIVR_1               (0x00000200UL)
#define  RCC_DCKCFGR_PLLDIVR_2               (0x00000400UL)
#define  RCC_DCKCFGR_PLLDIVR_3               (0x00000800UL)
#define  RCC_DCKCFGR_PLLDIVR_4               (0x00001000UL)
#define  RCC_DCKCFGR_CKDFSDM2ASEL            (0x00004000UL)
#endif /* STM32F413_423xx */

#define  RCC_DCKCFGR_SAI1ASRC                (0x00300000UL)
#define  RCC_DCKCFGR_SAI1ASRC_0              (0x00100000UL)
#define  RCC_DCKCFGR_SAI1ASRC_1              (0x00200000UL)
#if defined(STM32F446xx)
#define  RCC_DCKCFGR_SAI1SRC                 (0x00300000UL)
#define  RCC_DCKCFGR_SAI1SRC_0               (0x00100000UL)
#define  RCC_DCKCFGR_SAI1SRC_1               (0x00200000UL)
#endif /* STM32F446xx */

#define  RCC_DCKCFGR_SAI1BSRC                (0x00C00000UL)
#define  RCC_DCKCFGR_SAI1BSRC_0              (0x00400000UL)
#define  RCC_DCKCFGR_SAI1BSRC_1              (0x00800000UL)
#if defined(STM32F446xx)
#define  RCC_DCKCFGR_SAI2SRC                 (0x00C00000UL)
#define  RCC_DCKCFGR_SAI2SRC_0               (0x00400000UL)
#define  RCC_DCKCFGR_SAI2SRC_1               (0x00800000UL)
#endif /* STM32F446xx */

#define  RCC_DCKCFGR_TIMPRE                  (0x01000000UL)
#if defined(STM32F469_479xx)
#define  RCC_DCKCFGR_CK48MSEL                (0x08000000UL)
#define  RCC_DCKCFGR_SDIOSEL                 (0x10000000UL)
#define  RCC_DCKCFGR_DSISEL                  (0x20000000UL)
#endif /* STM32F469_479xx */

#if defined(STM32F412xG) || defined(STM32F413_423xx) || defined(STM32F446xx)
#define  RCC_DCKCFGR_I2S1SRC                 (0x06000000UL)
#define  RCC_DCKCFGR_I2S1SRC_0               (0x02000000UL)
#define  RCC_DCKCFGR_I2S1SRC_1               (0x04000000UL)
#define  RCC_DCKCFGR_I2S2SRC                 (0x18000000UL)
#define  RCC_DCKCFGR_I2S2SRC_0               (0x08000000UL)
#define  RCC_DCKCFGR_I2S2SRC_1               (0x10000000UL)

/********************  Bit definition for RCC_CKGATENR register  ***************/
#define  RCC_CKGATENR_AHB2APB1_CKEN          (0x00000001UL)
#define  RCC_CKGATENR_AHB2APB2_CKEN          (0x00000002UL)
#define  RCC_CKGATENR_CM4DBG_CKEN            (0x00000004UL)
#define  RCC_CKGATENR_SPARE_CKEN             (0x00000008UL)
#define  RCC_CKGATENR_SRAM_CKEN              (0x00000010UL)
#define  RCC_CKGATENR_FLITF_CKEN             (0x00000020UL)
#define  RCC_CKGATENR_RCC_CKEN               (0x00000040UL)
#if defined(STM32F412xG) || defined(STM32F413_423xx)
#define  RCC_CKGATENR_RCC_EVTCTL             (0x00000080UL)
#endif /* STM32F412xG || STM32F413_423xx */

/********************  Bit definition for RCC_DCKCFGR2 register  ***************/
#define  RCC_DCKCFGR2_FMPI2C1SEL             (0x00C00000UL)
#define  RCC_DCKCFGR2_FMPI2C1SEL_0           (0x00400000UL)
#define  RCC_DCKCFGR2_FMPI2C1SEL_1           (0x00800000UL)
#define  RCC_DCKCFGR2_CECSEL                 (0x04000000UL)
#define  RCC_DCKCFGR2_CK48MSEL               (0x08000000UL)
#define  RCC_DCKCFGR2_SDIOSEL                (0x10000000UL)
#if defined(STM32F446xx)
#define  RCC_DCKCFGR2_SPDIFRXSEL             (0x20000000UL)
#endif /* STM32F446xx */
#if defined(STM32F413_423xx)
#define  RCC_DCKCFGR2_LPTIM1SEL              (0xC0000000UL)
#define  RCC_DCKCFGR2_LPTIM1SEL_0            (0x40000000UL)
#define  RCC_DCKCFGR2_LPTIM1SEL_1            (0x80000000UL)
#endif /* STM32F413_423xx */
#endif /* STM32F412xG || STM32F413_423xx || STM32F446xx */

#if defined(STM32F410xx)
#define  RCC_DCKCFGR_I2SSRC                  (0x06000000UL)
#define  RCC_DCKCFGR_I2SSRC_0                (0x02000000UL)
#define  RCC_DCKCFGR_I2SSRC_1                (0x04000000UL)
#endif /* STM32F410xx */

#if defined(STM32F410xx)
/********************  Bit definition for RCC_DCKCFGR2 register  **************/
#define  RCC_DCKCFGR2_FMPI2C1SEL             (0x00C00000UL)
#define  RCC_DCKCFGR2_FMPI2C1SEL_0           (0x00400000UL)
#define  RCC_DCKCFGR2_FMPI2C1SEL_1           (0x00800000UL)
#define  RCC_DCKCFGR2_LPTIM1SEL              (0xC0000000UL)
#define  RCC_DCKCFGR2_LPTIM1SEL_0            (0x40000000UL)
#define  RCC_DCKCFGR2_LPTIM1SEL_1            (0x80000000UL)
#endif /* STM32F410xx */
/******************************************************************************/
/*                                                                            */
/*                                    RNG                                     */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RNG_CR register  *******************/
#define RNG_CR_RNGEN                         (0x00000004UL)
#define RNG_CR_IE                            (0x00000008UL)

/********************  Bits definition for RNG_SR register  *******************/
#define RNG_SR_DRDY                          (0x00000001UL)
#define RNG_SR_CECS                          (0x00000002UL)
#define RNG_SR_SECS                          (0x00000004UL)
#define RNG_SR_CEIS                          (0x00000020UL)
#define RNG_SR_SEIS                          (0x00000040UL)

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
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
#define RTC_PRER_PREDIV_S                    (0x00001FFFUL)

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
/*                          Serial Audio Interface                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for SAI_GCR register  *******************/
#define  SAI_GCR_SYNCIN                  (0x00000003UL)        /*!<SYNCIN[1:0] bits (Synchronization Inputs)   */
#define  SAI_GCR_SYNCIN_0                (0x00000001UL)        /*!<Bit 0 */
#define  SAI_GCR_SYNCIN_1                (0x00000002UL)        /*!<Bit 1 */

#define  SAI_GCR_SYNCOUT                 (0x00000030UL)        /*!<SYNCOUT[1:0] bits (Synchronization Outputs) */
#define  SAI_GCR_SYNCOUT_0               (0x00000010UL)        /*!<Bit 0 */
#define  SAI_GCR_SYNCOUT_1               (0x00000020UL)        /*!<Bit 1 */

/*******************  Bit definition for SAI_xCR1 register  *******************/
#define  SAI_xCR1_MODE                    (0x00000003UL)        /*!<MODE[1:0] bits (Audio Block Mode)           */
#define  SAI_xCR1_MODE_0                  (0x00000001UL)        /*!<Bit 0 */
#define  SAI_xCR1_MODE_1                  (0x00000002UL)        /*!<Bit 1 */

#define  SAI_xCR1_PRTCFG                  (0x0000000CUL)        /*!<PRTCFG[1:0] bits (Protocol Configuration)   */
#define  SAI_xCR1_PRTCFG_0                (0x00000004UL)        /*!<Bit 0 */
#define  SAI_xCR1_PRTCFG_1                (0x00000008UL)        /*!<Bit 1 */

#define  SAI_xCR1_DS                      (0x000000E0UL)        /*!<DS[1:0] bits (Data Size) */
#define  SAI_xCR1_DS_0                    (0x00000020UL)        /*!<Bit 0 */
#define  SAI_xCR1_DS_1                    (0x00000040UL)        /*!<Bit 1 */
#define  SAI_xCR1_DS_2                    (0x00000080UL)        /*!<Bit 2 */

#define  SAI_xCR1_LSBFIRST                (0x00000100UL)        /*!<LSB First Configuration  */
#define  SAI_xCR1_CKSTR                   (0x00000200UL)        /*!<ClocK STRobing edge      */

#define  SAI_xCR1_SYNCEN                  (0x00000C00UL)        /*!<SYNCEN[1:0](SYNChronization ENable) */
#define  SAI_xCR1_SYNCEN_0                (0x00000400UL)        /*!<Bit 0 */
#define  SAI_xCR1_SYNCEN_1                (0x00000800UL)        /*!<Bit 1 */

#define  SAI_xCR1_MONO                    (0x00001000UL)        /*!<Mono mode                  */
#define  SAI_xCR1_OUTDRIV                 (0x00002000UL)        /*!<Output Drive               */
#define  SAI_xCR1_SAIEN                   (0x00010000UL)        /*!<Audio Block enable         */
#define  SAI_xCR1_DMAEN                   (0x00020000UL)        /*!<DMA enable                 */
#define  SAI_xCR1_NODIV                   (0x00080000UL)        /*!<No Divider Configuration   */

#define  SAI_xCR1_MCKDIV                  (0x00780000UL)        /*!<MCKDIV[3:0] (Master ClocK Divider)  */
#define  SAI_xCR1_MCKDIV_0                (0x00080000UL)        /*!<Bit 0  */
#define  SAI_xCR1_MCKDIV_1                (0x00100000UL)        /*!<Bit 1  */
#define  SAI_xCR1_MCKDIV_2                (0x00200000UL)        /*!<Bit 2  */
#define  SAI_xCR1_MCKDIV_3                (0x00400000UL)        /*!<Bit 3  */

/*******************  Bit definition for SAI_xCR2 register  *******************/
#define  SAI_xCR2_FTH                     (0x00000003UL)        /*!<FTH[1:0](Fifo THreshold)  */
#define  SAI_xCR2_FTH_0                   (0x00000001UL)        /*!<Bit 0 */
#define  SAI_xCR2_FTH_1                   (0x00000002UL)        /*!<Bit 1 */

#define  SAI_xCR2_FFLUSH                  (0x00000008UL)        /*!<Fifo FLUSH                       */
#define  SAI_xCR2_TRIS                    (0x00000010UL)        /*!<TRIState Management on data line */
#define  SAI_xCR2_MUTE                    (0x00000020UL)        /*!<Mute mode                        */
#define  SAI_xCR2_MUTEVAL                 (0x00000040UL)        /*!<Muate value                      */

#define  SAI_xCR2_MUTECNT                  (0x00001F80UL)       /*!<MUTECNT[5:0] (MUTE counter) */
#define  SAI_xCR2_MUTECNT_0               (0x00000080UL)        /*!<Bit 0 */
#define  SAI_xCR2_MUTECNT_1               (0x00000100UL)        /*!<Bit 1 */
#define  SAI_xCR2_MUTECNT_2               (0x00000200UL)        /*!<Bit 2 */
#define  SAI_xCR2_MUTECNT_3               (0x00000400UL)        /*!<Bit 3 */
#define  SAI_xCR2_MUTECNT_4               (0x00000800UL)        /*!<Bit 4 */
#define  SAI_xCR2_MUTECNT_5               (0x00001000UL)        /*!<Bit 5 */

#define  SAI_xCR2_CPL                     (0x00002000UL)        /*!< Complement Bit             */

#define  SAI_xCR2_COMP                    (0x0000C000UL)        /*!<COMP[1:0] (Companding mode) */
#define  SAI_xCR2_COMP_0                  (0x00004000UL)        /*!<Bit 0 */
#define  SAI_xCR2_COMP_1                  (0x00008000UL)        /*!<Bit 1 */

/******************  Bit definition for SAI_xFRCR register  *******************/
#define  SAI_xFRCR_FRL                    (0x000000FFUL)        /*!<FRL[1:0](Frame length)  */
#define  SAI_xFRCR_FRL_0                  (0x00000001UL)        /*!<Bit 0 */
#define  SAI_xFRCR_FRL_1                  (0x00000002UL)        /*!<Bit 1 */
#define  SAI_xFRCR_FRL_2                  (0x00000004UL)        /*!<Bit 2 */
#define  SAI_xFRCR_FRL_3                  (0x00000008UL)        /*!<Bit 3 */
#define  SAI_xFRCR_FRL_4                  (0x00000010UL)        /*!<Bit 4 */
#define  SAI_xFRCR_FRL_5                  (0x00000020UL)        /*!<Bit 5 */
#define  SAI_xFRCR_FRL_6                  (0x00000040UL)        /*!<Bit 6 */
#define  SAI_xFRCR_FRL_7                  (0x00000080UL)        /*!<Bit 7 */

#define  SAI_xFRCR_FSALL                  (0x00007F00UL)        /*!<FRL[1:0] (Frame synchronization active level length)  */
#define  SAI_xFRCR_FSALL_0                (0x00000100UL)        /*!<Bit 0 */
#define  SAI_xFRCR_FSALL_1                (0x00000200UL)        /*!<Bit 1 */
#define  SAI_xFRCR_FSALL_2                (0x00000400UL)        /*!<Bit 2 */
#define  SAI_xFRCR_FSALL_3                (0x00000800UL)        /*!<Bit 3 */
#define  SAI_xFRCR_FSALL_4                (0x00001000UL)        /*!<Bit 4 */
#define  SAI_xFRCR_FSALL_5                (0x00002000UL)        /*!<Bit 5 */
#define  SAI_xFRCR_FSALL_6                (0x00004000UL)        /*!<Bit 6 */

#define  SAI_xFRCR_FSDEF                  (0x00010000UL)        /*!< Frame Synchronization Definition */
#define  SAI_xFRCR_FSPOL                  (0x00020000UL)        /*!<Frame Synchronization POLarity    */
#define  SAI_xFRCR_FSOFF                  (0x00040000UL)        /*!<Frame Synchronization OFFset      */
/* Legacy defines */
#define  SAI_xFRCR_FSPO                   SAI_xFRCR_FSPOL

/******************  Bit definition for SAI_xSLOTR register  *******************/
#define  SAI_xSLOTR_FBOFF                 (0x0000001FUL)        /*!<FRL[4:0](First Bit Offset)  */
#define  SAI_xSLOTR_FBOFF_0               (0x00000001UL)        /*!<Bit 0 */
#define  SAI_xSLOTR_FBOFF_1               (0x00000002UL)        /*!<Bit 1 */
#define  SAI_xSLOTR_FBOFF_2               (0x00000004UL)        /*!<Bit 2 */
#define  SAI_xSLOTR_FBOFF_3               (0x00000008UL)        /*!<Bit 3 */
#define  SAI_xSLOTR_FBOFF_4               (0x00000010UL)        /*!<Bit 4 */

#define  SAI_xSLOTR_SLOTSZ                (0x000000C0UL)        /*!<SLOTSZ[1:0] (Slot size)  */
#define  SAI_xSLOTR_SLOTSZ_0              (0x00000040UL)        /*!<Bit 0 */
#define  SAI_xSLOTR_SLOTSZ_1              (0x00000080UL)        /*!<Bit 1 */

#define  SAI_xSLOTR_NBSLOT                (0x00000F00UL)        /*!<NBSLOT[3:0] (Number of Slot in audio Frame)  */
#define  SAI_xSLOTR_NBSLOT_0              (0x00000100UL)        /*!<Bit 0 */
#define  SAI_xSLOTR_NBSLOT_1              (0x00000200UL)        /*!<Bit 1 */
#define  SAI_xSLOTR_NBSLOT_2              (0x00000400UL)        /*!<Bit 2 */
#define  SAI_xSLOTR_NBSLOT_3              (0x00000800UL)        /*!<Bit 3 */

#define  SAI_xSLOTR_SLOTEN                (0xFFFF0000UL)        /*!<SLOTEN[15:0] (Slot Enable)  */

/*******************  Bit definition for SAI_xIMR register  *******************/
#define  SAI_xIMR_OVRUDRIE                (0x00000001UL)        /*!<Overrun underrun interrupt enable                              */
#define  SAI_xIMR_MUTEDETIE               (0x00000002UL)        /*!<Mute detection interrupt enable                                */
#define  SAI_xIMR_WCKCFGIE                (0x00000004UL)        /*!<Wrong Clock Configuration interrupt enable                     */
#define  SAI_xIMR_FREQIE                  (0x00000008UL)        /*!<FIFO request interrupt enable                                  */
#define  SAI_xIMR_CNRDYIE                 (0x00000010UL)        /*!<Codec not ready interrupt enable                               */
#define  SAI_xIMR_AFSDETIE                (0x00000020UL)        /*!<Anticipated frame synchronization detection interrupt enable   */
#define  SAI_xIMR_LFSDETIE                (0x00000040UL)        /*!<Late frame synchronization detection interrupt enable          */

/********************  Bit definition for SAI_xSR register  *******************/
#define  SAI_xSR_OVRUDR                   (0x00000001UL)         /*!<Overrun underrun                               */
#define  SAI_xSR_MUTEDET                  (0x00000002UL)         /*!<Mute detection                                 */
#define  SAI_xSR_WCKCFG                   (0x00000004UL)         /*!<Wrong Clock Configuration                      */
#define  SAI_xSR_FREQ                     (0x00000008UL)         /*!<FIFO request                                   */
#define  SAI_xSR_CNRDY                    (0x00000010UL)         /*!<Codec not ready                                */
#define  SAI_xSR_AFSDET                   (0x00000020UL)         /*!<Anticipated frame synchronization detection    */
#define  SAI_xSR_LFSDET                   (0x00000040UL)         /*!<Late frame synchronization detection           */

#define  SAI_xSR_FLVL                     (0x00070000UL)         /*!<FLVL[2:0] (FIFO Level Threshold)               */
#define  SAI_xSR_FLVL_0                   (0x00010000UL)         /*!<Bit 0 */
#define  SAI_xSR_FLVL_1                   (0x00020000UL)         /*!<Bit 1 */
#define  SAI_xSR_FLVL_2                   (0x00030000UL)         /*!<Bit 2 */

/******************  Bit definition for SAI_xCLRFR register  ******************/
#define  SAI_xCLRFR_COVRUDR               (0x00000001UL)        /*!<Clear Overrun underrun                               */
#define  SAI_xCLRFR_CMUTEDET              (0x00000002UL)        /*!<Clear Mute detection                                 */
#define  SAI_xCLRFR_CWCKCFG               (0x00000004UL)        /*!<Clear Wrong Clock Configuration                      */
#define  SAI_xCLRFR_CFREQ                 (0x00000008UL)        /*!<Clear FIFO request                                   */
#define  SAI_xCLRFR_CCNRDY                (0x00000010UL)        /*!<Clear Codec not ready                                */
#define  SAI_xCLRFR_CAFSDET               (0x00000020UL)        /*!<Clear Anticipated frame synchronization detection    */
#define  SAI_xCLRFR_CLFSDET               (0x00000040UL)        /*!<Clear Late frame synchronization detection           */

/******************  Bit definition for SAI_xDR register  ******************/
#define  SAI_xDR_DATA                     (0xFFFFFFFFUL)

#if defined(STM32F446xx)
/******************************************************************************/
/*                                                                            */
/*                              SPDIF-RX Interface                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for SPDIFRX_CR register  *******************/
#define  SPDIFRX_CR_SPDIFEN                  (0x00000003UL)        /*!<Peripheral Block Enable                      */
#define  SPDIFRX_CR_RXDMAEN                  (0x00000004UL)        /*!<Receiver DMA Enable for data flow            */
#define  SPDIFRX_CR_RXSTEO                   (0x00000008UL)        /*!<Stereo Mode                                  */
#define  SPDIFRX_CR_DRFMT                    (0x00000030UL)        /*!<RX Data format                               */
#define  SPDIFRX_CR_PMSK                     (0x00000040UL)        /*!<Mask Parity error bit                        */
#define  SPDIFRX_CR_VMSK                     (0x00000080UL)        /*!<Mask of Validity bit                         */
#define  SPDIFRX_CR_CUMSK                    (0x00000100UL)        /*!<Mask of channel status and user bits         */
#define  SPDIFRX_CR_PTMSK                    (0x00000200UL)        /*!<Mask of Preamble Type bits                   */
#define  SPDIFRX_CR_CBDMAEN                  (0x00000400UL)        /*!<Control Buffer DMA ENable for control flow   */
#define  SPDIFRX_CR_CHSEL                    (0x00000800UL)        /*!<Channel Selection                            */
#define  SPDIFRX_CR_NBTR                     (0x00003000UL)        /*!<Maximum allowed re-tries during synchronization phase */
#define  SPDIFRX_CR_WFA                      (0x00004000UL)        /*!<Wait For Activity     */
#define  SPDIFRX_CR_INSEL                    (0x00070000UL)        /*!<SPDIFRX input selection */

/*******************  Bit definition for SPDIFRX_IMR register  *******************/
#define  SPDIFRX_IMR_RXNEIE                   (0x00000001UL)        /*!<RXNE interrupt enable                              */
#define  SPDIFRX_IMR_CSRNEIE                  (0x00000002UL)        /*!<Control Buffer Ready Interrupt Enable              */
#define  SPDIFRX_IMR_PERRIE                   (0x00000004UL)        /*!<Parity error interrupt enable                      */
#define  SPDIFRX_IMR_OVRIE                    (0x00000008UL)        /*!<Overrun error Interrupt Enable                     */
#define  SPDIFRX_IMR_SBLKIE                   (0x00000010UL)        /*!<Synchronization Block Detected Interrupt Enable    */
#define  SPDIFRX_IMR_SYNCDIE                  (0x00000020UL)        /*!<Synchronization Done                               */
#define  SPDIFRX_IMR_IFEIE                    (0x00000040UL)        /*!<Serial Interface Error Interrupt Enable            */

/*******************  Bit definition for SPDIFRX_SR register  *******************/
#define  SPDIFRX_SR_RXNE                   (0x00000001UL)       /*!<Read data register not empty                          */
#define  SPDIFRX_SR_CSRNE                  (0x00000002UL)       /*!<The Control Buffer register is not empty              */
#define  SPDIFRX_SR_PERR                   (0x00000004UL)       /*!<Parity error                                          */
#define  SPDIFRX_SR_OVR                    (0x00000008UL)       /*!<Overrun error                                         */
#define  SPDIFRX_SR_SBD                    (0x00000010UL)       /*!<Synchronization Block Detected                        */
#define  SPDIFRX_SR_SYNCD                  (0x00000020UL)       /*!<Synchronization Done                                  */
#define  SPDIFRX_SR_FERR                   (0x00000040UL)       /*!<Framing error                                         */
#define  SPDIFRX_SR_SERR                   (0x00000080UL)       /*!<Synchronization error                                 */
#define  SPDIFRX_SR_TERR                   (0x00000100UL)       /*!<Time-out error                                        */
#define  SPDIFRX_SR_WIDTH5                 (0x7FFF0000UL)       /*!<Duration of 5 symbols counted with SPDIFRX_clk        */

/*******************  Bit definition for SPDIFRX_IFCR register  *******************/
#define  SPDIFRX_IFCR_PERRCF               (0x00000004UL)       /*!<Clears the Parity error flag                         */
#define  SPDIFRX_IFCR_OVRCF                (0x00000008UL)       /*!<Clears the Overrun error flag                        */
#define  SPDIFRX_IFCR_SBDCF                (0x00000010UL)       /*!<Clears the Synchronization Block Detected flag       */
#define  SPDIFRX_IFCR_SYNCDCF              (0x00000020UL)       /*!<Clears the Synchronization Done flag                 */

/*******************  Bit definition for SPDIFRX_DR register  (DRFMT = 0b00 case) *******************/
#define  SPDIFRX_DR0_DR                    (0x00FFFFFFUL)        /*!<Data value            */
#define  SPDIFRX_DR0_PE                    (0x01000000UL)        /*!<Parity Error bit      */
#define  SPDIFRX_DR0_V                     (0x02000000UL)        /*!<Validity bit          */
#define  SPDIFRX_DR0_U                     (0x04000000UL)        /*!<User bit              */
#define  SPDIFRX_DR0_C                     (0x08000000UL)        /*!<Channel Status bit    */
#define  SPDIFRX_DR0_PT                    (0x30000000UL)        /*!<Preamble Type         */

/*******************  Bit definition for SPDIFRX_DR register  (DRFMT = 0b01 case) *******************/
#define  SPDIFRX_DR1_DR                    (0xFFFFFF00UL)        /*!<Data value            */
#define  SPDIFRX_DR1_PT                    (0x00000030UL)        /*!<Preamble Type         */
#define  SPDIFRX_DR1_C                     (0x00000008UL)        /*!<Channel Status bit    */
#define  SPDIFRX_DR1_U                     (0x00000004UL)        /*!<User bit              */
#define  SPDIFRX_DR1_V                     (0x00000002UL)        /*!<Validity bit          */
#define  SPDIFRX_DR1_PE                    (0x00000001UL)        /*!<Parity Error bit      */

/*******************  Bit definition for SPDIFRX_DR register  (DRFMT = 0b10 case) *******************/
#define  SPDIFRX_DR1_DRNL1                 (0xFFFF0000UL)        /*!<Data value Channel B      */
#define  SPDIFRX_DR1_DRNL2                 (0x0000FFFFUL)        /*!<Data value Channel A      */

/*******************  Bit definition for SPDIFRX_CSR register   *******************/
#define  SPDIFRX_CSR_USR                     (0x0000FFFFUL)        /*!<User data information           */
#define  SPDIFRX_CSR_CS                      (0x00FF0000UL)        /*!<Channel A status information    */
#define  SPDIFRX_CSR_SOB                     (0x01000000UL)        /*!<Start Of Block                  */

/*******************  Bit definition for SPDIFRX_DIR register    *******************/
#define  SPDIFRX_DIR_THI                 (0x000013FFUL)        /*!<Threshold LOW      */
#define  SPDIFRX_DIR_TLO                 (0x1FFF0000UL)        /*!<Threshold HIGH     */
#endif /* STM32F446xx */

/******************************************************************************/
/*                                                                            */
/*                          SD host Interface                                 */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SDIO_POWER register  ******************/
#define  SDIO_POWER_PWRCTRL                  (0x03U)               /*!<PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDIO_POWER_PWRCTRL_0                (0x01U)               /*!<Bit 0 */
#define  SDIO_POWER_PWRCTRL_1                (0x02U)               /*!<Bit 1 */

/******************  Bit definition for SDIO_CLKCR register  ******************/
#define  SDIO_CLKCR_CLKDIV                   (0x00FFU)            /*!<Clock divide factor             */
#define  SDIO_CLKCR_CLKEN                    (0x0100U)            /*!<Clock enable bit                */
#define  SDIO_CLKCR_PWRSAV                   (0x0200U)            /*!<Power saving configuration bit  */
#define  SDIO_CLKCR_BYPASS                   (0x0400U)            /*!<Clock divider bypass enable bit */

#define  SDIO_CLKCR_WIDBUS                   (0x1800U)            /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDIO_CLKCR_WIDBUS_0                 (0x0800U)            /*!<Bit 0 */
#define  SDIO_CLKCR_WIDBUS_1                 (0x1000U)            /*!<Bit 1 */

#define  SDIO_CLKCR_NEGEDGE                  (0x2000U)            /*!<SDIO_CK dephasing selection bit */
#define  SDIO_CLKCR_HWFC_EN                  (0x4000U)            /*!<HW Flow Control enable          */

/*******************  Bit definition for SDIO_ARG register  *******************/
#define  SDIO_ARG_CMDARG                     (0xFFFFFFFFUL)            /*!<Command argument */

/*******************  Bit definition for SDIO_CMD register  *******************/
#define  SDIO_CMD_CMDINDEX                   (0x003FU)            /*!<Command Index                               */

#define  SDIO_CMD_WAITRESP                   (0x00C0U)            /*!<WAITRESP[1:0] bits (Wait for response bits) */
#define  SDIO_CMD_WAITRESP_0                 (0x0040U)            /*!< Bit 0 */
#define  SDIO_CMD_WAITRESP_1                 (0x0080U)            /*!< Bit 1 */

#define  SDIO_CMD_WAITINT                    (0x0100U)            /*!<CPSM Waits for Interrupt Request                               */
#define  SDIO_CMD_WAITPEND                   (0x0200U)            /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDIO_CMD_CPSMEN                     (0x0400U)            /*!<Command path state machine (CPSM) Enable bit                   */
#define  SDIO_CMD_SDIOSUSPEND                (0x0800U)            /*!<SD I/O suspend command                                         */
#define  SDIO_CMD_ENCMDCOMPL                 (0x1000U)            /*!<Enable CMD completion                                          */
#define  SDIO_CMD_NIEN                       (0x2000U)            /*!<Not Interrupt Enable */
#define  SDIO_CMD_CEATACMD                   (0x4000U)            /*!<CE-ATA command       */

/*****************  Bit definition for SDIO_RESPCMD register  *****************/
#define  SDIO_RESPCMD_RESPCMD                (0x3FU)               /*!<Response command index */

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
#define  SDIO_DCTRL_DTEN                     (0x0001U)            /*!<Data transfer enabled bit         */
#define  SDIO_DCTRL_DTDIR                    (0x0002U)            /*!<Data transfer direction selection */
#define  SDIO_DCTRL_DTMODE                   (0x0004U)            /*!<Data transfer mode selection      */
#define  SDIO_DCTRL_DMAEN                    (0x0008U)            /*!<DMA enabled bit                   */

#define  SDIO_DCTRL_DBLOCKSIZE               (0x00F0U)            /*!<DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDIO_DCTRL_DBLOCKSIZE_0             (0x0010U)            /*!<Bit 0 */
#define  SDIO_DCTRL_DBLOCKSIZE_1             (0x0020U)            /*!<Bit 1 */
#define  SDIO_DCTRL_DBLOCKSIZE_2             (0x0040U)            /*!<Bit 2 */
#define  SDIO_DCTRL_DBLOCKSIZE_3             (0x0080U)            /*!<Bit 3 */

#define  SDIO_DCTRL_RWSTART                  (0x0100U)            /*!<Read wait start         */
#define  SDIO_DCTRL_RWSTOP                   (0x0200U)            /*!<Read wait stop          */
#define  SDIO_DCTRL_RWMOD                    (0x0400U)            /*!<Read wait mode          */
#define  SDIO_DCTRL_SDIOEN                   (0x0800U)            /*!<SD I/O enable functions */

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
#define  SPI_CR1_CPHA                        (0x0001U)            /*!<Clock Phase      */
#define  SPI_CR1_CPOL                        (0x0002U)            /*!<Clock Polarity   */
#define  SPI_CR1_MSTR                        (0x0004U)            /*!<Master Selection */

#define  SPI_CR1_BR                          (0x0038U)            /*!<BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        (0x0008U)            /*!<Bit 0 */
#define  SPI_CR1_BR_1                        (0x0010U)            /*!<Bit 1 */
#define  SPI_CR1_BR_2                        (0x0020U)            /*!<Bit 2 */

#define  SPI_CR1_SPE                         (0x0040U)            /*!<SPI Enable                          */
#define  SPI_CR1_LSBFIRST                    (0x0080U)            /*!<Frame Format                        */
#define  SPI_CR1_SSI                         (0x0100U)            /*!<Internal slave select               */
#define  SPI_CR1_SSM                         (0x0200U)            /*!<Software slave management           */
#define  SPI_CR1_RXONLY                      (0x0400U)            /*!<Receive only                        */
#define  SPI_CR1_DFF                         (0x0800U)            /*!<Data Frame Format                   */
#define  SPI_CR1_CRCNEXT                     (0x1000U)            /*!<Transmit CRC next                   */
#define  SPI_CR1_CRCEN                       (0x2000U)            /*!<Hardware CRC calculation enable     */
#define  SPI_CR1_BIDIOE                      (0x4000U)            /*!<Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    (0x8000U)            /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     (0x01U)               /*!<Rx Buffer DMA Enable                 */
#define  SPI_CR2_TXDMAEN                     (0x02U)               /*!<Tx Buffer DMA Enable                 */
#define  SPI_CR2_SSOE                        (0x04U)               /*!<SS Output Enable                     */
#define  SPI_CR2_ERRIE                       (0x20U)               /*!<Error Interrupt Enable               */
#define  SPI_CR2_RXNEIE                      (0x40U)               /*!<RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       (0x80U)               /*!<Tx buffer Empty Interrupt Enable     */

/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         (0x01U)               /*!<Receive buffer Not Empty */
#define  SPI_SR_TXE                          (0x02U)               /*!<Transmit buffer Empty    */
#define  SPI_SR_CHSIDE                       (0x04U)               /*!<Channel side             */
#define  SPI_SR_UDR                          (0x08U)               /*!<Underrun flag            */
#define  SPI_SR_CRCERR                       (0x10U)               /*!<CRC Error flag           */
#define  SPI_SR_MODF                         (0x20U)               /*!<Mode fault               */
#define  SPI_SR_OVR                          (0x40U)               /*!<Overrun flag             */
#define  SPI_SR_BSY                          (0x80U)               /*!<Busy flag                */

/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           (0xFFFFU)            /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   (0xFFFFU)            /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    (0xFFFFU)            /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    (0xFFFFU)            /*!<Tx CRC Register         */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   (0x0001U)            /*!<Channel length (number of bits per audio channel) */

#define  SPI_I2SCFGR_DATLEN                  (0x0006U)            /*!<DATLEN[1:0] bits (Data length to be transferred)  */
#define  SPI_I2SCFGR_DATLEN_0                (0x0002U)            /*!<Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                (0x0004U)            /*!<Bit 1 */

#define  SPI_I2SCFGR_CKPOL                   (0x0008U)            /*!<steady state clock polarity               */

#define  SPI_I2SCFGR_I2SSTD                  (0x0030U)            /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                (0x0010U)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                (0x0020U)            /*!<Bit 1 */

#define  SPI_I2SCFGR_PCMSYNC                 (0x0080U)            /*!<PCM frame synchronization                 */

#define  SPI_I2SCFGR_I2SCFG                  (0x0300U)            /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                (0x0100U)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                (0x0200U)            /*!<Bit 1 */

#define  SPI_I2SCFGR_I2SE                    (0x0400U)            /*!<I2S Enable         */
#define  SPI_I2SCFGR_I2SMOD                  (0x0800U)            /*!<I2S mode selection */
#if defined(STM32F413_423xx) || defined(STM32F446xx)
#define  SPI_I2SCFGR_ASTRTEN                 (0x1000U)            /*!<Asynchronous start enable */
#endif /* STM32F413_423xx */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    (0x00FFU)            /*!<I2S Linear prescaler         */
#define  SPI_I2SPR_ODD                       (0x0100U)            /*!<Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     (0x0200U)            /*!<Master Clock Output Enable   */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/
#define SYSCFG_MEMRMP_MEM_MODE          (0x00000007UL) /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0        (0x00000001UL) /*!<Bit 0 */
#define SYSCFG_MEMRMP_MEM_MODE_1        (0x00000002UL) /*!<Bit 1 */
#define SYSCFG_MEMRMP_MEM_MODE_2        (0x00000004UL) /*!<Bit 2 */

#define SYSCFG_MEMRMP_FB_MODE           (0x00000100UL) /*!< User Flash Bank mode */

#define SYSCFG_MEMRMP_SWP_FMC           (0x00000C00UL) /*!< FMC memory mapping swap */
#define SYSCFG_MEMRMP_SWP_FMC_0         (0x00000400UL) /*!<Bit 0 */
#define SYSCFG_MEMRMP_SWP_FMC_1         (0x00000800UL) /*!<Bit 1 */


/******************  Bit definition for SYSCFG_PMC register  ******************/
#define SYSCFG_PMC_ADCxDC2              (0x00070000UL) /*!< Refer to AN4073 on how to use this bit  */
#define SYSCFG_PMC_ADC1DC2              (0x00010000UL) /*!< Refer to AN4073 on how to use this bit  */
#define SYSCFG_PMC_ADC2DC2              (0x00020000UL) /*!< Refer to AN4073 on how to use this bit  */
#define SYSCFG_PMC_ADC3DC2              (0x00040000UL) /*!< Refer to AN4073 on how to use this bit  */

#define SYSCFG_PMC_MII_RMII_SEL         (0x00800000UL) /*!<Ethernet PHY interface selection */
/* Old MII_RMII_SEL bit definition, maintained for legacy purpose */
#define SYSCFG_PMC_MII_RMII             SYSCFG_PMC_MII_RMII_SEL

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0            (0x000FU) /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1            (0x00F0U) /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2            (0x0F00U) /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3            (0xF000U) /*!<EXTI 3 configuration */
/**
  * @brief   EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA         (0x0000U) /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB         (0x0001U) /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC         (0x0002U) /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD         (0x0003U) /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE         (0x0004U) /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PF         (0x0005U) /*!<PF[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PG         (0x0006U) /*!<PG[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH         (0x0007U) /*!<PH[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PI         (0x0008U) /*!<PI[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PJ         (0x0009U) /*!<PJ[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PK         (0x000AU) /*!<PK[0] pin */

/**
  * @brief   EXTI1 configuration
  */
#define SYSCFG_EXTICR1_EXTI1_PA         (0x0000U) /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB         (0x0010U) /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC         (0x0020U) /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD         (0x0030U) /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE         (0x0040U) /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PF         (0x0050U) /*!<PF[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PG         (0x0060U) /*!<PG[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH         (0x0070U) /*!<PH[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PI         (0x0080U) /*!<PI[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PJ         (0x0090U) /*!<PJ[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PK         (0x00A0U) /*!<PK[1] pin */

/**
  * @brief   EXTI2 configuration
  */
#define SYSCFG_EXTICR1_EXTI2_PA         (0x0000U) /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB         (0x0100U) /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC         (0x0200U) /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD         (0x0300U) /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE         (0x0400U) /*!<PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PF         (0x0500U) /*!<PF[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PG         (0x0600U) /*!<PG[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PH         (0x0700U) /*!<PH[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PI         (0x0800U) /*!<PI[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PJ         (0x0900U) /*!<PJ[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PK         (0x0A00U) /*!<PK[2] pin */

/**
  * @brief   EXTI3 configuration
  */
#define SYSCFG_EXTICR1_EXTI3_PA         (0x0000U) /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB         (0x1000U) /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC         (0x2000U) /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD         (0x3000U) /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE         (0x4000U) /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PF         (0x5000U) /*!<PF[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PG         (0x6000U) /*!<PG[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PH         (0x7000U) /*!<PH[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PI         (0x8000U) /*!<PI[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PJ         (0x9000U) /*!<PJ[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PK         (0xA000U) /*!<PK[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4            (0x000FU) /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5            (0x00F0U) /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6            (0x0F00U) /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7            (0xF000U) /*!<EXTI 7 configuration */
/**
  * @brief   EXTI4 configuration
  */
#define SYSCFG_EXTICR2_EXTI4_PA         (0x0000U) /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB         (0x0001U) /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC         (0x0002U) /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD         (0x0003U) /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE         (0x0004U) /*!<PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PF         (0x0005U) /*!<PF[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PG         (0x0006U) /*!<PG[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PH         (0x0007U) /*!<PH[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PI         (0x0008U) /*!<PI[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PJ         (0x0009U) /*!<PJ[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PK         (0x000AU) /*!<PK[4] pin */

/**
  * @brief   EXTI5 configuration
  */
#define SYSCFG_EXTICR2_EXTI5_PA         (0x0000U) /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB         (0x0010U) /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC         (0x0020U) /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD         (0x0030U) /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE         (0x0040U) /*!<PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PF         (0x0050U) /*!<PF[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PG         (0x0060U) /*!<PG[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PH         (0x0070U) /*!<PH[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PI         (0x0080U) /*!<PI[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PJ         (0x0090U) /*!<PJ[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PK         (0x00A0U) /*!<PK[5] pin */

/**
  * @brief   EXTI6 configuration
  */
#define SYSCFG_EXTICR2_EXTI6_PA         (0x0000U) /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB         (0x0100U) /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC         (0x0200U) /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD         (0x0300U) /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE         (0x0400U) /*!<PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PF         (0x0500U) /*!<PF[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PG         (0x0600U) /*!<PG[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PH         (0x0700U) /*!<PH[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PI         (0x0800U) /*!<PI[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PJ         (0x0900U) /*!<PJ[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PK         (0x0A00U) /*!<PK[6] pin */

/**
  * @brief   EXTI7 configuration
  */
#define SYSCFG_EXTICR2_EXTI7_PA         (0x0000U) /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB         (0x1000U) /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC         (0x2000U) /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD         (0x3000U) /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE         (0x4000U) /*!<PE[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PF         (0x5000U) /*!<PF[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PG         (0x6000U) /*!<PG[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PH         (0x7000U) /*!<PH[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PI         (0x8000U) /*!<PI[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PJ         (0x9000U) /*!<PJ[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PK         (0xA000U) /*!<PK[7] pin */

/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8            (0x000FU) /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9            (0x00F0U) /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10           (0x0F00U) /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11           (0xF000U) /*!<EXTI 11 configuration */

/**
  * @brief   EXTI8 configuration
  */
#define SYSCFG_EXTICR3_EXTI8_PA         (0x0000U) /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB         (0x0001U) /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC         (0x0002U) /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD         (0x0003U) /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE         (0x0004U) /*!<PE[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PF         (0x0005U) /*!<PF[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PG         (0x0006U) /*!<PG[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PH         (0x0007U) /*!<PH[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PI         (0x0008U) /*!<PI[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PJ         (0x0009U) /*!<PJ[8] pin */

/**
  * @brief   EXTI9 configuration
  */
#define SYSCFG_EXTICR3_EXTI9_PA         (0x0000U) /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB         (0x0010U) /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC         (0x0020U) /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD         (0x0030U) /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE         (0x0040U) /*!<PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PF         (0x0050U) /*!<PF[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PG         (0x0060U) /*!<PG[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PH         (0x0070U) /*!<PH[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PI         (0x0080U) /*!<PI[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PJ         (0x0090U) /*!<PJ[9] pin */

/**
  * @brief   EXTI10 configuration
  */
#define SYSCFG_EXTICR3_EXTI10_PA        (0x0000U) /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB        (0x0100U) /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC        (0x0200U) /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD        (0x0300U) /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE        (0x0400U) /*!<PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PF        (0x0500U) /*!<PF[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PG        (0x0600U) /*!<PG[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PH        (0x0700U) /*!<PH[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PI        (0x0800U) /*!<PI[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PJ        (0x0900U) /*!<PJ[10] pin */

/**
  * @brief   EXTI11 configuration
  */
#define SYSCFG_EXTICR3_EXTI11_PA        (0x0000U) /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB        (0x1000U) /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC        (0x2000U) /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD        (0x3000U) /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE        (0x4000U) /*!<PE[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PF        (0x5000U) /*!<PF[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PG        (0x6000U) /*!<PG[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PH        (0x7000U) /*!<PH[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PI        (0x8000U) /*!<PI[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PJ        (0x9000U) /*!<PJ[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12           (0x000FU) /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13           (0x00F0U) /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14           (0x0F00U) /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15           (0xF000U) /*!<EXTI 15 configuration */
/**
  * @brief   EXTI12 configuration
  */
#define SYSCFG_EXTICR4_EXTI12_PA        (0x0000U) /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB        (0x0001U) /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC        (0x0002U) /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD        (0x0003U) /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE        (0x0004U) /*!<PE[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PF        (0x0005U) /*!<PF[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PG        (0x0006U) /*!<PG[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PH        (0x0007U) /*!<PH[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PI        (0x0008U) /*!<PI[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PJ        (0x0009U) /*!<PJ[12] pin */

/**
  * @brief   EXTI13 configuration
  */
#define SYSCFG_EXTICR4_EXTI13_PA        (0x0000U) /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB        (0x0010U) /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC        (0x0020U) /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD        (0x0030U) /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE        (0x0040U) /*!<PE[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PF        (0x0050U) /*!<PF[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PG        (0x0060U) /*!<PG[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PH        (0x0070U) /*!<PH[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PI        (0x0008U) /*!<PI[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PJ        (0x0009U) /*!<PJ[13] pin */

/**
  * @brief   EXTI14 configuration
  */
#define SYSCFG_EXTICR4_EXTI14_PA        (0x0000U) /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB        (0x0100U) /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC        (0x0200U) /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD        (0x0300U) /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE        (0x0400U) /*!<PE[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PF        (0x0500U) /*!<PF[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PG        (0x0600U) /*!<PG[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PH        (0x0700U) /*!<PH[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PI        (0x0800U) /*!<PI[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PJ        (0x0900U) /*!<PJ[14] pin */

/**
  * @brief   EXTI15 configuration
  */
#define SYSCFG_EXTICR4_EXTI15_PA        (0x0000U) /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB        (0x1000U) /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC        (0x2000U) /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD        (0x3000U) /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE        (0x4000U) /*!<PE[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PF        (0x5000U) /*!<PF[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PG        (0x6000U) /*!<PG[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PH        (0x7000U) /*!<PH[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PI        (0x8000U) /*!<PI[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PJ        (0x9000U) /*!<PJ[15] pin */

#if defined(STM32F412xG) || defined(STM32F413_423xx)
/******************  Bit definition for SYSCFG_CFGR register  *****************/
#define SYSCFG_CFGR_FMPI2C1_SCL         (0x00000001UL) /*!<FM+ drive capability for FMPI2C1_SCL pin */
#define SYSCFG_CFGR_FMPI2C1_SDA         (0x00000002UL) /*!<FM+ drive capability for FMPI2C1_SDA pin */
#endif /* STM32F412xG || STM32413_423xx */

#if defined (STM32F410xx) || defined(STM32F412xG) || defined(STM32F413_423xx)
/******************  Bit definition for SYSCFG_CFGR2 register  ****************/
#define SYSCFG_CFGR2_CLL                (0x00000001UL) /*!< Core Lockup Lock */
#define SYSCFG_CFGR2_PVDL               (0x00000004UL) /*!<  PVD Lock */
#endif /* STM32F410xx || STM32F412xG || STM32F413_423xx */
/******************  Bit definition for SYSCFG_CMPCR register  ****************/
#define SYSCFG_CMPCR_CMP_PD             (0x00000001UL) /*!<Compensation cell ready flag */
#define SYSCFG_CMPCR_READY              (0x00000100UL) /*!<Compensation cell power-down */

#if defined(STM32F413_423xx)
/******************  Bit definition for SYSCFG_MCHDLYCR register  *****************/
#define SYSCFG_MCHDLYCR_BSCKSEL         (0x00000001UL) /*!<Bitstream clock source selection                     */
#define SYSCFG_MCHDLYCR_MCHDLY1EN       (0x00000002UL) /*!<MCHDLY clock enable for DFSDM1                       */
#define SYSCFG_MCHDLYCR_DFSDM1D0SEL     (0x00000004UL) /*!<Source selection for DatIn0 for DFSDM1               */
#define SYSCFG_MCHDLYCR_DFSDM1D2SEL     (0x00000008UL) /*!<Source selection for DatIn2 for DFSDM1               */
#define SYSCFG_MCHDLYCR_DFSDM1CK02SEL   (0x00000010UL) /*!<Distribution of the bitstreamclock gated by TIM4 OC2 */
#define SYSCFG_MCHDLYCR_DFSDM1CK13SEL   (0x00000020UL) /*!<Distribution of the bitstreamclock gated by TIM4 OC1 */
#define SYSCFG_MCHDLYCR_DFSDM1CFG       (0x00000040UL) /*!<Source selection for DFSDM1                          */
#define SYSCFG_MCHDLYCR_DFSDM1CKOSEL    (0x00000080UL) /*!<Source selection for 1_CKOUT                         */
#define SYSCFG_MCHDLYCR_MCHDLY2EN       (0x00000100UL) /*!<MCHDLY clock enable for DFSDM2                       */
#define SYSCFG_MCHDLYCR_DFSDM2D0SEL     (0x00000200UL) /*!<Source selection for DatIn0 for DFSDM2               */
#define SYSCFG_MCHDLYCR_DFSDM2D2SEL     (0x00000400UL) /*!<Source selection for DatIn2 for DFSDM2               */
#define SYSCFG_MCHDLYCR_DFSDM2D4SEL     (0x00000800UL) /*!<Source selection for DatIn4 for DFSDM2               */
#define SYSCFG_MCHDLYCR_DFSDM2D6SEL     (0x00001000UL) /*!<Source selection for DatIn6 for DFSDM2               */
#define SYSCFG_MCHDLYCR_DFSDM2CK04SEL   (0x00002000UL) /*!<Distribution of the bitstreamclock gated by TIM3 OC4 */
#define SYSCFG_MCHDLYCR_DFSDM2CK15SEL   (0x00004000UL) /*!<Distribution of the bitstreamclock gated by TIM3 OC3 */
#define SYSCFG_MCHDLYCR_DFSDM2CK26SEL   (0x00008000UL) /*!Distribution of the bitstreamclock gated by TIM3 OC2  */
#define SYSCFG_MCHDLYCR_DFSDM2CK37SEL   (0x00010000UL) /*!<Distribution of the bitstreamclock gated by TIM3 OC1 */
#define SYSCFG_MCHDLYCR_DFSDM2CFG       (0x00020000UL) /*!<Source selection for DFSDM2                          */
#define SYSCFG_MCHDLYCR_DFSDM2CKOSEL    (0x00040000UL) /*!<Source selection for 2_CKOUT                         */
#endif /* STM32F413_423xx */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN                         (0x0001U)            /*!<Counter enable        */
#define  TIM_CR1_UDIS                        (0x0002U)            /*!<Update disable        */
#define  TIM_CR1_URS                         (0x0004U)            /*!<Update request source */
#define  TIM_CR1_OPM                         (0x0008U)            /*!<One pulse mode        */
#define  TIM_CR1_DIR                         (0x0010U)            /*!<Direction             */

#define  TIM_CR1_CMS                         (0x0060U)            /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0                       (0x0020U)            /*!<Bit 0 */
#define  TIM_CR1_CMS_1                       (0x0040U)            /*!<Bit 1 */

#define  TIM_CR1_ARPE                        (0x0080U)            /*!<Auto-reload preload enable     */

#define  TIM_CR1_CKD                         (0x0300U)            /*!<CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0                       (0x0100U)            /*!<Bit 0 */
#define  TIM_CR1_CKD_1                       (0x0200U)            /*!<Bit 1 */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC                        (0x0001U)            /*!<Capture/Compare Preloaded Control        */
#define  TIM_CR2_CCUS                        (0x0004U)            /*!<Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS                        (0x0008U)            /*!<Capture/Compare DMA Selection            */

#define  TIM_CR2_MMS                         (0x0070U)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0                       (0x0010U)            /*!<Bit 0 */
#define  TIM_CR2_MMS_1                       (0x0020U)            /*!<Bit 1 */
#define  TIM_CR2_MMS_2                       (0x0040U)            /*!<Bit 2 */

#define  TIM_CR2_TI1S                        (0x0080U)            /*!<TI1 Selection */
#define  TIM_CR2_OIS1                        (0x0100U)            /*!<Output Idle state 1 (OC1 output)  */
#define  TIM_CR2_OIS1N                       (0x0200U)            /*!<Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2                        (0x0400U)            /*!<Output Idle state 2 (OC2 output)  */
#define  TIM_CR2_OIS2N                       (0x0800U)            /*!<Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3                        (0x1000U)            /*!<Output Idle state 3 (OC3 output)  */
#define  TIM_CR2_OIS3N                       (0x2000U)            /*!<Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4                        (0x4000U)            /*!<Output Idle state 4 (OC4 output)  */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS                        (0x0007U)            /*!<SMS[2:0] bits (Slave mode selection)    */
#define  TIM_SMCR_SMS_0                      (0x0001U)            /*!<Bit 0 */
#define  TIM_SMCR_SMS_1                      (0x0002U)            /*!<Bit 1 */
#define  TIM_SMCR_SMS_2                      (0x0004U)            /*!<Bit 2 */

#define  TIM_SMCR_TS                         (0x0070U)            /*!<TS[2:0] bits (Trigger selection)        */
#define  TIM_SMCR_TS_0                       (0x0010U)            /*!<Bit 0 */
#define  TIM_SMCR_TS_1                       (0x0020U)            /*!<Bit 1 */
#define  TIM_SMCR_TS_2                       (0x0040U)            /*!<Bit 2 */

#define  TIM_SMCR_MSM                        (0x0080U)            /*!<Master/slave mode                       */

#define  TIM_SMCR_ETF                        (0x0F00U)            /*!<ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0                      (0x0100U)            /*!<Bit 0 */
#define  TIM_SMCR_ETF_1                      (0x0200U)            /*!<Bit 1 */
#define  TIM_SMCR_ETF_2                      (0x0400U)            /*!<Bit 2 */
#define  TIM_SMCR_ETF_3                      (0x0800U)            /*!<Bit 3 */

#define  TIM_SMCR_ETPS                       (0x3000U)            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0                     (0x1000U)            /*!<Bit 0 */
#define  TIM_SMCR_ETPS_1                     (0x2000U)            /*!<Bit 1 */

#define  TIM_SMCR_ECE                        (0x4000U)            /*!<External clock enable     */
#define  TIM_SMCR_ETP                        (0x8000U)            /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE                        (0x0001U)            /*!<Update interrupt enable */
#define  TIM_DIER_CC1IE                      (0x0002U)            /*!<Capture/Compare 1 interrupt enable   */
#define  TIM_DIER_CC2IE                      (0x0004U)            /*!<Capture/Compare 2 interrupt enable   */
#define  TIM_DIER_CC3IE                      (0x0008U)            /*!<Capture/Compare 3 interrupt enable   */
#define  TIM_DIER_CC4IE                      (0x0010U)            /*!<Capture/Compare 4 interrupt enable   */
#define  TIM_DIER_COMIE                      (0x0020U)            /*!<COM interrupt enable                 */
#define  TIM_DIER_TIE                        (0x0040U)            /*!<Trigger interrupt enable             */
#define  TIM_DIER_BIE                        (0x0080U)            /*!<Break interrupt enable               */
#define  TIM_DIER_UDE                        (0x0100U)            /*!<Update DMA request enable            */
#define  TIM_DIER_CC1DE                      (0x0200U)            /*!<Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE                      (0x0400U)            /*!<Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE                      (0x0800U)            /*!<Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE                      (0x1000U)            /*!<Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE                      (0x2000U)            /*!<COM DMA request enable               */
#define  TIM_DIER_TDE                        (0x4000U)            /*!<Trigger DMA request enable           */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                          (0x0001U)            /*!<Update interrupt Flag              */
#define  TIM_SR_CC1IF                        (0x0002U)            /*!<Capture/Compare 1 interrupt Flag   */
#define  TIM_SR_CC2IF                        (0x0004U)            /*!<Capture/Compare 2 interrupt Flag   */
#define  TIM_SR_CC3IF                        (0x0008U)            /*!<Capture/Compare 3 interrupt Flag   */
#define  TIM_SR_CC4IF                        (0x0010U)            /*!<Capture/Compare 4 interrupt Flag   */
#define  TIM_SR_COMIF                        (0x0020U)            /*!<COM interrupt Flag                 */
#define  TIM_SR_TIF                          (0x0040U)            /*!<Trigger interrupt Flag             */
#define  TIM_SR_BIF                          (0x0080U)            /*!<Break interrupt Flag               */
#define  TIM_SR_CC1OF                        (0x0200U)            /*!<Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF                        (0x0400U)            /*!<Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF                        (0x0800U)            /*!<Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF                        (0x1000U)            /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                          (0x01U)               /*!<Update Generation                         */
#define  TIM_EGR_CC1G                        (0x02U)               /*!<Capture/Compare 1 Generation              */
#define  TIM_EGR_CC2G                        (0x04U)               /*!<Capture/Compare 2 Generation              */
#define  TIM_EGR_CC3G                        (0x08U)               /*!<Capture/Compare 3 Generation              */
#define  TIM_EGR_CC4G                        (0x10U)               /*!<Capture/Compare 4 Generation              */
#define  TIM_EGR_COMG                        (0x20U)               /*!<Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                          (0x40U)               /*!<Trigger Generation                        */
#define  TIM_EGR_BG                          (0x80U)               /*!<Break Generation                          */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S                      (0x0003U)            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0                    (0x0001U)            /*!<Bit 0 */
#define  TIM_CCMR1_CC1S_1                    (0x0002U)            /*!<Bit 1 */

#define  TIM_CCMR1_OC1FE                     (0x0004U)            /*!<Output Compare 1 Fast enable                 */
#define  TIM_CCMR1_OC1PE                     (0x0008U)            /*!<Output Compare 1 Preload enable              */

#define  TIM_CCMR1_OC1M                      (0x0070U)            /*!<OC1M[2:0] bits (Output Compare 1 Mode)       */
#define  TIM_CCMR1_OC1M_0                    (0x0010U)            /*!<Bit 0 */
#define  TIM_CCMR1_OC1M_1                    (0x0020U)            /*!<Bit 1 */
#define  TIM_CCMR1_OC1M_2                    (0x0040U)            /*!<Bit 2 */

#define  TIM_CCMR1_OC1CE                     (0x0080U)            /*!<Output Compare 1Clear Enable                 */

#define  TIM_CCMR1_CC2S                      (0x0300U)            /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0                    (0x0100U)            /*!<Bit 0 */
#define  TIM_CCMR1_CC2S_1                    (0x0200U)            /*!<Bit 1 */

#define  TIM_CCMR1_OC2FE                     (0x0400U)            /*!<Output Compare 2 Fast enable                 */
#define  TIM_CCMR1_OC2PE                     (0x0800U)            /*!<Output Compare 2 Preload enable              */

#define  TIM_CCMR1_OC2M                      (0x7000U)            /*!<OC2M[2:0] bits (Output Compare 2 Mode)       */
#define  TIM_CCMR1_OC2M_0                    (0x1000U)            /*!<Bit 0 */
#define  TIM_CCMR1_OC2M_1                    (0x2000U)            /*!<Bit 1 */
#define  TIM_CCMR1_OC2M_2                    (0x4000U)            /*!<Bit 2 */

#define  TIM_CCMR1_OC2CE                     (0x8000U)            /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR1_IC1PSC                    (0x000CU)            /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0                  (0x0004U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1PSC_1                  (0x0008U)            /*!<Bit 1 */

#define  TIM_CCMR1_IC1F                      (0x00F0U)            /*!<IC1F[3:0] bits (Input Capture 1 Filter)      */
#define  TIM_CCMR1_IC1F_0                    (0x0010U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1F_1                    (0x0020U)            /*!<Bit 1 */
#define  TIM_CCMR1_IC1F_2                    (0x0040U)            /*!<Bit 2 */
#define  TIM_CCMR1_IC1F_3                    (0x0080U)            /*!<Bit 3 */

#define  TIM_CCMR1_IC2PSC                    (0x0C00U)            /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler)  */
#define  TIM_CCMR1_IC2PSC_0                  (0x0400U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2PSC_1                  (0x0800U)            /*!<Bit 1 */

#define  TIM_CCMR1_IC2F                      (0xF000U)            /*!<IC2F[3:0] bits (Input Capture 2 Filter)       */
#define  TIM_CCMR1_IC2F_0                    (0x1000U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2F_1                    (0x2000U)            /*!<Bit 1 */
#define  TIM_CCMR1_IC2F_2                    (0x4000U)            /*!<Bit 2 */
#define  TIM_CCMR1_IC2F_3                    (0x8000U)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S                      (0x0003U)            /*!<CC3S[1:0] bits (Capture/Compare 3 Selection)  */
#define  TIM_CCMR2_CC3S_0                    (0x0001U)            /*!<Bit 0 */
#define  TIM_CCMR2_CC3S_1                    (0x0002U)            /*!<Bit 1 */

#define  TIM_CCMR2_OC3FE                     (0x0004U)            /*!<Output Compare 3 Fast enable           */
#define  TIM_CCMR2_OC3PE                     (0x0008U)            /*!<Output Compare 3 Preload enable        */

#define  TIM_CCMR2_OC3M                      (0x0070U)            /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0                    (0x0010U)            /*!<Bit 0 */
#define  TIM_CCMR2_OC3M_1                    (0x0020U)            /*!<Bit 1 */
#define  TIM_CCMR2_OC3M_2                    (0x0040U)            /*!<Bit 2 */

#define  TIM_CCMR2_OC3CE                     (0x0080U)            /*!<Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S                      (0x0300U)            /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0                    (0x0100U)            /*!<Bit 0 */
#define  TIM_CCMR2_CC4S_1                    (0x0200U)            /*!<Bit 1 */

#define  TIM_CCMR2_OC4FE                     (0x0400U)            /*!<Output Compare 4 Fast enable    */
#define  TIM_CCMR2_OC4PE                     (0x0800U)            /*!<Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M                      (0x7000U)            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0                    (0x1000U)            /*!<Bit 0 */
#define  TIM_CCMR2_OC4M_1                    (0x2000U)            /*!<Bit 1 */
#define  TIM_CCMR2_OC4M_2                    (0x4000U)            /*!<Bit 2 */

#define  TIM_CCMR2_OC4CE                     (0x8000U)            /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR2_IC3PSC                    (0x000CU)            /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0                  (0x0004U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3PSC_1                  (0x0008U)            /*!<Bit 1 */

#define  TIM_CCMR2_IC3F                      (0x00F0U)            /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0                    (0x0010U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3F_1                    (0x0020U)            /*!<Bit 1 */
#define  TIM_CCMR2_IC3F_2                    (0x0040U)            /*!<Bit 2 */
#define  TIM_CCMR2_IC3F_3                    (0x0080U)            /*!<Bit 3 */

#define  TIM_CCMR2_IC4PSC                    (0x0C00U)            /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0                  (0x0400U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4PSC_1                  (0x0800U)            /*!<Bit 1 */

#define  TIM_CCMR2_IC4F                      (0xF000U)            /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0                    (0x1000U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4F_1                    (0x2000U)            /*!<Bit 1 */
#define  TIM_CCMR2_IC4F_2                    (0x4000U)            /*!<Bit 2 */
#define  TIM_CCMR2_IC4F_3                    (0x8000U)            /*!<Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E                       (0x0001U)            /*!<Capture/Compare 1 output enable                 */
#define  TIM_CCER_CC1P                       (0x0002U)            /*!<Capture/Compare 1 output Polarity               */
#define  TIM_CCER_CC1NE                      (0x0004U)            /*!<Capture/Compare 1 Complementary output enable   */
#define  TIM_CCER_CC1NP                      (0x0008U)            /*!<Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E                       (0x0010U)            /*!<Capture/Compare 2 output enable                 */
#define  TIM_CCER_CC2P                       (0x0020U)            /*!<Capture/Compare 2 output Polarity               */
#define  TIM_CCER_CC2NE                      (0x0040U)            /*!<Capture/Compare 2 Complementary output enable   */
#define  TIM_CCER_CC2NP                      (0x0080U)            /*!<Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E                       (0x0100U)            /*!<Capture/Compare 3 output enable                 */
#define  TIM_CCER_CC3P                       (0x0200U)            /*!<Capture/Compare 3 output Polarity               */
#define  TIM_CCER_CC3NE                      (0x0400U)            /*!<Capture/Compare 3 Complementary output enable   */
#define  TIM_CCER_CC3NP                      (0x0800U)            /*!<Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E                       (0x1000U)            /*!<Capture/Compare 4 output enable                 */
#define  TIM_CCER_CC4P                       (0x2000U)            /*!<Capture/Compare 4 output Polarity               */
#define  TIM_CCER_CC4NP                      (0x8000U)            /*!<Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT                         (0xFFFFU)            /*!<Counter Value            */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC                         (0xFFFFU)            /*!<Prescaler Value          */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR                         (0xFFFFU)            /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP                         (0xFFU)               /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1                       (0xFFFFU)            /*!<Capture/Compare 1 Value  */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2                       (0xFFFFU)            /*!<Capture/Compare 2 Value  */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3                       (0xFFFFU)            /*!<Capture/Compare 3 Value  */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4                       (0xFFFFU)            /*!<Capture/Compare 4 Value  */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG                        (0x00FFU)            /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0                      (0x0001U)            /*!<Bit 0 */
#define  TIM_BDTR_DTG_1                      (0x0002U)            /*!<Bit 1 */
#define  TIM_BDTR_DTG_2                      (0x0004U)            /*!<Bit 2 */
#define  TIM_BDTR_DTG_3                      (0x0008U)            /*!<Bit 3 */
#define  TIM_BDTR_DTG_4                      (0x0010U)            /*!<Bit 4 */
#define  TIM_BDTR_DTG_5                      (0x0020U)            /*!<Bit 5 */
#define  TIM_BDTR_DTG_6                      (0x0040U)            /*!<Bit 6 */
#define  TIM_BDTR_DTG_7                      (0x0080U)            /*!<Bit 7 */

#define  TIM_BDTR_LOCK                       (0x0300U)            /*!<LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0                     (0x0100U)            /*!<Bit 0 */
#define  TIM_BDTR_LOCK_1                     (0x0200U)            /*!<Bit 1 */

#define  TIM_BDTR_OSSI                       (0x0400U)            /*!<Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR                       (0x0800U)            /*!<Off-State Selection for Run mode  */
#define  TIM_BDTR_BKE                        (0x1000U)            /*!<Break enable                      */
#define  TIM_BDTR_BKP                        (0x2000U)            /*!<Break Polarity                    */
#define  TIM_BDTR_AOE                        (0x4000U)            /*!<Automatic Output enable           */
#define  TIM_BDTR_MOE                        (0x8000U)            /*!<Main Output enable                */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA                         (0x001FU)            /*!<DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0                       (0x0001U)            /*!<Bit 0 */
#define  TIM_DCR_DBA_1                       (0x0002U)            /*!<Bit 1 */
#define  TIM_DCR_DBA_2                       (0x0004U)            /*!<Bit 2 */
#define  TIM_DCR_DBA_3                       (0x0008U)            /*!<Bit 3 */
#define  TIM_DCR_DBA_4                       (0x0010U)            /*!<Bit 4 */

#define  TIM_DCR_DBL                         (0x1F00U)            /*!<DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0                       (0x0100U)            /*!<Bit 0 */
#define  TIM_DCR_DBL_1                       (0x0200U)            /*!<Bit 1 */
#define  TIM_DCR_DBL_2                       (0x0400U)            /*!<Bit 2 */
#define  TIM_DCR_DBL_3                       (0x0800U)            /*!<Bit 3 */
#define  TIM_DCR_DBL_4                       (0x1000U)            /*!<Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB                       (0xFFFFU)            /*!<DMA register for burst accesses                    */

/*******************  Bit definition for TIM_OR register  *********************/
#define TIM_OR_TI4_RMP                       (0x00C0U)            /*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)             */
#define TIM_OR_TI4_RMP_0                     (0x0040U)            /*!<Bit 0 */
#define TIM_OR_TI4_RMP_1                     (0x0080U)            /*!<Bit 1 */
#define TIM_OR_ITR1_RMP                      (0x0C00U)            /*!<ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap) */
#define TIM_OR_ITR1_RMP_0                    (0x0400U)            /*!<Bit 0 */
#define TIM_OR_ITR1_RMP_1                    (0x0800U)            /*!<Bit 1 */

#if defined(STM32F410xx) || defined(STM32F413_423xx)
/******************************************************************************/
/*                                                                            */
/*                         Low Power Timer (LPTIM)                            */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for LPTIM_ISR register  *******************/
#define  LPTIM_ISR_CMPM                         (0x00000001UL)            /*!< Compare match                       */
#define  LPTIM_ISR_ARRM                         (0x00000002UL)            /*!< Autoreload match                    */
#define  LPTIM_ISR_EXTTRIG                      (0x00000004UL)            /*!< External trigger edge event         */
#define  LPTIM_ISR_CMPOK                        (0x00000008UL)            /*!< Compare register update OK          */
#define  LPTIM_ISR_ARROK                        (0x00000010UL)            /*!< Autoreload register update OK       */
#define  LPTIM_ISR_UP                           (0x00000020UL)            /*!< Counter direction change down to up */
#define  LPTIM_ISR_DOWN                         (0x00000040UL)            /*!< Counter direction change up to down */

/******************  Bit definition for LPTIM_ICR register  *******************/
#define  LPTIM_ICR_CMPMCF                       (0x00000001UL)            /*!< Compare match Clear Flag                       */
#define  LPTIM_ICR_ARRMCF                       (0x00000002UL)            /*!< Autoreload match Clear Flag                    */
#define  LPTIM_ICR_EXTTRIGCF                    (0x00000004UL)            /*!< External trigger edge event Clear Flag         */
#define  LPTIM_ICR_CMPOKCF                      (0x00000008UL)            /*!< Compare register update OK Clear Flag          */
#define  LPTIM_ICR_ARROKCF                      (0x00000010UL)            /*!< Autoreload register update OK Clear Flag       */
#define  LPTIM_ICR_UPCF                         (0x00000020UL)            /*!< Counter direction change down to up Clear Flag */
#define  LPTIM_ICR_DOWNCF                       (0x00000040UL)            /*!< Counter direction change up to down Clear Flag */

/******************  Bit definition for LPTIM_IER register ********************/
#define  LPTIM_IER_CMPMIE                       (0x00000001UL)            /*!< Compare match Interrupt Enable                       */
#define  LPTIM_IER_ARRMIE                       (0x00000002UL)            /*!< Autoreload match Interrupt Enable                    */
#define  LPTIM_IER_EXTTRIGIE                    (0x00000004UL)            /*!< External trigger edge event Interrupt Enable         */
#define  LPTIM_IER_CMPOKIE                      (0x00000008UL)            /*!< Compare register update OK Interrupt Enable          */
#define  LPTIM_IER_ARROKIE                      (0x00000010UL)            /*!< Autoreload register update OK Interrupt Enable       */
#define  LPTIM_IER_UPIE                         (0x00000020UL)            /*!< Counter direction change down to up Interrupt Enable */
#define  LPTIM_IER_DOWNIE                       (0x00000040UL)            /*!< Counter direction change up to down Interrupt Enable */

/******************  Bit definition for LPTIM_CFGR register *******************/
#define  LPTIM_CFGR_CKSEL                       (0x00000001UL)             /*!< Clock selector */

#define  LPTIM_CFGR_CKPOL                       (0x00000006UL)             /*!< CKPOL[1:0] bits (Clock polarity) */
#define  LPTIM_CFGR_CKPOL_0                     (0x00000002UL)             /*!< Bit 0 */
#define  LPTIM_CFGR_CKPOL_1                     (0x00000004UL)             /*!< Bit 1 */

#define  LPTIM_CFGR_CKFLT                       (0x00000018UL)             /*!< CKFLT[1:0] bits (Configurable digital filter for external clock) */
#define  LPTIM_CFGR_CKFLT_0                     (0x00000008UL)             /*!< Bit 0 */
#define  LPTIM_CFGR_CKFLT_1                     (0x00000010UL)             /*!< Bit 1 */

#define  LPTIM_CFGR_TRGFLT                      (0x000000C0UL)             /*!< TRGFLT[1:0] bits (Configurable digital filter for trigger) */
#define  LPTIM_CFGR_TRGFLT_0                    (0x00000040UL)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRGFLT_1                    (0x00000080UL)             /*!< Bit 1 */

#define  LPTIM_CFGR_PRESC                       (0x00000E00UL)             /*!< PRESC[2:0] bits (Clock prescaler) */
#define  LPTIM_CFGR_PRESC_0                     (0x00000200UL)             /*!< Bit 0 */
#define  LPTIM_CFGR_PRESC_1                     (0x00000400UL)             /*!< Bit 1 */
#define  LPTIM_CFGR_PRESC_2                     (0x00000800UL)             /*!< Bit 2 */

#define  LPTIM_CFGR_TRIGSEL                     (0x0000E000UL)             /*!< TRIGSEL[2:0]] bits (Trigger selector) */
#define  LPTIM_CFGR_TRIGSEL_0                   (0x00002000UL)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRIGSEL_1                   (0x00004000UL)             /*!< Bit 1 */
#define  LPTIM_CFGR_TRIGSEL_2                   (0x00008000UL)             /*!< Bit 2 */

#define  LPTIM_CFGR_TRIGEN                      (0x00060000UL)             /*!< TRIGEN[1:0] bits (Trigger enable and polarity) */
#define  LPTIM_CFGR_TRIGEN_0                    (0x00020000UL)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRIGEN_1                    (0x00040000UL)             /*!< Bit 1 */

#define  LPTIM_CFGR_TIMOUT                      (0x00080000UL)             /*!< Timout enable           */
#define  LPTIM_CFGR_WAVE                        (0x00100000UL)             /*!< Waveform shape          */
#define  LPTIM_CFGR_WAVPOL                      (0x00200000UL)             /*!< Waveform shape polarity */
#define  LPTIM_CFGR_PRELOAD                     (0x00400000UL)             /*!< Reg update mode         */
#define  LPTIM_CFGR_COUNTMODE                   (0x00800000UL)             /*!< Counter mode enable     */
#define  LPTIM_CFGR_ENC                         (0x01000000UL)             /*!< Encoder mode enable     */

/******************  Bit definition for LPTIM_CR register  ********************/
#define  LPTIM_CR_ENABLE                        (0x00000001UL)             /*!< LPTIMer enable                 */
#define  LPTIM_CR_SNGSTRT                       (0x00000002UL)             /*!< Timer start in single mode     */
#define  LPTIM_CR_CNTSTRT                       (0x00000004UL)             /*!< Timer start in continuous mode */

/******************  Bit definition for LPTIM_CMP register  *******************/
#define  LPTIM_CMP_CMP                          (0x0000FFFFUL)             /*!< Compare register     */

/******************  Bit definition for LPTIM_ARR register  *******************/
#define  LPTIM_ARR_ARR                          (0x0000FFFFUL)             /*!< Auto reload register */

/******************  Bit definition for LPTIM_CNT register  *******************/
#define  LPTIM_CNT_CNT                          (0x0000FFFFUL)             /*!< Counter register     */

/******************  Bit definition for LPTIM_OR register  *******************/
#define  LPTIM_OR_OR                           (0x00000003UL)               /*!< LPTIMER[1:0] bits (Remap selection) */
#define  LPTIM_OR_OR_0                         (0x00000001UL)               /*!< Bit 0 */
#define  LPTIM_OR_OR_1                         (0x00000002UL)               /*!< Bit 1 */
#endif /* STM32F410xx || STM32F413_423xx */

/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define  USART_SR_PE                         (0x0001U)            /*!<Parity Error                 */
#define  USART_SR_FE                         (0x0002U)            /*!<Framing Error                */
#define  USART_SR_NE                         (0x0004U)            /*!<Noise Error Flag             */
#define  USART_SR_ORE                        (0x0008U)            /*!<OverRun Error                */
#define  USART_SR_IDLE                       (0x0010U)            /*!<IDLE line detected           */
#define  USART_SR_RXNE                       (0x0020U)            /*!<Read Data Register Not Empty */
#define  USART_SR_TC                         (0x0040U)            /*!<Transmission Complete        */
#define  USART_SR_TXE                        (0x0080U)            /*!<Transmit Data Register Empty */
#define  USART_SR_LBD                        (0x0100U)            /*!<LIN Break Detection Flag     */
#define  USART_SR_CTS                        (0x0200U)            /*!<CTS Flag                     */

/*******************  Bit definition for USART_DR register  *******************/
#define  USART_DR_DR                         (0x01FFU)            /*!<Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_Fraction              (0x000FU)            /*!<Fraction of USARTDIV */
#define  USART_BRR_DIV_Mantissa              (0xFFF0U)            /*!<Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_SBK                       (0x0001U)            /*!<Send Break                             */
#define  USART_CR1_RWU                       (0x0002U)            /*!<Receiver wakeup                        */
#define  USART_CR1_RE                        (0x0004U)            /*!<Receiver Enable                        */
#define  USART_CR1_TE                        (0x0008U)            /*!<Transmitter Enable                     */
#define  USART_CR1_IDLEIE                    (0x0010U)            /*!<IDLE Interrupt Enable                  */
#define  USART_CR1_RXNEIE                    (0x0020U)            /*!<RXNE Interrupt Enable                  */
#define  USART_CR1_TCIE                      (0x0040U)            /*!<Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     (0x0080U)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PEIE                      (0x0100U)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PS                        (0x0200U)            /*!<Parity Selection                       */
#define  USART_CR1_PCE                       (0x0400U)            /*!<Parity Control Enable                  */
#define  USART_CR1_WAKE                      (0x0800U)            /*!<Wakeup method                          */
#define  USART_CR1_M                         (0x1000U)            /*!<Word length                            */
#define  USART_CR1_UE                        (0x2000U)            /*!<USART Enable                           */
#define  USART_CR1_OVER8                     (0x8000U)            /*!<USART Oversampling by 8 enable         */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADD                       (0x000FU)            /*!<Address of the USART node            */
#define  USART_CR2_LBDL                      (0x0020U)            /*!<LIN Break Detection Length           */
#define  USART_CR2_LBDIE                     (0x0040U)            /*!<LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      (0x0100U)            /*!<Last Bit Clock pulse                 */
#define  USART_CR2_CPHA                      (0x0200U)            /*!<Clock Phase                          */
#define  USART_CR2_CPOL                      (0x0400U)            /*!<Clock Polarity                       */
#define  USART_CR2_CLKEN                     (0x0800U)            /*!<Clock Enable                         */

#define  USART_CR2_STOP                      (0x3000U)            /*!<STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    (0x1000U)            /*!<Bit 0 */
#define  USART_CR2_STOP_1                    (0x2000U)            /*!<Bit 1 */

#define  USART_CR2_LINEN                     (0x4000U)            /*!<LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       (0x0001U)            /*!<Error Interrupt Enable      */
#define  USART_CR3_IREN                      (0x0002U)            /*!<IrDA mode Enable            */
#define  USART_CR3_IRLP                      (0x0004U)            /*!<IrDA Low-Power              */
#define  USART_CR3_HDSEL                     (0x0008U)            /*!<Half-Duplex Selection       */
#define  USART_CR3_NACK                      (0x0010U)            /*!<Smartcard NACK enable       */
#define  USART_CR3_SCEN                      (0x0020U)            /*!<Smartcard mode enable       */
#define  USART_CR3_DMAR                      (0x0040U)            /*!<DMA Enable Receiver         */
#define  USART_CR3_DMAT                      (0x0080U)            /*!<DMA Enable Transmitter      */
#define  USART_CR3_RTSE                      (0x0100U)            /*!<RTS Enable                  */
#define  USART_CR3_CTSE                      (0x0200U)            /*!<CTS Enable                  */
#define  USART_CR3_CTSIE                     (0x0400U)            /*!<CTS Interrupt Enable        */
#define  USART_CR3_ONEBIT                    (0x0800U)            /*!<USART One bit method enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      (0x00FFU)            /*!<PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_PSC_0                    (0x0001U)            /*!<Bit 0 */
#define  USART_GTPR_PSC_1                    (0x0002U)            /*!<Bit 1 */
#define  USART_GTPR_PSC_2                    (0x0004U)            /*!<Bit 2 */
#define  USART_GTPR_PSC_3                    (0x0008U)            /*!<Bit 3 */
#define  USART_GTPR_PSC_4                    (0x0010U)            /*!<Bit 4 */
#define  USART_GTPR_PSC_5                    (0x0020U)            /*!<Bit 5 */
#define  USART_GTPR_PSC_6                    (0x0040U)            /*!<Bit 6 */
#define  USART_GTPR_PSC_7                    (0x0080U)            /*!<Bit 7 */

#define  USART_GTPR_GT                       (0xFF00U)            /*!<Guard time value */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           (0x7FU)               /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T_0                         (0x01U)               /*!<Bit 0 */
#define  WWDG_CR_T_1                         (0x02U)               /*!<Bit 1 */
#define  WWDG_CR_T_2                         (0x04U)               /*!<Bit 2 */
#define  WWDG_CR_T_3                         (0x08U)               /*!<Bit 3 */
#define  WWDG_CR_T_4                         (0x10U)               /*!<Bit 4 */
#define  WWDG_CR_T_5                         (0x20U)               /*!<Bit 5 */
#define  WWDG_CR_T_6                         (0x40U)               /*!<Bit 6 */
/* Legacy defines */
#define  WWDG_CR_T0                          WWDG_CR_T_0
#define  WWDG_CR_T1                          WWDG_CR_T_1
#define  WWDG_CR_T2                          WWDG_CR_T_2
#define  WWDG_CR_T3                          WWDG_CR_T_3
#define  WWDG_CR_T4                          WWDG_CR_T_4
#define  WWDG_CR_T5                          WWDG_CR_T_5
#define  WWDG_CR_T6                          WWDG_CR_T_6

#define  WWDG_CR_WDGA                        (0x80U)               /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          (0x007FU)            /*!<W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W_0                        (0x0001U)            /*!<Bit 0 */
#define  WWDG_CFR_W_1                        (0x0002U)            /*!<Bit 1 */
#define  WWDG_CFR_W_2                        (0x0004U)            /*!<Bit 2 */
#define  WWDG_CFR_W_3                        (0x0008U)            /*!<Bit 3 */
#define  WWDG_CFR_W_4                        (0x0010U)            /*!<Bit 4 */
#define  WWDG_CFR_W_5                        (0x0020U)            /*!<Bit 5 */
#define  WWDG_CFR_W_6                        (0x0040U)            /*!<Bit 6 */
/* Legacy defines */
#define  WWDG_CFR_W0                         WWDG_CFR_W_0
#define  WWDG_CFR_W1                         WWDG_CFR_W_1
#define  WWDG_CFR_W2                         WWDG_CFR_W_2
#define  WWDG_CFR_W3                         WWDG_CFR_W_3
#define  WWDG_CFR_W4                         WWDG_CFR_W_4
#define  WWDG_CFR_W5                         WWDG_CFR_W_5
#define  WWDG_CFR_W6                         WWDG_CFR_W_6

#define  WWDG_CFR_WDGTB                      (0x0180U)            /*!<WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB_0                    (0x0080U)            /*!<Bit 0 */
#define  WWDG_CFR_WDGTB_1                    (0x0100U)            /*!<Bit 1 */
/* Legacy defines */
#define  WWDG_CFR_WDGTB0                     WWDG_CFR_WDGTB_0
#define  WWDG_CFR_WDGTB1                     WWDG_CFR_WDGTB_1

#define  WWDG_CFR_EWI                        (0x0200U)            /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        (0x01U)               /*!<Early Wakeup Interrupt Flag */


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
#define  DBGMCU_CR_TRACE_MODE_0              (0x00000040UL)
#define  DBGMCU_CR_TRACE_MODE_1              (0x00000080UL)

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

/********************  Bit definition for DBGMCU_APB1_FZ register  ************/
#define  DBGMCU_APB1_FZ_DBG_TIM1_STOP        (0x00000001UL)
#define  DBGMCU_APB1_FZ_DBG_TIM8_STOP        (0x00000002UL)
#define  DBGMCU_APB1_FZ_DBG_TIM9_STOP        (0x00010000UL)
#define  DBGMCU_APB1_FZ_DBG_TIM10_STOP       (0x00020000UL)
#define  DBGMCU_APB1_FZ_DBG_TIM11_STOP       (0x00040000UL)

/******************************************************************************/
/*                                                                            */
/*                Ethernet MAC Registers bits definitions                     */
/*                                                                            */
/******************************************************************************/
/* Bit definition for Ethernet MAC Control Register register */
#define ETH_MACCR_WD      (0x00800000UL)  /* Watchdog disable */
#define ETH_MACCR_JD      (0x00400000UL)  /* Jabber disable */
#define ETH_MACCR_IFG     (0x000E0000UL)  /* Inter-frame gap */
#define ETH_MACCR_IFG_96Bit     (0x00000000UL)  /* Minimum IFG between frames during transmission is 96Bit */
#define ETH_MACCR_IFG_88Bit     (0x00020000UL)  /* Minimum IFG between frames during transmission is 88Bit */
#define ETH_MACCR_IFG_80Bit     (0x00040000UL)  /* Minimum IFG between frames during transmission is 80Bit */
#define ETH_MACCR_IFG_72Bit     (0x00060000UL)  /* Minimum IFG between frames during transmission is 72Bit */
#define ETH_MACCR_IFG_64Bit     (0x00080000UL)  /* Minimum IFG between frames during transmission is 64Bit */
#define ETH_MACCR_IFG_56Bit     (0x000A0000UL)  /* Minimum IFG between frames during transmission is 56Bit */
#define ETH_MACCR_IFG_48Bit     (0x000C0000UL)  /* Minimum IFG between frames during transmission is 48Bit */
#define ETH_MACCR_IFG_40Bit     (0x000E0000UL)  /* Minimum IFG between frames during transmission is 40Bit */
#define ETH_MACCR_CSD     (0x00010000UL)  /* Carrier sense disable (during transmission) */
#define ETH_MACCR_FES     (0x00004000UL)  /* Fast ethernet speed */
#define ETH_MACCR_ROD     (0x00002000UL)  /* Receive own disable */
#define ETH_MACCR_LM      (0x00001000UL)  /* loopback mode */
#define ETH_MACCR_DM      (0x00000800UL)  /* Duplex mode */
#define ETH_MACCR_IPCO    (0x00000400UL)  /* IP Checksum offload */
#define ETH_MACCR_RD      (0x00000200UL)  /* Retry disable */
#define ETH_MACCR_APCS    (0x00000080UL)  /* Automatic Pad/CRC stripping */
#define ETH_MACCR_BL      (0x00000060UL)  /* Back-off limit: random integer number (r) of slot time delays before rescheduling
                                                       a transmission attempt during retries after a collision: 0 =< r <2^k */
#define ETH_MACCR_BL_10    (0x00000000UL)  /* k = min (n, 10) */
#define ETH_MACCR_BL_8     (0x00000020UL)  /* k = min (n, 8) */
#define ETH_MACCR_BL_4     (0x00000040UL)  /* k = min (n, 4) */
#define ETH_MACCR_BL_1     (0x00000060UL)  /* k = min (n, 1) */
#define ETH_MACCR_DC      (0x00000010UL)  /* Defferal check */
#define ETH_MACCR_TE      (0x00000008UL)  /* Transmitter enable */
#define ETH_MACCR_RE      (0x00000004UL)  /* Receiver enable */

/* Bit definition for Ethernet MAC Frame Filter Register */
#define ETH_MACFFR_RA     (0x80000000UL)  /* Receive all */
#define ETH_MACFFR_HPF    (0x00000400UL)  /* Hash or perfect filter */
#define ETH_MACFFR_SAF    (0x00000200UL)  /* Source address filter enable */
#define ETH_MACFFR_SAIF   (0x00000100UL)  /* SA inverse filtering */
#define ETH_MACFFR_PCF    (0x000000C0UL)  /* Pass control frames: 3 cases */
#define ETH_MACFFR_PCF_BlockAll                (0x00000040UL)  /* MAC filters all control frames from reaching the application */
#define ETH_MACFFR_PCF_ForwardAll              (0x00000080UL)  /* MAC forwards all control frames to application even if they fail the Address Filter */
#define ETH_MACFFR_PCF_ForwardPassedAddrFilter (0x000000C0UL)  /* MAC forwards control frames that pass the Address Filter. */
#define ETH_MACFFR_BFD    (0x00000020UL)  /* Broadcast frame disable */
#define ETH_MACFFR_PAM    (0x00000010UL)  /* Pass all mutlicast */
#define ETH_MACFFR_DAIF   (0x00000008UL)  /* DA Inverse filtering */
#define ETH_MACFFR_HM     (0x00000004UL)  /* Hash multicast */
#define ETH_MACFFR_HU     (0x00000002UL)  /* Hash unicast */
#define ETH_MACFFR_PM     (0x00000001UL)  /* Promiscuous mode */

/* Bit definition for Ethernet MAC Hash Table High Register */
#define ETH_MACHTHR_HTH   (0xFFFFFFFFUL)  /* Hash table high */

/* Bit definition for Ethernet MAC Hash Table Low Register */
#define ETH_MACHTLR_HTL   (0xFFFFFFFFUL)  /* Hash table low */

/* Bit definition for Ethernet MAC MII Address Register */
#define ETH_MACMIIAR_PA   (0x0000F800UL)  /* Physical layer address */
#define ETH_MACMIIAR_MR   (0x000007C0UL)  /* MII register in the selected PHY */
#define ETH_MACMIIAR_CR   (0x0000001CUL)  /* CR clock range: 6 cases */
#define ETH_MACMIIAR_CR_Div42   (0x00000000UL)  /* HCLK:60-100 MHz; MDC clock= HCLK/42 */
#define ETH_MACMIIAR_CR_Div62   (0x00000004UL)  /* HCLK:100-150 MHz; MDC clock= HCLK/62 */
#define ETH_MACMIIAR_CR_Div16   (0x00000008UL)  /* HCLK:20-35 MHz; MDC clock= HCLK/16 */
#define ETH_MACMIIAR_CR_Div26   (0x0000000CUL)  /* HCLK:35-60 MHz; MDC clock= HCLK/26 */
#define ETH_MACMIIAR_CR_Div102  (0x00000010UL)  /* HCLK:150-168 MHz; MDC clock= HCLK/102 */
#define ETH_MACMIIAR_MW   (0x00000002UL)  /* MII write */
#define ETH_MACMIIAR_MB   (0x00000001UL)  /* MII busy */

/* Bit definition for Ethernet MAC MII Data Register */
#define ETH_MACMIIDR_MD   (0x0000FFFFUL)  /* MII data: read/write data from/to PHY */

/* Bit definition for Ethernet MAC Flow Control Register */
#define ETH_MACFCR_PT     (0xFFFF0000UL)  /* Pause time */
#define ETH_MACFCR_ZQPD   (0x00000080UL)  /* Zero-quanta pause disable */
#define ETH_MACFCR_PLT    (0x00000030UL)  /* Pause low threshold: 4 cases */
#define ETH_MACFCR_PLT_Minus4   (0x00000000UL)  /* Pause time minus 4 slot times */
#define ETH_MACFCR_PLT_Minus28  (0x00000010UL)  /* Pause time minus 28 slot times */
#define ETH_MACFCR_PLT_Minus144 (0x00000020UL)  /* Pause time minus 144 slot times */
#define ETH_MACFCR_PLT_Minus256 (0x00000030UL)  /* Pause time minus 256 slot times */
#define ETH_MACFCR_UPFD   (0x00000008UL)  /* Unicast pause frame detect */
#define ETH_MACFCR_RFCE   (0x00000004UL)  /* Receive flow control enable */
#define ETH_MACFCR_TFCE   (0x00000002UL)  /* Transmit flow control enable */
#define ETH_MACFCR_FCBBPA (0x00000001UL)  /* Flow control busy/backpressure activate */

/* Bit definition for Ethernet MAC VLAN Tag Register */
#define ETH_MACVLANTR_VLANTC (0x00010000UL)  /* 12-bit VLAN tag comparison */
#define ETH_MACVLANTR_VLANTI (0x0000FFFFUL)  /* VLAN tag identifier (for receive frames) */

/* Bit definition for Ethernet MAC Remote Wake-UpFrame Filter Register */
#define ETH_MACRWUFFR_D   (0xFFFFFFFFUL)  /* Wake-up frame filter register data */
/* Eight sequential Writes to this address (offset 0x28) will write all Wake-UpFrame Filter Registers.
   Eight sequential Reads from this address (offset 0x28) will read all Wake-UpFrame Filter Registers. */
/* Wake-UpFrame Filter Reg0 : Filter 0 Byte Mask
   Wake-UpFrame Filter Reg1 : Filter 1 Byte Mask
   Wake-UpFrame Filter Reg2 : Filter 2 Byte Mask
   Wake-UpFrame Filter Reg3 : Filter 3 Byte Mask
   Wake-UpFrame Filter Reg4 : RSVD - Filter3 Command - RSVD - Filter2 Command -
                              RSVD - Filter1 Command - RSVD - Filter0 Command
   Wake-UpFrame Filter Re5 : Filter3 Offset - Filter2 Offset - Filter1 Offset - Filter0 Offset
   Wake-UpFrame Filter Re6 : Filter1 CRC16 - Filter0 CRC16
   Wake-UpFrame Filter Re7 : Filter3 CRC16 - Filter2 CRC16 */

/* Bit definition for Ethernet MAC PMT Control and Status Register */
#define ETH_MACPMTCSR_WFFRPR (0x80000000UL)  /* Wake-Up Frame Filter Register Pointer Reset */
#define ETH_MACPMTCSR_GU     (0x00000200UL)  /* Global Unicast */
#define ETH_MACPMTCSR_WFR    (0x00000040UL)  /* Wake-Up Frame Received */
#define ETH_MACPMTCSR_MPR    (0x00000020UL)  /* Magic Packet Received */
#define ETH_MACPMTCSR_WFE    (0x00000004UL)  /* Wake-Up Frame Enable */
#define ETH_MACPMTCSR_MPE    (0x00000002UL)  /* Magic Packet Enable */
#define ETH_MACPMTCSR_PD     (0x00000001UL)  /* Power Down */

/* Bit definition for Ethernet MAC Status Register */
#define ETH_MACSR_TSTS      (0x00000200UL)  /* Time stamp trigger status */
#define ETH_MACSR_MMCTS     (0x00000040UL)  /* MMC transmit status */
#define ETH_MACSR_MMMCRS    (0x00000020UL)  /* MMC receive status */
#define ETH_MACSR_MMCS      (0x00000010UL)  /* MMC status */
#define ETH_MACSR_PMTS      (0x00000008UL)  /* PMT status */

/* Bit definition for Ethernet MAC Interrupt Mask Register */
#define ETH_MACIMR_TSTIM     (0x00000200UL)  /* Time stamp trigger interrupt mask */
#define ETH_MACIMR_PMTIM     (0x00000008UL)  /* PMT interrupt mask */

/* Bit definition for Ethernet MAC Address0 High Register */
#define ETH_MACA0HR_MACA0H   (0x0000FFFFUL)  /* MAC address0 high */

/* Bit definition for Ethernet MAC Address0 Low Register */
#define ETH_MACA0LR_MACA0L   (0xFFFFFFFFUL)  /* MAC address0 low */

/* Bit definition for Ethernet MAC Address1 High Register */
#define ETH_MACA1HR_AE       (0x80000000UL)  /* Address enable */
#define ETH_MACA1HR_SA       (0x40000000UL)  /* Source address */
#define ETH_MACA1HR_MBC      (0x3F000000UL)  /* Mask byte control: bits to mask for comparison of the MAC Address bytes */
#define ETH_MACA1HR_MBC_HBits15_8    (0x20000000UL)  /* Mask MAC Address high reg bits [15:8] */
#define ETH_MACA1HR_MBC_HBits7_0     (0x10000000UL)  /* Mask MAC Address high reg bits [7:0] */
#define ETH_MACA1HR_MBC_LBits31_24   (0x08000000UL)  /* Mask MAC Address low reg bits [31:24] */
#define ETH_MACA1HR_MBC_LBits23_16   (0x04000000UL)  /* Mask MAC Address low reg bits [23:16] */
#define ETH_MACA1HR_MBC_LBits15_8    (0x02000000UL)  /* Mask MAC Address low reg bits [15:8] */
#define ETH_MACA1HR_MBC_LBits7_0     (0x01000000UL)  /* Mask MAC Address low reg bits [7:0] */
#define ETH_MACA1HR_MACA1H   (0x0000FFFFUL)  /* MAC address1 high */

/* Bit definition for Ethernet MAC Address1 Low Register */
#define ETH_MACA1LR_MACA1L   (0xFFFFFFFFUL)  /* MAC address1 low */

/* Bit definition for Ethernet MAC Address2 High Register */
#define ETH_MACA2HR_AE       (0x80000000UL)  /* Address enable */
#define ETH_MACA2HR_SA       (0x40000000UL)  /* Source address */
#define ETH_MACA2HR_MBC      (0x3F000000UL)  /* Mask byte control */
#define ETH_MACA2HR_MBC_HBits15_8    (0x20000000UL)  /* Mask MAC Address high reg bits [15:8] */
#define ETH_MACA2HR_MBC_HBits7_0     (0x10000000UL)  /* Mask MAC Address high reg bits [7:0] */
#define ETH_MACA2HR_MBC_LBits31_24   (0x08000000UL)  /* Mask MAC Address low reg bits [31:24] */
#define ETH_MACA2HR_MBC_LBits23_16   (0x04000000UL)  /* Mask MAC Address low reg bits [23:16] */
#define ETH_MACA2HR_MBC_LBits15_8    (0x02000000UL)  /* Mask MAC Address low reg bits [15:8] */
#define ETH_MACA2HR_MBC_LBits7_0     (0x01000000UL)  /* Mask MAC Address low reg bits [70] */
#define ETH_MACA2HR_MACA2H   (0x0000FFFFUL)  /* MAC address1 high */

/* Bit definition for Ethernet MAC Address2 Low Register */
#define ETH_MACA2LR_MACA2L   (0xFFFFFFFFUL)  /* MAC address2 low */

/* Bit definition for Ethernet MAC Address3 High Register */
#define ETH_MACA3HR_AE       (0x80000000UL)  /* Address enable */
#define ETH_MACA3HR_SA       (0x40000000UL)  /* Source address */
#define ETH_MACA3HR_MBC      (0x3F000000UL)  /* Mask byte control */
#define ETH_MACA3HR_MBC_HBits15_8    (0x20000000UL)  /* Mask MAC Address high reg bits [15:8] */
#define ETH_MACA3HR_MBC_HBits7_0     (0x10000000UL)  /* Mask MAC Address high reg bits [7:0] */
#define ETH_MACA3HR_MBC_LBits31_24   (0x08000000UL)  /* Mask MAC Address low reg bits [31:24] */
#define ETH_MACA3HR_MBC_LBits23_16   (0x04000000UL)  /* Mask MAC Address low reg bits [23:16] */
#define ETH_MACA3HR_MBC_LBits15_8    (0x02000000UL)  /* Mask MAC Address low reg bits [15:8] */
#define ETH_MACA3HR_MBC_LBits7_0     (0x01000000UL)  /* Mask MAC Address low reg bits [70] */
#define ETH_MACA3HR_MACA3H   (0x0000FFFFUL)  /* MAC address3 high */

/* Bit definition for Ethernet MAC Address3 Low Register */
#define ETH_MACA3LR_MACA3L   (0xFFFFFFFFUL)  /* MAC address3 low */

/******************************************************************************/
/*                Ethernet MMC Registers bits definition                      */
/******************************************************************************/

/* Bit definition for Ethernet MMC Contol Register */
#define ETH_MMCCR_MCFHP      (0x00000020UL)  /* MMC counter Full-Half preset */
#define ETH_MMCCR_MCP        (0x00000010UL)  /* MMC counter preset */
#define ETH_MMCCR_MCF        (0x00000008UL)  /* MMC Counter Freeze */
#define ETH_MMCCR_ROR        (0x00000004UL)  /* Reset on Read */
#define ETH_MMCCR_CSR        (0x00000002UL)  /* Counter Stop Rollover */
#define ETH_MMCCR_CR         (0x00000001UL)  /* Counters Reset */

/* Bit definition for Ethernet MMC Receive Interrupt Register */
#define ETH_MMCRIR_RGUFS     (0x00020000UL)  /* Set when Rx good unicast frames counter reaches half the maximum value */
#define ETH_MMCRIR_RFAES     (0x00000040UL)  /* Set when Rx alignment error counter reaches half the maximum value */
#define ETH_MMCRIR_RFCES     (0x00000020UL)  /* Set when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Register */
#define ETH_MMCTIR_TGFS      (0x00200000UL)  /* Set when Tx good frame count counter reaches half the maximum value */
#define ETH_MMCTIR_TGFMSCS   (0x00008000UL)  /* Set when Tx good multi col counter reaches half the maximum value */
#define ETH_MMCTIR_TGFSCS    (0x00004000UL)  /* Set when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Receive Interrupt Mask Register */
#define ETH_MMCRIMR_RGUFM    (0x00020000UL)  /* Mask the interrupt when Rx good unicast frames counter reaches half the maximum value */
#define ETH_MMCRIMR_RFAEM    (0x00000040UL)  /* Mask the interrupt when when Rx alignment error counter reaches half the maximum value */
#define ETH_MMCRIMR_RFCEM    (0x00000020UL)  /* Mask the interrupt when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Mask Register */
#define ETH_MMCTIMR_TGFM     (0x00200000UL)  /* Mask the interrupt when Tx good frame count counter reaches half the maximum value */
#define ETH_MMCTIMR_TGFMSCM  (0x00008000UL)  /* Mask the interrupt when Tx good multi col counter reaches half the maximum value */
#define ETH_MMCTIMR_TGFSCM   (0x00004000UL)  /* Mask the interrupt when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmitted Good Frames after Single Collision Counter Register */
#define ETH_MMCTGFSCCR_TGFSCC     (0xFFFFFFFFUL)  /* Number of successfully transmitted frames after a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames after More than a Single Collision Counter Register */
#define ETH_MMCTGFMSCCR_TGFMSCC   (0xFFFFFFFFUL)  /* Number of successfully transmitted frames after more than a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames Counter Register */
#define ETH_MMCTGFCR_TGFC    (0xFFFFFFFFUL)  /* Number of good frames transmitted. */

/* Bit definition for Ethernet MMC Received Frames with CRC Error Counter Register */
#define ETH_MMCRFCECR_RFCEC  (0xFFFFFFFFUL)  /* Number of frames received with CRC error. */

/* Bit definition for Ethernet MMC Received Frames with Alignement Error Counter Register */
#define ETH_MMCRFAECR_RFAEC  (0xFFFFFFFFUL)  /* Number of frames received with alignment (dribble) error */

/* Bit definition for Ethernet MMC Received Good Unicast Frames Counter Register */
#define ETH_MMCRGUFCR_RGUFC  (0xFFFFFFFFUL)  /* Number of good unicast frames received. */

/******************************************************************************/
/*               Ethernet PTP Registers bits definition                       */
/******************************************************************************/

/* Bit definition for Ethernet PTP Time Stamp Contol Register */
#define ETH_PTPTSCR_TSCNT       (0x00030000UL)  /* Time stamp clock node type */
#define ETH_PTPTSSR_TSSMRME     (0x00008000UL)  /* Time stamp snapshot for message relevant to master enable */
#define ETH_PTPTSSR_TSSEME      (0x00004000UL)  /* Time stamp snapshot for event message enable */
#define ETH_PTPTSSR_TSSIPV4FE   (0x00002000UL)  /* Time stamp snapshot for IPv4 frames enable */
#define ETH_PTPTSSR_TSSIPV6FE   (0x00001000UL)  /* Time stamp snapshot for IPv6 frames enable */
#define ETH_PTPTSSR_TSSPTPOEFE  (0x00000800UL)  /* Time stamp snapshot for PTP over ethernet frames enable */
#define ETH_PTPTSSR_TSPTPPSV2E  (0x00000400UL)  /* Time stamp PTP packet snooping for version2 format enable */
#define ETH_PTPTSSR_TSSSR       (0x00000200UL)  /* Time stamp Sub-seconds rollover */
#define ETH_PTPTSSR_TSSARFE     (0x00000100UL)  /* Time stamp snapshot for all received frames enable */

#define ETH_PTPTSCR_TSARU    (0x00000020UL)  /* Addend register update */
#define ETH_PTPTSCR_TSITE    (0x00000010UL)  /* Time stamp interrupt trigger enable */
#define ETH_PTPTSCR_TSSTU    (0x00000008UL)  /* Time stamp update */
#define ETH_PTPTSCR_TSSTI    (0x00000004UL)  /* Time stamp initialize */
#define ETH_PTPTSCR_TSFCU    (0x00000002UL)  /* Time stamp fine or coarse update */
#define ETH_PTPTSCR_TSE      (0x00000001UL)  /* Time stamp enable */

/* Bit definition for Ethernet PTP Sub-Second Increment Register */
#define ETH_PTPSSIR_STSSI    (0x000000FFUL)  /* System time Sub-second increment value */

/* Bit definition for Ethernet PTP Time Stamp High Register */
#define ETH_PTPTSHR_STS      (0xFFFFFFFFUL)  /* System Time second */

/* Bit definition for Ethernet PTP Time Stamp Low Register */
#define ETH_PTPTSLR_STPNS    (0x80000000UL)  /* System Time Positive or negative time */
#define ETH_PTPTSLR_STSS     (0x7FFFFFFFUL)  /* System Time sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp High Update Register */
#define ETH_PTPTSHUR_TSUS    (0xFFFFFFFFUL)  /* Time stamp update seconds */

/* Bit definition for Ethernet PTP Time Stamp Low Update Register */
#define ETH_PTPTSLUR_TSUPNS  (0x80000000UL)  /* Time stamp update Positive or negative time */
#define ETH_PTPTSLUR_TSUSS   (0x7FFFFFFFUL)  /* Time stamp update sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp Addend Register */
#define ETH_PTPTSAR_TSA      (0xFFFFFFFFUL)  /* Time stamp addend */

/* Bit definition for Ethernet PTP Target Time High Register */
#define ETH_PTPTTHR_TTSH     (0xFFFFFFFFUL)  /* Target time stamp high */

/* Bit definition for Ethernet PTP Target Time Low Register */
#define ETH_PTPTTLR_TTSL     (0xFFFFFFFFUL)  /* Target time stamp low */

/* Bit definition for Ethernet PTP Time Stamp Status Register */
#define ETH_PTPTSSR_TSTTR    (0x00000020UL)  /* Time stamp target time reached */
#define ETH_PTPTSSR_TSSO     (0x00000010UL)  /* Time stamp seconds overflow */

/******************************************************************************/
/*                 Ethernet DMA Registers bits definition                     */
/******************************************************************************/

/* Bit definition for Ethernet DMA Bus Mode Register */
#define ETH_DMABMR_AAB       (0x02000000UL)  /* Address-Aligned beats */
#define ETH_DMABMR_FPM        (0x01000000UL)  /* 4xPBL mode */
#define ETH_DMABMR_USP       (0x00800000UL)  /* Use separate PBL */
#define ETH_DMABMR_RDP       (0x007E0000UL)  /* RxDMA PBL */
#define ETH_DMABMR_RDP_1Beat    (0x00020000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 1 */
#define ETH_DMABMR_RDP_2Beat    (0x00040000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 2 */
#define ETH_DMABMR_RDP_4Beat    (0x00080000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
#define ETH_DMABMR_RDP_8Beat    (0x00100000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
#define ETH_DMABMR_RDP_16Beat   (0x00200000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
#define ETH_DMABMR_RDP_32Beat   (0x00400000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 32 */
#define ETH_DMABMR_RDP_4xPBL_4Beat   (0x01020000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
#define ETH_DMABMR_RDP_4xPBL_8Beat   (0x01040000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
#define ETH_DMABMR_RDP_4xPBL_16Beat  (0x01080000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
#define ETH_DMABMR_RDP_4xPBL_32Beat  (0x01100000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 32 */
#define ETH_DMABMR_RDP_4xPBL_64Beat  (0x01200000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 64 */
#define ETH_DMABMR_RDP_4xPBL_128Beat (0x01400000UL)  /* maximum number of beats to be transferred in one RxDMA transaction is 128 */
#define ETH_DMABMR_FB        (0x00010000UL)  /* Fixed Burst */
#define ETH_DMABMR_RTPR      (0x0000C000UL)  /* Rx Tx priority ratio */
#define ETH_DMABMR_RTPR_1_1     (0x00000000UL)  /* Rx Tx priority ratio */
#define ETH_DMABMR_RTPR_2_1     (0x00004000UL)  /* Rx Tx priority ratio */
#define ETH_DMABMR_RTPR_3_1     (0x00008000UL)  /* Rx Tx priority ratio */
#define ETH_DMABMR_RTPR_4_1     (0x0000C000UL)  /* Rx Tx priority ratio */
#define ETH_DMABMR_PBL    (0x00003F00UL)  /* Programmable burst length */
#define ETH_DMABMR_PBL_1Beat    (0x00000100UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 1 */
#define ETH_DMABMR_PBL_2Beat    (0x00000200UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 2 */
#define ETH_DMABMR_PBL_4Beat    (0x00000400UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
#define ETH_DMABMR_PBL_8Beat    (0x00000800UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
#define ETH_DMABMR_PBL_16Beat   (0x00001000UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
#define ETH_DMABMR_PBL_32Beat   (0x00002000UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
#define ETH_DMABMR_PBL_4xPBL_4Beat   (0x01000100UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
#define ETH_DMABMR_PBL_4xPBL_8Beat   (0x01000200UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
#define ETH_DMABMR_PBL_4xPBL_16Beat  (0x01000400UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
#define ETH_DMABMR_PBL_4xPBL_32Beat  (0x01000800UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
#define ETH_DMABMR_PBL_4xPBL_64Beat  (0x01001000UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 64 */
#define ETH_DMABMR_PBL_4xPBL_128Beat (0x01002000UL)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 128 */
#define ETH_DMABMR_EDE       (0x00000080UL)  /* Enhanced Descriptor Enable */
#define ETH_DMABMR_DSL       (0x0000007CUL)  /* Descriptor Skip Length */
#define ETH_DMABMR_DA        (0x00000002UL)  /* DMA arbitration scheme */
#define ETH_DMABMR_SR        (0x00000001UL)  /* Software reset */

/* Bit definition for Ethernet DMA Transmit Poll Demand Register */
#define ETH_DMATPDR_TPD      (0xFFFFFFFFUL)  /* Transmit poll demand */

/* Bit definition for Ethernet DMA Receive Poll Demand Register */
#define ETH_DMARPDR_RPD      (0xFFFFFFFFUL)  /* Receive poll demand  */

/* Bit definition for Ethernet DMA Receive Descriptor List Address Register */
#define ETH_DMARDLAR_SRL     (0xFFFFFFFFUL)  /* Start of receive list */

/* Bit definition for Ethernet DMA Transmit Descriptor List Address Register */
#define ETH_DMATDLAR_STL     (0xFFFFFFFFUL)  /* Start of transmit list */

/* Bit definition for Ethernet DMA Status Register */
#define ETH_DMASR_TSTS       (0x20000000UL)  /* Time-stamp trigger status */
#define ETH_DMASR_PMTS       (0x10000000UL)  /* PMT status */
#define ETH_DMASR_MMCS       (0x08000000UL)  /* MMC status */
#define ETH_DMASR_EBS        (0x03800000UL)  /* Error bits status */
  /* combination with EBS[2:0] for GetFlagStatus function */
#define ETH_DMASR_EBS_DescAccess      (0x02000000UL)  /* Error bits 0-data buffer, 1-desc. access */
#define ETH_DMASR_EBS_ReadTransf      (0x01000000UL)  /* Error bits 0-write trnsf, 1-read transfr */
#define ETH_DMASR_EBS_DataTransfTx    (0x00800000UL)  /* Error bits 0-Rx DMA, 1-Tx DMA */
#define ETH_DMASR_TPS         (0x00700000UL)  /* Transmit process state */
#define ETH_DMASR_TPS_Stopped         (0x00000000UL)  /* Stopped - Reset or Stop Tx Command issued  */
#define ETH_DMASR_TPS_Fetching        (0x00100000UL)  /* Running - fetching the Tx descriptor */
#define ETH_DMASR_TPS_Waiting         (0x00200000UL)  /* Running - waiting for status */
#define ETH_DMASR_TPS_Reading         (0x00300000UL)  /* Running - reading the data from host memory */
#define ETH_DMASR_TPS_Suspended       (0x00600000UL)  /* Suspended - Tx Descriptor unavailabe */
#define ETH_DMASR_TPS_Closing         (0x00700000UL)  /* Running - closing Rx descriptor */
#define ETH_DMASR_RPS         (0x000E0000UL)  /* Receive process state */
#define ETH_DMASR_RPS_Stopped         (0x00000000UL)  /* Stopped - Reset or Stop Rx Command issued */
#define ETH_DMASR_RPS_Fetching        (0x00020000UL)  /* Running - fetching the Rx descriptor */
#define ETH_DMASR_RPS_Waiting         (0x00060000UL)  /* Running - waiting for packet */
#define ETH_DMASR_RPS_Suspended       (0x00080000UL)  /* Suspended - Rx Descriptor unavailable */
#define ETH_DMASR_RPS_Closing         (0x000A0000UL)  /* Running - closing descriptor */
#define ETH_DMASR_RPS_Queuing         (0x000E0000UL)  /* Running - queuing the recieve frame into host memory */
#define ETH_DMASR_NIS        (0x00010000UL)  /* Normal interrupt summary */
#define ETH_DMASR_AIS        (0x00008000UL)  /* Abnormal interrupt summary */
#define ETH_DMASR_ERS        (0x00004000UL)  /* Early receive status */
#define ETH_DMASR_FBES       (0x00002000UL)  /* Fatal bus error status */
#define ETH_DMASR_ETS        (0x00000400UL)  /* Early transmit status */
#define ETH_DMASR_RWTS       (0x00000200UL)  /* Receive watchdog timeout status */
#define ETH_DMASR_RPSS       (0x00000100UL)  /* Receive process stopped status */
#define ETH_DMASR_RBUS       (0x00000080UL)  /* Receive buffer unavailable status */
#define ETH_DMASR_RS         (0x00000040UL)  /* Receive status */
#define ETH_DMASR_TUS        (0x00000020UL)  /* Transmit underflow status */
#define ETH_DMASR_ROS        (0x00000010UL)  /* Receive overflow status */
#define ETH_DMASR_TJTS       (0x00000008UL)  /* Transmit jabber timeout status */
#define ETH_DMASR_TBUS       (0x00000004UL)  /* Transmit buffer unavailable status */
#define ETH_DMASR_TPSS       (0x00000002UL)  /* Transmit process stopped status */
#define ETH_DMASR_TS         (0x00000001UL)  /* Transmit status */

/* Bit definition for Ethernet DMA Operation Mode Register */
#define ETH_DMAOMR_DTCEFD    (0x04000000UL)  /* Disable Dropping of TCP/IP checksum error frames */
#define ETH_DMAOMR_RSF       (0x02000000UL)  /* Receive store and forward */
#define ETH_DMAOMR_DFRF      (0x01000000UL)  /* Disable flushing of received frames */
#define ETH_DMAOMR_TSF       (0x00200000UL)  /* Transmit store and forward */
#define ETH_DMAOMR_FTF       (0x00100000UL)  /* Flush transmit FIFO */
#define ETH_DMAOMR_TTC       (0x0001C000UL)  /* Transmit threshold control */
#define ETH_DMAOMR_TTC_64Bytes       (0x00000000UL)  /* threshold level of the MTL Transmit FIFO is 64 Bytes */
#define ETH_DMAOMR_TTC_128Bytes      (0x00004000UL)  /* threshold level of the MTL Transmit FIFO is 128 Bytes */
#define ETH_DMAOMR_TTC_192Bytes      (0x00008000UL)  /* threshold level of the MTL Transmit FIFO is 192 Bytes */
#define ETH_DMAOMR_TTC_256Bytes      (0x0000C000UL)  /* threshold level of the MTL Transmit FIFO is 256 Bytes */
#define ETH_DMAOMR_TTC_40Bytes       (0x00010000UL)  /* threshold level of the MTL Transmit FIFO is 40 Bytes */
#define ETH_DMAOMR_TTC_32Bytes       (0x00014000UL)  /* threshold level of the MTL Transmit FIFO is 32 Bytes */
#define ETH_DMAOMR_TTC_24Bytes       (0x00018000UL)  /* threshold level of the MTL Transmit FIFO is 24 Bytes */
#define ETH_DMAOMR_TTC_16Bytes       (0x0001C000UL)  /* threshold level of the MTL Transmit FIFO is 16 Bytes */
#define ETH_DMAOMR_ST        (0x00002000UL)  /* Start/stop transmission command */
#define ETH_DMAOMR_FEF       (0x00000080UL)  /* Forward error frames */
#define ETH_DMAOMR_FUGF      (0x00000040UL)  /* Forward undersized good frames */
#define ETH_DMAOMR_RTC       (0x00000018UL)  /* receive threshold control */
#define ETH_DMAOMR_RTC_64Bytes       (0x00000000UL)  /* threshold level of the MTL Receive FIFO is 64 Bytes */
#define ETH_DMAOMR_RTC_32Bytes       (0x00000008UL)  /* threshold level of the MTL Receive FIFO is 32 Bytes */
#define ETH_DMAOMR_RTC_96Bytes       (0x00000010UL)  /* threshold level of the MTL Receive FIFO is 96 Bytes */
#define ETH_DMAOMR_RTC_128Bytes      (0x00000018UL)  /* threshold level of the MTL Receive FIFO is 128 Bytes */
#define ETH_DMAOMR_OSF       (0x00000004UL)  /* operate on second frame */
#define ETH_DMAOMR_SR        (0x00000002UL)  /* Start/stop receive */

/* Bit definition for Ethernet DMA Interrupt Enable Register */
#define ETH_DMAIER_NISE      (0x00010000UL)  /* Normal interrupt summary enable */
#define ETH_DMAIER_AISE      (0x00008000UL)  /* Abnormal interrupt summary enable */
#define ETH_DMAIER_ERIE      (0x00004000UL)  /* Early receive interrupt enable */
#define ETH_DMAIER_FBEIE     (0x00002000UL)  /* Fatal bus error interrupt enable */
#define ETH_DMAIER_ETIE      (0x00000400UL)  /* Early transmit interrupt enable */
#define ETH_DMAIER_RWTIE     (0x00000200UL)  /* Receive watchdog timeout interrupt enable */
#define ETH_DMAIER_RPSIE     (0x00000100UL)  /* Receive process stopped interrupt enable */
#define ETH_DMAIER_RBUIE     (0x00000080UL)  /* Receive buffer unavailable interrupt enable */
#define ETH_DMAIER_RIE       (0x00000040UL)  /* Receive interrupt enable */
#define ETH_DMAIER_TUIE      (0x00000020UL)  /* Transmit Underflow interrupt enable */
#define ETH_DMAIER_ROIE      (0x00000010UL)  /* Receive Overflow interrupt enable */
#define ETH_DMAIER_TJTIE     (0x00000008UL)  /* Transmit jabber timeout interrupt enable */
#define ETH_DMAIER_TBUIE     (0x00000004UL)  /* Transmit buffer unavailable interrupt enable */
#define ETH_DMAIER_TPSIE     (0x00000002UL)  /* Transmit process stopped interrupt enable */
#define ETH_DMAIER_TIE       (0x00000001UL)  /* Transmit interrupt enable */

/* Bit definition for Ethernet DMA Missed Frame and Buffer Overflow Counter Register */
#define ETH_DMAMFBOCR_OFOC   (0x10000000UL)  /* Overflow bit for FIFO overflow counter */
#define ETH_DMAMFBOCR_MFA    (0x0FFE0000UL)  /* Number of frames missed by the application */
#define ETH_DMAMFBOCR_OMFC   (0x00010000UL)  /* Overflow bit for missed frame counter */
#define ETH_DMAMFBOCR_MFC    (0x0000FFFFUL)  /* Number of frames missed by the controller */

/* Bit definition for Ethernet DMA Current Host Transmit Descriptor Register */
#define ETH_DMACHTDR_HTDAP   (0xFFFFFFFFUL)  /* Host transmit descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Descriptor Register */
#define ETH_DMACHRDR_HRDAP   (0xFFFFFFFFUL)  /* Host receive descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Transmit Buffer Address Register */
#define ETH_DMACHTBAR_HTBAP  (0xFFFFFFFFUL)  /* Host transmit buffer address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Buffer Address Register */
#define ETH_DMACHRBAR_HRBAP  (0xFFFFFFFFUL)  /* Host receive buffer address pointer */

/**
  *
  */

 /**
  * @}
  */

#ifdef USE_STDPERIPH_DRIVER
  #include "stm32f4xx_conf.h"
#endif /* USE_STDPERIPH_DRIVER */

/** @addtogroup Exported_macro
  * @{
  */

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F4xx_H */
