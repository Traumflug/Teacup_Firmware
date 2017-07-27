/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-June-2014
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  * This file configures the system clock as follows:
  *-----------------------------------------------------------------------------
  * System clock source                | 1- PLL_HSE_EXTC        | 3- PLL_HSI
  *                                    | (external 8 MHz clock) | (internal 16 MHz)
  *                                    | 2- PLL_HSE_XTAL        |
  *                                    | (external 8 MHz xtal)  |
  *-----------------------------------------------------------------------------
  * SYSCLK(MHz)                        | 96                     | 96
  *-----------------------------------------------------------------------------
  * AHBCLK (MHz)                       | 96                     | 96
  *-----------------------------------------------------------------------------
  * APB1CLK (MHz)                      | 48                     | 48
  *-----------------------------------------------------------------------------
  * APB2CLK (MHz)                      | 96                     | 96
  *-----------------------------------------------------------------------------
  * USB capable (48 MHz precise clock) | YES                    | YES
  *-----------------------------------------------------------------------------
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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

  Copied from $(MBED)/libraries/mbed/targets/cmsis/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F411RE/system_stm32f4xx.c.

  - Prefixed names of #include files with mbed- to match the names of the
    copies in the Teacup repo.
  - Wrapped the whole file in #ifdef __ARM_STM32__ to not cause conflicts with
    AVR builds.
  - Rebuild SystemInit() and SetSysClock() to get rid of most mbed-files. Please take a look into history.
  - Rework SetSysClock completely
*/

#ifdef __ARM_STM32__

#include "cmsis-stm32f4xx.h"
#include "arduino_stm32f411.h"

#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)8000000) /*!< Default value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x200. */
/******************************************************************************/

/* Select the clock sources (other than HSI) to start with (0=OFF, 1=ON) */
#define USE_PLL_HSE_EXTC (1) /* Use external clock */
#define USE_PLL_HSE_XTAL (1) /* Use external xtal */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory
  *         configuration.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= RCC_CR_HSION;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= ~(RCC_CR_HSEON |
               RCC_CR_CSSON |
               RCC_CR_PLLON);

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= ~(RCC_CR_HSEBYP);

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */

  /* Configure the System clock source, PLL Multiplier and Divider factors,
     AHB/APBx prescalers and Flash settings */
  SetSysClock();
}

/**
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file (its value
  *              depends on the application requirements), user has to ensure that HSE_VALUE
  *              is same as the real frequency of the crystal used. Otherwise, this function
  *              may have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case RCC_CFGR_SWS_HSI:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;
    case RCC_CFGR_SWS_HSE:  /* HSE used as system clock source */
      SystemCoreClock = HSE_VALUE;
      break;
    case RCC_CFGR_SWS_PLL:  /* PLL used as system clock source */

      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      
      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco/pllp;
      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK frequency --------------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK frequency */
  SystemCoreClock >>= tmp;
}

/**
  * @brief  Configures the System clock source, PLL Multiplier and Divider factors,
  *               AHB/APBx prescalers and Flash settings
  * @note   This function should be called only once the RCC clock configuration
  *         is reset to the default reset state (done in SystemInit() function).
  * @param  None
  * @retval None
  */
void SetSysClock(void)
{
  
  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet. */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;

  PWR->CR |= PWR_CR_VOS;
  
  /* Enable HSE oscillator and activate PLL with HSE as source */
  /*------------------------------- HSE Configuration ------------------------*/

  /* Reset HSEON and HSEBYP bits before configuring the HSE --------------*/
  RCC->CR &= ~(RCC_CR_HSEON);
  while(RCC->CR & RCC_CR_HSERDY);
  
  /* Set the new HSE configuration ---------------------------------------*/
  RCC->CR |= RCC_CR_HSEON;
  while(!(RCC->CR & RCC_CR_HSERDY));

  /*-------------------------------- PLL Configuration -----------------------*/
  /* Disable the main PLL. */
  RCC->CR &= ~(RCC_CR_PLLON);
  while(RCC->CR & RCC_CR_PLLRDY);

  /* Configure the main PLL clock source, multiplication and division factors. */
  // PLLM:      VCO input clock = 2 MHz (8 MHz / 4)
  // PLLN:      VCO output clock = 384 MHz (2 MHz * 192)
  // PLLP:      PLLCLK = 96 MHz (384 MHz / 4)
  // PLLQ:      USB clock = 48 MHz (384 MHz / 8) --> 48MHz is best choice for USB
  #if __SYSTEM_CLOCK == 96000000
    #if !defined(STM32F411xE) && !defined(STM32F446xx)
      #warning You are running the controller out of specification! 96 MHz!
    #endif
    #define PLLM 4
    #define PLLN 192
    #define PLLP 4
    #define PLLQ 8
    #define LATENCY FLASH_ACR_LATENCY_3WS
  #elif __SYSTEM_CLOCK == 100000000
    #if !defined(STM32F411xE) && !defined(STM32F446xx)
      #warning You are running the controller out of specification! 100 MHz!
    #endif
    #define PLLM 4
    #define PLLN 200
    #define PLLP 4
    #define PLLQ 8
    #define LATENCY FLASH_ACR_LATENCY_3WS
  #elif __SYSTEM_CLOCK == 108000000
    #if !defined(STM32F446xx)
      #warning You are running the controller out of specification! 108 MHz!
    #endif
    #define PLLM 4
    #define PLLN 216
    #define PLLP 4
    #define PLLQ 9
    #define LATENCY FLASH_ACR_LATENCY_3WS
  #elif __SYSTEM_CLOCK == 125000000
    #if !defined(STM32F446xx)
      #warning You are running the controller out of specification! 125 MHz!
    #endif
    #define PLLM 4
    #define PLLN 250
    #define PLLP 4
    #define PLLQ 10
    #define LATENCY FLASH_ACR_LATENCY_4WS
  #elif __SYSTEM_CLOCK == 84000000
    #define PLLM 4
    #define PLLN 168
    #define PLLP 4
    #define PLLQ 7
    #define LATENCY FLASH_ACR_LATENCY_2WS
  #elif __SYSTEM_CLOCK == 180000000
    #if !defined(STM32F446xx)
      #error You are running the controller out of specification! 180 MHz!
    #endif
    #define PLLM 4
    #define PLLN 180
    #define PLLP 2
    #define PLLQ 8
    #define LATENCY FLASH_ACR_LATENCY_5WS
  #elif __SYSTEM_CLOCK == 250000000
    #if !defined(STM32F446xx)
      #error You are running the controller out of specification! 250 MHz!
    #else
      #warning You are running the controller out of specification! 250 MHz!
    #endif
    #define PLLM 4
    #define PLLN 250
    #define PLLP 2
    #define PLLQ 9
    #define LATENCY FLASH_ACR_LATENCY_5WS
  #endif

  RCC->PLLCFGR =  RCC_PLLCFGR_PLLSRC_HSE |
                  ((PLLM << 0) & RCC_PLLCFGR_PLLM) |
                  ((PLLN << 6) & RCC_PLLCFGR_PLLN) |
                  (((PLLP/2 - 1) << 16) & RCC_PLLCFGR_PLLP) |
                  ((PLLQ << 24) & RCC_PLLCFGR_PLLQ);

  /* Enable the main PLL. */
  RCC->CR |= RCC_CR_PLLON;
  while(!(RCC->CR & RCC_CR_PLLRDY));

  /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
    must be correctly programmed according to the frequency of the CPU clock
    (HCLK) and the supply voltage of the device. */

  /* Increasing the CPU frequency */
  if(LATENCY > (FLASH->ACR & FLASH_ACR_LATENCY))
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    FLASH->ACR &= ~(FLASH_ACR_LATENCY);
    FLASH->ACR |= LATENCY;

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_SW);
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_SW_PLL;
  }
  /* Decreasing the CPU frequency */
  else
  {
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_SW);
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_SW_PLL;

    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    FLASH->ACR &= ~(FLASH_ACR_LATENCY);
    FLASH->ACR |= LATENCY;
  }

  FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

  RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
  RCC->CFGR |= PPRE1_DIV | PPRE2_DIV;

  RCC->DCKCFGR |= RCC_DCKCFGR_TIMPRE;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif
