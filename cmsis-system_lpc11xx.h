/**************************************************************************//**
 * @file     system_LPC11xx.h
 * @brief    CMSIS Cortex-M0 Device Peripheral Access Layer Header File
 *           for the NXP LPC11xx/LPC11Cxx Device Series
 * @version  V1.10
 * @date     24. November 2010
 *
 * @note
 * Copyright (C) 2009-2010 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/
/*
  Copied in spring 2015 from https://github.com/mbedmicro/mbed, file
    mbed/libraries/mbed/targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/
      system_LPC11xx.h

  Changes for Teacup:

  - Moved all the definitions from system_LPC11xx.h to here to have them
    available elsewhere. It's the section between
      // From system_LPC11xx.c start.
      and
      // From system_LPC11xx.c start.
  - Replaced tabs by spaces and removed trailing whitespace.
*/

#ifndef __SYSTEM_LPC11xx_H
#define __SYSTEM_LPC11xx_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// From system_LPC11xx.c start.
/*
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/

/*--------------------- Clock Configuration ----------------------------------
//
// <e> Clock Configuration
//   <h> System Oscillator Control Register (SYSOSCCTRL)
//     <o1.0>      BYPASS: System Oscillator Bypass Enable
//                     <i> If enabled then PLL input (sys_osc_clk) is fed
//                     <i> directly from XTALIN and XTALOUT pins.
//     <o1.9>      FREQRANGE: System Oscillator Frequency Range
//                     <i> Determines frequency range for Low-power oscillator.
//                   <0=> 1 - 20 MHz
//                   <1=> 15 - 25 MHz
//   </h>
//
//   <h> Watchdog Oscillator Control Register (WDTOSCCTRL)
//     <o2.0..4>   DIVSEL: Select Divider for Fclkana
//                     <i> wdt_osc_clk = Fclkana/ (2 * (1 + DIVSEL))
//                   <0-31>
//     <o2.5..8>   FREQSEL: Select Watchdog Oscillator Analog Output Frequency (Fclkana)
//                   <0=> Undefined
//                   <1=> 0.5 MHz
//                   <2=> 0.8 MHz
//                   <3=> 1.1 MHz
//                   <4=> 1.4 MHz
//                   <5=> 1.6 MHz
//                   <6=> 1.8 MHz
//                   <7=> 2.0 MHz
//                   <8=> 2.2 MHz
//                   <9=> 2.4 MHz
//                   <10=> 2.6 MHz
//                   <11=> 2.7 MHz
//                   <12=> 2.9 MHz
//                   <13=> 3.1 MHz
//                   <14=> 3.2 MHz
//                   <15=> 3.4 MHz
//   </h>
//
//   <h> System PLL Control Register (SYSPLLCTRL)
//                   <i> F_clkout = M * F_clkin = F_CCO / (2 * P)
//                   <i> F_clkin must be in the range of  10 MHz to  25 MHz
//                   <i> F_CCO   must be in the range of 156 MHz to 320 MHz
//     <o3.0..4>   MSEL: Feedback Divider Selection
//                     <i> M = MSEL + 1
//                   <0-31>
//     <o3.5..6>   PSEL: Post Divider Selection
//                   <0=> P = 1
//                   <1=> P = 2
//                   <2=> P = 4
//                   <3=> P = 8
//   </h>
//
//   <h> System PLL Clock Source Select Register (SYSPLLCLKSEL)
//     <o4.0..1>   SEL: System PLL Clock Source
//                   <0=> IRC Oscillator
//                   <1=> System Oscillator
//                   <2=> Reserved
//                   <3=> Reserved
//   </h>
//
//   <h> Main Clock Source Select Register (MAINCLKSEL)
//     <o5.0..1>   SEL: Clock Source for Main Clock
//                   <0=> IRC Oscillator
//                   <1=> Input Clock to System PLL
//                   <2=> WDT Oscillator
//                   <3=> System PLL Clock Out
//   </h>
//
//   <h> System AHB Clock Divider Register (SYSAHBCLKDIV)
//     <o6.0..7>   DIV: System AHB Clock Divider
//                     <i> Divides main clock to provide system clock to core, memories, and peripherals.
//                     <i> 0 = is disabled
//                   <0-255>
//   </h>
// </e>
*/
#define CLOCK_SETUP           1
#define SYSOSCCTRL_Val        0x00000000              // Reset: 0x000
#define WDTOSCCTRL_Val        0x00000000              // Reset: 0x000
#define SYSPLLCTRL_Val        0x00000023              // Reset: 0x000
#define SYSPLLCLKSEL_Val      0x00000000              // Reset: 0x000   // Define as using IRC
#define MAINCLKSEL_Val        0x00000003              // Reset: 0x000   // Define as using System PLL clock out
#define SYSAHBCLKDIV_Val      0x00000001              // Reset: 0x001

/*
//-------- <<< end of configuration section >>> ------------------------------
*/

/*----------------------------------------------------------------------------
  Check the register settings
 *----------------------------------------------------------------------------*/
#define CHECK_RANGE(val, min, max)                ((val < min) || (val > max))
#define CHECK_RSVD(val, mask)                     (val & mask)

/* Clock Configuration -------------------------------------------------------*/
#if (CHECK_RSVD((SYSOSCCTRL_Val),  ~0x00000003))
   #error "SYSOSCCTRL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((WDTOSCCTRL_Val),  ~0x000001FF))
   #error "WDTOSCCTRL: Invalid values of reserved bits!"
#endif

#if (CHECK_RANGE((SYSPLLCLKSEL_Val), 0, 2))
   #error "SYSPLLCLKSEL: Value out of range!"
#endif

#if (CHECK_RSVD((SYSPLLCTRL_Val),  ~0x000001FF))
   #error "SYSPLLCTRL: Invalid values of reserved bits!"
#endif

#if (CHECK_RSVD((MAINCLKSEL_Val),  ~0x00000003))
   #error "MAINCLKSEL: Invalid values of reserved bits!"
#endif

#if (CHECK_RANGE((SYSAHBCLKDIV_Val), 0, 255))
   #error "SYSAHBCLKDIV: Value out of range!"
#endif


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __XTAL            (12000000UL)    /* Oscillator frequency             */
#define __SYS_OSC_CLK     (    __XTAL)    /* Main oscillator frequency        */
#define __IRC_OSC_CLK     (12000000UL)    /* Internal RC oscillator frequency */


#define __FREQSEL   ((WDTOSCCTRL_Val >> 5) & 0x0F)
#define __DIVSEL   (((WDTOSCCTRL_Val & 0x1F) << 1) + 2)

#if (CLOCK_SETUP)                         /* Clock Setup              */
  #if  (__FREQSEL ==  0)
    #define __WDT_OSC_CLK        ( 0)                  /* undefined */
  #elif (__FREQSEL ==  1)
    #define __WDT_OSC_CLK        ( 500000 / __DIVSEL)
  #elif (__FREQSEL ==  2)
    #define __WDT_OSC_CLK        ( 800000 / __DIVSEL)
  #elif (__FREQSEL ==  3)
    #define __WDT_OSC_CLK        (1100000 / __DIVSEL)
  #elif (__FREQSEL ==  4)
    #define __WDT_OSC_CLK        (1400000 / __DIVSEL)
  #elif (__FREQSEL ==  5)
    #define __WDT_OSC_CLK        (1600000 / __DIVSEL)
  #elif (__FREQSEL ==  6)
    #define __WDT_OSC_CLK        (1800000 / __DIVSEL)
  #elif (__FREQSEL ==  7)
    #define __WDT_OSC_CLK        (2000000 / __DIVSEL)
  #elif (__FREQSEL ==  8)
    #define __WDT_OSC_CLK        (2200000 / __DIVSEL)
  #elif (__FREQSEL ==  9)
    #define __WDT_OSC_CLK        (2400000 / __DIVSEL)
  #elif (__FREQSEL == 10)
    #define __WDT_OSC_CLK        (2600000 / __DIVSEL)
  #elif (__FREQSEL == 11)
    #define __WDT_OSC_CLK        (2700000 / __DIVSEL)
  #elif (__FREQSEL == 12)
    #define __WDT_OSC_CLK        (2900000 / __DIVSEL)
  #elif (__FREQSEL == 13)
    #define __WDT_OSC_CLK        (3100000 / __DIVSEL)
  #elif (__FREQSEL == 14)
    #define __WDT_OSC_CLK        (3200000 / __DIVSEL)
  #else
    #define __WDT_OSC_CLK        (3400000 / __DIVSEL)
  #endif

  /* sys_pllclkin calculation */
  #if   ((SYSPLLCLKSEL_Val & 0x03) == 0)
    #define __SYS_PLLCLKIN           (__IRC_OSC_CLK)
  #elif ((SYSPLLCLKSEL_Val & 0x03) == 1)
    #define __SYS_PLLCLKIN           (__SYS_OSC_CLK)
  #else
    #define __SYS_PLLCLKIN           (0)
  #endif

  #define  __SYS_PLLCLKOUT         (__SYS_PLLCLKIN * ((SYSPLLCTRL_Val & 0x01F) + 1))

  /* main clock calculation */
  #if   ((MAINCLKSEL_Val & 0x03) == 0)
    #define __MAIN_CLOCK             (__IRC_OSC_CLK)
  #elif ((MAINCLKSEL_Val & 0x03) == 1)
    #define __MAIN_CLOCK             (__SYS_PLLCLKIN)
  #elif ((MAINCLKSEL_Val & 0x03) == 2)
    #if (__FREQSEL ==  0)
      #error "MAINCLKSEL: WDT Oscillator selected but FREQSEL is undefined!"
    #else
      #define __MAIN_CLOCK           (__WDT_OSC_CLK)
    #endif
  #elif ((MAINCLKSEL_Val & 0x03) == 3)
    #define __MAIN_CLOCK             (__SYS_PLLCLKOUT)
  #else
    #define __MAIN_CLOCK             (0)
  #endif

  #define __SYSTEM_CLOCK             (__MAIN_CLOCK / SYSAHBCLKDIV_Val)

#else
  #define __SYSTEM_CLOCK             (__IRC_OSC_CLK)
#endif  // CLOCK_SETUP
// From system_LPC11xx.c end.


extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_LPC11xx_H */
