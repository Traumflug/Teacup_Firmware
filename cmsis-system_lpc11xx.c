/**************************************************************************//**
 * @file     system_LPC11xx.c
 * @brief    CMSIS Cortex-M0 Device Peripheral Access Layer Source File
 *           for the NXP LPC11xx/LPC11Cxx Devices
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
      TARGET_LPC11XX/system_LPC11xx.c

  Changes for Teacup:

  - Prefixed names of #include files with cmsis- to match the names of the
    copies in the Teacup repo.
  - Wrapped the whole file in #ifdef __ARMEL__ to not cause conflicts with
    AVR builds.
  - Silenced this warning:
      system_LPC11xx.c:344:21: warning: unused variable 'i'
    by declaring i directly where it's used and renaming the other to 'j'.
  - Moved definitions to system_LPC11xx.h to have them available elsewhere.
    Replaced by this marker:
      // Moved definitions from here to system_LPC11xx.h.
    and replaced #include <stdint.h> with #include "mbed-system_LPC11xx.h"
  - Replaced tabs by spaces and removed trailing whitespace.
*/

#ifdef __ARMEL__

#include "cmsis-system_lpc11xx.h"
#include "cmsis-lpc11xx.h"

// Moved definitions from here to system_LPC11xx.h.

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __SYSTEM_CLOCK;/*!< System Clock Frequency (Core Clock)*/


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  uint32_t wdt_osc = 0;

  /* Determine clock frequency according to clock register values             */
  switch ((LPC_SYSCON->WDTOSCCTRL >> 5) & 0x0F) {
    case 0:  wdt_osc =       0; break;
    case 1:  wdt_osc =  500000; break;
    case 2:  wdt_osc =  800000; break;
    case 3:  wdt_osc = 1100000; break;
    case 4:  wdt_osc = 1400000; break;
    case 5:  wdt_osc = 1600000; break;
    case 6:  wdt_osc = 1800000; break;
    case 7:  wdt_osc = 2000000; break;
    case 8:  wdt_osc = 2200000; break;
    case 9:  wdt_osc = 2400000; break;
    case 10: wdt_osc = 2600000; break;
    case 11: wdt_osc = 2700000; break;
    case 12: wdt_osc = 2900000; break;
    case 13: wdt_osc = 3100000; break;
    case 14: wdt_osc = 3200000; break;
    case 15: wdt_osc = 3400000; break;
  }
  wdt_osc /= ((LPC_SYSCON->WDTOSCCTRL & 0x1F) << 1) + 2;

  switch (LPC_SYSCON->MAINCLKSEL & 0x03) {
    case 0:                             /* Internal RC oscillator             */
      SystemCoreClock = __IRC_OSC_CLK;
      break;
    case 1:                             /* Input Clock to System PLL          */
      switch (LPC_SYSCON->SYSPLLCLKSEL & 0x03) {
          case 0:                       /* Internal RC oscillator             */
            SystemCoreClock = __IRC_OSC_CLK;
            break;
          case 1:                       /* System oscillator                  */
            SystemCoreClock = __SYS_OSC_CLK;
            break;
          case 2:                       /* Reserved                           */
          case 3:                       /* Reserved                           */
            SystemCoreClock = 0;
            break;
      }
      break;
    case 2:                             /* WDT Oscillator                     */
      SystemCoreClock = wdt_osc;
      break;
    case 3:                             /* System PLL Clock Out               */
      switch (LPC_SYSCON->SYSPLLCLKSEL & 0x03) {
          case 0:                       /* Internal RC oscillator             */
            if (LPC_SYSCON->SYSPLLCTRL & 0x180) {
              SystemCoreClock = __IRC_OSC_CLK;
            } else {
              SystemCoreClock = __IRC_OSC_CLK * ((LPC_SYSCON->SYSPLLCTRL & 0x01F) + 1);
            }
            break;
          case 1:                       /* System oscillator                  */
            if (LPC_SYSCON->SYSPLLCTRL & 0x180) {
              SystemCoreClock = __SYS_OSC_CLK;
            } else {
              SystemCoreClock = __SYS_OSC_CLK * ((LPC_SYSCON->SYSPLLCTRL & 0x01F) + 1);
            }
            break;
          case 2:                       /* Reserved                           */
          case 3:                       /* Reserved                           */
            SystemCoreClock = 0;
            break;
      }
      break;
  }

  SystemCoreClock /= LPC_SYSCON->SYSAHBCLKDIV;

}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void) {

#if (CLOCK_SETUP)                                 /* Clock Setup              */

#if ((SYSPLLCLKSEL_Val & 0x03) == 1)
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 5);          /* Power-up System Osc      */
  LPC_SYSCON->SYSOSCCTRL    = SYSOSCCTRL_Val;
  for (volatile uint32_t i = 0; i < 200; i++) __NOP();
#endif

  LPC_SYSCON->SYSPLLCLKSEL  = SYSPLLCLKSEL_Val;   /* Select PLL Input         */
  LPC_SYSCON->SYSPLLCLKUEN  = 0x01;               /* Update Clock Source      */
  LPC_SYSCON->SYSPLLCLKUEN  = 0x00;               /* Toggle Update Register   */
  LPC_SYSCON->SYSPLLCLKUEN  = 0x01;
  while (!(LPC_SYSCON->SYSPLLCLKUEN & 0x01));     /* Wait Until Updated       */
#if ((MAINCLKSEL_Val & 0x03) == 3)                /* Main Clock is PLL Out    */
  LPC_SYSCON->SYSPLLCTRL    = SYSPLLCTRL_Val;
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 7);          /* Power-up SYSPLL          */
  while (!(LPC_SYSCON->SYSPLLSTAT & 0x01));       /* Wait Until PLL Locked    */
#endif

#if (((MAINCLKSEL_Val & 0x03) == 2) )
  LPC_SYSCON->WDTOSCCTRL    = WDTOSCCTRL_Val;
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 6);          /* Power-up WDT Clock       */
  for (volatile uint32_t j = 0; j < 200; j++) __NOP();
#endif

  LPC_SYSCON->MAINCLKSEL    = MAINCLKSEL_Val;     /* Select PLL Clock Output  */
  LPC_SYSCON->MAINCLKUEN    = 0x01;               /* Update MCLK Clock Source */
  LPC_SYSCON->MAINCLKUEN    = 0x00;               /* Toggle Update Register   */
  LPC_SYSCON->MAINCLKUEN    = 0x01;
  while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       /* Wait Until Updated       */

  LPC_SYSCON->SYSAHBCLKDIV  = SYSAHBCLKDIV_Val;
#endif
   /* System clock to the IOCON needs to be enabled or
  most of the I/O related peripherals won't work. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<16);
}
#endif /* __ARMEL__ */
