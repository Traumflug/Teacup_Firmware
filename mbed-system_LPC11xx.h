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
  Notes for Teacup:

  Copied from $(MBED)/libraries/mbed/targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/system_LPC11xx.h.

  Used only to get things running quickly. Without serial it's almost
  impossible to see wether code changes work. Should go away soon, because
  all this MBED stuff is too bloated for Teacup's purposes.

  - Prefixed names of #include files with mbed- to match the names of the
    copies in the Teacup repo.
*/


#ifndef __SYSTEM_LPC11xx_H
#define __SYSTEM_LPC11xx_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

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
