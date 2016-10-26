/**
 * TMC2130 stepper driver
 *
 *  Copyright 2016 by Nico Tonnhofer <wurstnase.reprap@gmail.com>
 *
 *  Licensed under GNU General Public License 3.0 or later. 
 *  Some rights reserved. See COPYING, AUTHORS.
 *
 * @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

/*
 * Ideas and help from Moritz Walter
 * https://github.com/makertum/Trinamic_TMC2130
 */

#ifndef _TMC2130_H
#define _TMC2130_H

#include "config_wrapper.h"

#ifdef TMC2130

#include <stdint.h>

#include "dda.h"
#include "spi.h"

void tmc2130_init(void);

void tmc_set_axis(enum axis_e axis);

void tmc_tick(void);

uint8_t read_status_tmc2130(void);
uint8_t read_register_tmc2130(uint8_t tmc_register, uint32_t *data);

uint8_t write_register_tmc2130(uint8_t tmc_register, uint32_t data);

void tmc2130_init_axis(enum axis_e axis);

uint8_t set_gconf(uint32_t gconf, uint32_t mask);

uint8_t set_mres(uint8_t mres, uint8_t intpol);
uint8_t set_tbl(uint8_t blank_time);
uint8_t set_toff(uint8_t off_time);

uint8_t get_ioin(uint32_t *data);

uint8_t set_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t ihold_delay);
uint8_t set_tcoolthrs(uint32_t tcoolthrs);
uint8_t set_i_scale_analog(uint8_t ain);

uint8_t set_sgt(int8_t sgt);
uint8_t set_sfilt(uint8_t sfilt);
uint8_t get_sg_result(uint16_t *data);
uint8_t get_stallguard(uint8_t *sgstatus);
uint8_t get_ot_flags(uint8_t *data);

// RW
#define TMC_READ                         (0x00)
#define TMC_WRITE                        (0x80)

// SPISTATUS MASKS
#define TMC_SPISTATUS_RESET_MASK         (0x01)
#define TMC_SPISTATUS_ERROR_MASK         (0x02)
#define TMC_SPISTATUS_STALLGUARD_MASK    (0x04)
#define TMC_SPISTATUS_STANDSTILL_MASK    (0x08)

// REGISTERS
#define TMC_REG_GCONF                    (0x00) // RW //    17 // Global configuration flags
#define TMC_REG_GSTAT                    (0x01) // RC //     3 // Global status flags
#define TMC_REG_IOIN                     (0x04) // R  //   8+8 // Reads the state of all input pins available
#define TMC_REG_IHOLD_IRUN               (0x10) // W  // 5+5+4 // Driver current control
#define TMC_REG_TPOWERDOWN               (0x11) // W  //     8 // sets delay time after stand still (stst) to motor current power down (0-4 seconds) 0_((2^8)-1) * 2^18 tclk
#define TMC_REG_TSTEP                    (0x12) // R  //    20 // Actual measured time between two 1/256 microsteps derived from the step input frequency in units of 1/fCLK. Measured value is (2^20)-1 in case of overflow or stand still
#define TMC_REG_TPWMTHRS                 (0x13) // W  //    20 // Upper velocity threshold for stealthChop voltage PWM mode
#define TMC_REG_TCOOLTHRS                (0x14) // W  //    20 // Lower threshold velocity for switching on smart energy coolStep and stallGuard feature (unsigned)
#define TMC_REG_THIGH                    (0x15) // W  //    20 // Velocity dependend switching into different chopper mode and fullstepping to maximize torque (unsigned)
#define TMC_REG_XDIRECT                  (0x2D) // RW //    32 // specifies motor coil currents and polarity directly programmed via SPI. Use signed, two's complement numbers. In this mode, the current is scaled by IHOLD. Velocity based current regulation of voltage PWM is not available in this mode. +- 255 for both coils
#define TMC_REG_VDCMIN                   (0x33) // W  //    23 // automatic commutation dcStep becomes enabled by the external signal DCEN. VDCMIN is used as the minimum step velocity when the motor is heavily loaded. Hint: Also set DCCTRL parameters in order to operate dcStep
#define TMC_REG_MSLUT0                   (0x60) // W  //    32 // Each bit gives the difference between entry x and entry x+1 when combined with the corresponding MSLUTSEL W bits. Differential coding for the first quarter of a wave. Start values for CUR_A and CUR_B are stored for MSCNT position 0 in START_SIN and START_SIN90.
#define TMC_REG_MSLUT1                   (0x61) // W  //    32 //
#define TMC_REG_MSLUT2                   (0x62) // W  //    32 //
#define TMC_REG_MSLUT3                   (0x63) // W  //    32 //
#define TMC_REG_MSLUT4                   (0x64) // W  //    32 //
#define TMC_REG_MSLUT5                   (0x65) // W  //    32 //
#define TMC_REG_MSLUT6                   (0x66) // W  //    32 //
#define TMC_REG_MSLUT7                   (0x67) // W  //    32 //
#define TMC_REG_MSLUTSEL                 (0x68) // W  //    32 // defines four segments within each quarter MSLUT wave. Four 2 bit entries determine the meaning of a 0 and a 1 bit in the corresponding segment of MSLUT
#define TMC_REG_MSLUTSTART               (0x69) // W  //   8+8 //
#define TMC_REG_MSCNT                    (0x6A) // R  //    10 //
#define TMC_REG_MSCURACT                 (0x6B) // R  //   9+9 //
#define TMC_REG_CHOPCONF                 (0x6C) // RW //    32 //
#define TMC_REG_COOLCONF                 (0x6D) // W  //    25 //
#define TMC_REG_DCCTRL                   (0x6E) // W  //    24 //
#define TMC_REG_DRV_STATUS               (0x6F) // R  //    22 //
#define TMC_REG_PWMCONF                  (0x70) // W  //     8 //
#define TMC_REG_PWM_SCALE                (0x71) // R  //     8 //
#define TMC_REG_ENCM_CTRL                (0x72) // W  //     2 //
#define TMC_REG_LOST_STEPS               (0x73) // R  //    20 //

#define GCONF_I_SCALE_ANALOG             (0x00000001UL)
#define GCONF_INTERNAL_RSENSE            (0x00000002UL)
#define GCONF_EN_PWM_MODE                (0x00000004UL)
#define GCONF_ENC_COMMUTATION            (0x00000008UL)
#define GCONF_SHAFT                      (0x00000010UL)
#define GCONF_DIAG0_ERROR                (0x00000020UL)
#define GCONF_DIAG0_OTPW                 (0x00000040UL)
#define GCONF_DIAG0_STALL                (0x00000080UL)
#define GCONF_DIAG1_STALL                (0x00000100UL)
#define GCONF_DIAG1_INDEX                (0x00000200UL)
#define GCONF_DIAG1_ONSTATE              (0x00000400UL)
#define GCONF_DIAG1_STEPS_SKIPPED        (0x00000800UL)
#define GCONF_DIAG0_PUSHPULL             (0x00001000UL)
#define GCONF_DIAG1_PUSHPULL             (0x00002000UL)
#define GCONF_SMALL_HYSTERESIS           (0x00004000UL)
#define GCONF_STOP_ENABLE                (0x00008000UL)
#define GCONF_DIRECT_MODE                (0x00010000UL)
//#define GCONF_TEST_MODE                (0x00020000UL)

#endif /* TMC2130 */

#endif /* _TMC2130_H */
