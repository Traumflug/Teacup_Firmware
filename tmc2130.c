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

#include "tmc2130.h"
#include "sersendf.h"

#ifdef TMC2130

static uint8_t status;
static enum axis_e current_axis;

void tmc2130_init() {
  #ifdef X_TMC_CS_PIN
    SET_OUTPUT(X_TMC_CS_PIN);
    WRITE(X_TMC_CS_PIN, 1);
    tmc2130_init_axis(X);
  #endif
  #ifdef Y_TMC_CS_PIN
    SET_OUTPUT(Y_TMC_CS_PIN);
    WRITE(Y_TMC_CS_PIN, 1);
    tmc2130_init_axis(Y);
  #endif
  #ifdef Z_TMC_CS_PIN
    SET_OUTPUT(Z_TMC_CS_PIN);
    WRITE(Z_TMC_CS_PIN, 1);
    tmc2130_init_axis(Z);
  #endif
  #ifdef E_TMC_CS_PIN
    SET_OUTPUT(E_TMC_CS_PIN);
    WRITE(E_TMC_CS_PIN, 1);
    tmc2130_init_axis(E);
  #endif
}

void tmc2130_init_axis(enum axis_e axis) {
  current_axis = axis;
  set_mres(0, 1); //64
  set_ihold_irun(0, 20, 5);
  set_i_scale_analog(1);
  set_tbl(1);
  set_toff(8);
  set_tcoolthrs(1000);
  set_gconf(GCONF_DIAG1_STALL | GCONF_DIAG1_PUSHPULL, GCONF_DIAG1_STALL | GCONF_DIAG1_PUSHPULL); // diag1_stall
  set_sfilt(1);
  set_sgt(13);
}

void tmc_set_axis(enum axis_e axis) {
  current_axis = axis;
}

/*
  Maybe we need later some debug-code or whatelse in clock.c
  For example I used this for setting up the correct values for StallGuard2.
  Add tmc_tick() in clock.c just below heater_temp_tick(). Then you can see
  the values on your host in the temperature graph. While this is an ugly hack,
  only use it for viewing the SG values. Maybe you will see some spikes from
  your normal heater readings.
*/
void tmc_tick() {
  // uint16_t sg_result;
  // get_sg_result(&sg_result);
  // sersendf_P(PSTR("T:%d\n"), sg_result);
}

/*  
*   Reading or writing, we always return the status bits 
*   from the TMC2130. 
*/
uint8_t read_status_tmc2130() {
  spi_mode(3);

  spi_select_tmc2130(current_axis);
  status = spi_rw(0x00);

  for (uint8_t i=4; i; i--) {
    spi_rw(0x00);
  }
  spi_deselect_tmc2130(current_axis);

  spi_mode(0);
  return status;
}

uint8_t read_register_tmc2130(uint8_t tmc_register, uint32_t *data) {
  spi_mode(3);

  spi_select_tmc2130(current_axis);
  status = spi_rw(tmc_register & ~(TMC_WRITE));

  for (uint8_t i=4; i; i--) {
    spi_rw(0x00);
  }
  spi_deselect_tmc2130(current_axis);

  spi_select_tmc2130(current_axis);

  status = spi_rw(0x00);

  *data = spi_rw(0x00) & 0xFF;
  *data <<= 8;
  *data |= spi_rw(0x00) & 0xFF;
  *data <<= 8;
  *data |= spi_rw(0x00) & 0xFF;
  *data <<= 8;
  *data |= spi_rw(0x00) & 0xFF;

  spi_deselect_tmc2130(current_axis);

  spi_mode(0);
  return status;
}

uint8_t write_register_tmc2130(uint8_t tmc_register, uint32_t data) {
  spi_mode(3);

  spi_select_tmc2130(current_axis);
  status = spi_rw(tmc_register | TMC_WRITE);

  spi_rw((data >> 24) & 0xFF);
  spi_rw((data >> 16) & 0xFF);
  spi_rw((data >> 8) & 0xFF);
  spi_rw((data) & 0xFF);

  spi_deselect_tmc2130(current_axis);

  spi_mode(0);
  return status;
}

// micosteps mres
// IHOLD_IRUN 31,31,5 [0-31][0-31][0-5]
// I_scale_analog 1 -> 0 internal, 1 external
// tbl 1 -> set comparator blank time
// toff 8 -> [0-15] off time setting during slow decay phase

uint8_t set_register_tmc2130(uint8_t tmc_register, uint32_t data, uint32_t mask) {
  uint32_t current_data;
  
  read_register_tmc2130(tmc_register, &current_data);
  data = (current_data & ~(mask)) | (data & mask);
  write_register_tmc2130(tmc_register, data);

  return status;
}

uint8_t set_i_scale_analog(uint8_t ain) {
  set_register_tmc2130(TMC_REG_GCONF, ain << 0, 0x1 << 0);
  return status;
}

uint8_t set_gconf(uint32_t gconf, uint32_t mask) {
  gconf &= ~(1U << 17);
  set_register_tmc2130(TMC_REG_GCONF, gconf, mask);
  return status;
}

/* start CHOPCONF */
uint8_t set_mres(uint8_t mres, uint8_t intpol) {
  set_register_tmc2130(TMC_REG_CHOPCONF, mres << 24 | intpol << 28, (0xF << 24) | (0x1 << 28));
  return status;
}

uint8_t set_tbl(uint8_t blank_time) {
  set_register_tmc2130(TMC_REG_CHOPCONF, blank_time << 15, 0x3 << 15);
  return status;
}

uint8_t set_toff(uint8_t off_time) {
  set_register_tmc2130(TMC_REG_CHOPCONF, off_time, 0xF);
  return status;
}

/* end CHOPCONF */
uint8_t set_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t ihold_delay) {
  uint32_t data;
  
  data = ihold & 0x1F;
  data |= ((irun & 0x1F) << 8);
  data |= ((ihold_delay & 0xF) << 16);

  write_register_tmc2130(TMC_REG_IHOLD_IRUN, data);

  return status;
}

uint8_t set_tcoolthrs(uint32_t tcoolthrs) {
  write_register_tmc2130(TMC_REG_TCOOLTHRS, tcoolthrs & 0xFFFFF);
  return status;
}

uint8_t get_ioin(uint32_t *data) {
  read_register_tmc2130(TMC_REG_IOIN, data);
  return status;
}

uint8_t set_sgt(int8_t sgt) {
  if (sgt > 0) {
    write_register_tmc2130(TMC_REG_COOLCONF, (sgt & 0x7F) << 16);
  }
  else {
    write_register_tmc2130(TMC_REG_COOLCONF, ((sgt & 0x7F) << 16) | (1UL << 22));
  }
  return status;
}

uint8_t set_sfilt(uint8_t sfilt) {
  write_register_tmc2130(TMC_REG_COOLCONF, (sfilt & 0x1) << 24);
  return status;
}

uint8_t get_sg_result(uint16_t *data) {
  uint32_t temp;
  read_register_tmc2130(TMC_REG_DRV_STATUS, &temp);
  *data = (temp & 0x1FF);
  return status;
}

uint8_t get_stallguard(uint8_t *sgstatus) {
  uint32_t temp;
  read_register_tmc2130(TMC_REG_DRV_STATUS, &temp);
  *sgstatus = (temp >> 24) & 0x1;
  return status;
}

uint8_t get_ot_flags(uint8_t *data) {
  uint32_t temp;
  read_register_tmc2130(TMC_REG_DRV_STATUS, &temp);
  *data = (temp >> 25) & 0x3;
  return status;
}

#endif
