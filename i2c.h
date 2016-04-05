#ifndef _I2C_H
#define _I2C_H

#include "config_wrapper.h"

#ifdef I2C

// Uncomment if we act as master device.
#define I2C_MASTER_MODE
// Uncomment if we act as slave device.
//#define I2C_SLAVE_MODE
// Uncomment if we use EEPROM chips.
//#define I2C_EEPROM_SUPPORT

// Bus speed. Maximum is said to be 400000.
#define I2C_BITRATE                 100000
// Comment out if there are external pullups.
#define I2C_ENABLE_PULLUPS

/** \def I2C_BUFFER_SIZE

  Size of send buffer. MUST be a \f$2^n\f$ value, maximum is 512.

  A larger buffer allows to store more display data immediately, so it can
  speed operations up. An exhaused buffer doesn't mean data gets lost, writing
  to the buffer then waits until sufficient previous data is sent.
*/
#define I2C_BUFFER_SIZE             128

#ifdef I2C_SLAVE_MODE
  #define I2C_SLAVE_RX_BUFFER_SIZE  1
  #define I2C_SLAVE_TX_BUFFER_SIZE  1
#endif /* I2C_SLAVE_MODE */

#ifdef I2C_EEPROM_SUPPORT
  // Depends on EEPROM type, usually it is 1 or 2 bytes.
  #define I2C_PAGE_ADDRESS_SIZE     2
#endif /* I2C_EEPROM_SUPPORT */


void i2c_init(uint8_t address);
uint8_t i2c_busy(void);
void i2c_write(uint8_t address, uint8_t data, uint8_t last_byte);

#endif /* I2C */

#endif /* _I2C_H */
