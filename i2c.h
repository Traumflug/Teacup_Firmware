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

#define I2C_BITRATE                 100000
#define I2C_PORT                    PORTC
#define I2C_DDR                     DDRC

#define I2C_SCL_PIN                 0
#define I2C_SDA_PIN                 1
#define I2C_ENABLE_PULLUPS

#define I2C_BUFFER_SIZE             4
#ifdef I2C_EEPROM_SUPPORT
  // Depends on EEPROM type, usually it is 1 or 2 bytes.
  #define I2C_PAGE_ADDRESS_SIZE     2
#endif /* I2C_EEPROM_SUPPORT */

#ifdef I2C_SLAVE_MODE
  #define I2C_SLAVE_RX_BUFFER_SIZE  1
  #define I2C_SLAVE_TX_BUFFER_SIZE  1
#endif /* I2C_SLAVE_MODE */

#ifdef I2C_SLAVE_MODE
  #define I2C_MODE 1
#else
  #define I2C_MODE 0
#endif

#define I2C_MODE_MASK           0b00001100
#define I2C_MODE_SARP           0b00000000 // Start-Addr_R-Read-Stop: just read mode
#define I2C_MODE_SAWP           0b00000100 // Start-Addr_W-Write-Stop: just write mode
#define I2C_MODE_ENHA           0b00001000 // Start-Addr_W-WrPageAdr-rStart-Addr_R-Read-Stop
#define I2C_MODE_BUSY           0b01000000 // Transponder is busy
#define I2C_MODE_FREE           0b10111111 // Transponder is free

// Transmission interrupted.
#define I2C_INTERRUPTED                 0b10000000
// Transmission not interrupted.
#define I2C_NOINTERRUPTED               0b01111111

#define I2C_ERROR_BUS_FAIL              0b00000001
#define I2C_ERROR_NACK                  0b00000010
#define I2C_ERROR_NO_ANSWER             0b00010000
#define I2C_ERROR_LOW_PRIO              0b00100000


void i2c_init(uint8_t address);
void i2c_send(uint8_t address, uint8_t* block, uint8_t tx_len);

#endif /* I2C */

#endif /* _I2C_H */
