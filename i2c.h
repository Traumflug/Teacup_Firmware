#ifndef _I2C_H
#define _I2C_H

#include "config_wrapper.h"

#ifdef I2C

/** \def I2C_MASTER_MODE

  Wether we act as I2C master. Masters initiate transmissions and can talk to
  multiple clients. Multiple masters can exist on a single bus.

  This implementation can act either as master or as slave. Uncomment this
  #define to act as master.
*/
#define I2C_MASTER_MODE

/** \def I2C_SLAVE_MODE

  Uncomment this #define to make this implementation act as slave. Note that
  this part of the implementation is untested and might not even compile.
*/
//#define I2C_SLAVE_MODE

/** \def I2C_READ_SUPPORT

  Currently, reading from the I2C bus is implemented only partially. The
  already existing parts are wrapped with this #define.

  If reading from I2C is needed, at least a read buffer has to be added. This
  buffer used to be 'i2c_buffer[]', a pointer 'i2c_index' into this buffer and
  a variable 'i2c_byte_count' to track on how many bytes should be read.
  Further requirements are adjustments to the wrapped code to use this buffer
  and a function to make the read data available to calling code.
*/
//#define I2C_READ_SUPPORT

/** \def I2C_EEPROM_SUPPORT

  It's currently unclear what this enables exactly, apparently something to
  deal easier with EEPROM chips. Uncomment this #define in addition to
  I2C_MASTER_MODE or I2C_SLAVE_MODE to enable it.

  Note that this part of the implementation is untested, it might not even
  compile.
*/
//#define I2C_EEPROM_SUPPORT

/** \def I2C_BITRATE

  Define the I2C bus speed here if acting as master. Maximum supported by
  AVRs is said to be 400000.

  Unit: bits/second.
*/
#define I2C_BITRATE                 100000

/** \def I2C_ENABLE_PULLUPS

  Comment this out if there are external pullups in the hardware.
*/
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
void i2c_write(uint8_t data, uint8_t last_byte);

#endif /* I2C */

#endif /* _I2C_H */
