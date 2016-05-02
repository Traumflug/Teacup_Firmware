
/** \file

  \brief Display bus broker.

  Here we map generic calls to the display bus to calls to the actually used
  bus.

  This is a slightly different #include strategy than the one used for display
  and fonts. Instead of including a selected .c file and having the same
  function names in each included .c file, we map functions with simple,
  generic named inline funtions to get what we want. This allows to re-use
  functions used elsewhere. For example SPI functions, which are used for SD
  card reading and some temperature sensors, too.

  The other approach, in turn, needs no mapping and can use variables, too,
  but allows to use the functions in one place, only.
*/

#ifndef _DISPLAYBUS_H
#define _DISPLAYBUS_H

#include "config_wrapper.h"

#if defined DISPLAY_BUS_4BIT

  #include "parallel-4bit.h"

  static void displaybus_init(uint8_t address) __attribute__ ((always_inline));
  inline void displaybus_init(uint8_t address) {
    return parallel_4bit_init();
  }

  static uint8_t displaybus_busy(void) __attribute__ ((always_inline));
  inline uint8_t displaybus_busy(void) {
    return parallel_4bit_busy();
  }

  /**
    Note: we use 'rs' here to decide wether to send data or a command. Other
          buses, like I2C, make no such distinction, but have last_byte
          instead. This works, because there is currently no display supported
          which can use both, I2C and the 4-bit bus.

          In case such support is wanted, displaybus_write() likely needs to
          take both parameters. The actually best solution will be seen better
          if such a display actually appears.
  */
  static void displaybus_write(uint8_t data, enum rs_e rs) \
    __attribute__ ((always_inline));
  inline void displaybus_write(uint8_t data, enum rs_e rs) {
    return parallel_4bit_write(data, rs);
  }

  #define DISPLAY_BUS

#elif defined DISPLAY_BUS_8BIT

  #error Display connected directly via 8 pins is not yet supported.

#elif defined DISPLAY_BUS_I2C

  #include "i2c.h"

  static void displaybus_init(uint8_t address) __attribute__ ((always_inline));
  inline void displaybus_init(uint8_t address) {
    return i2c_init(address);
  }

  static uint8_t displaybus_busy(void) __attribute__ ((always_inline));
  inline uint8_t displaybus_busy(void) {
    return i2c_busy();
  }

  static void displaybus_write(uint8_t data, uint8_t last_byte) \
    __attribute__ ((always_inline));
  inline void displaybus_write(uint8_t data, uint8_t last_byte) {
    return i2c_write(data, last_byte);
  }

  #define DISPLAY_BUS

#elif defined DISPLAY_BUS_SPI

  #error Display connected via SPI not yet supported.

#endif

#endif /* _DISPLAYBUS_H */
