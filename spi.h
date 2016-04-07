#ifndef _SPI_H
#define _SPI_H

#include "config_wrapper.h"
#include "arduino.h"
#include "pinio.h"

#ifdef SPI

/**
  Test configuration.
*/
#ifdef __ARMEL__
  #error SPI (SD_CARD_SELECT_PIN, TEMP_MAX6675, TEMP_MCP3008) not yet supported on ARM.
#endif

// Uncomment this to double SPI frequency from (F_CPU / 4) to (F_CPU / 2).
//#define SPI_2X

/** Initialise SPI subsystem.
*/
void spi_init(void);

/** SPI device selection.

  Because out famous WRITE() macro works with constant pins, only, we define
  a (de)select function for each of them. In case you add another SPI device,
  you also have to define a pair of these functions.
*/
#ifdef SD
static void spi_select_sd(void) __attribute__ ((always_inline));
inline void spi_select_sd(void) {
  WRITE(SD_CARD_SELECT_PIN, 0);
}

static void spi_deselect_sd(void) __attribute__ ((always_inline));
inline void spi_deselect_sd(void) {
  WRITE(SD_CARD_SELECT_PIN, 1);
}
#endif /* SD */

#ifdef TEMP_MAX6675
// Note: the pin choosen with DEFINE_TEMP_SENSOR() in the board configuration
//       should be used here. Currently it's a requirement that this device's
//       Chip Select pin is actually SS, while any other pin would work just
//       as fine.
static void spi_select_max6675(void) __attribute__ ((always_inline));
inline void spi_select_max6675(void) {
  WRITE(SS, 0);
}

static void spi_deselect_max6675(void) __attribute__ ((always_inline));
inline void spi_deselect_max6675(void) {
  WRITE(SS, 1);
}
#endif /* TEMP_MAX6675 */

#ifdef TEMP_MCP3008
static void spi_select_mcp3008(void) __attribute__ ((always_inline));
inline void spi_select_mcp3008(void) {
  WRITE(MCP3008_SELECT_PIN, 0);
}

static void spi_deselect_mcp3008(void) __attribute__ ((always_inline));
inline void spi_deselect_mcp3008(void) {
  WRITE(MCP3008_SELECT_PIN, 1);
}
#endif /* TEMP_MCP3008 */

/** Set SPI clock speed to something between 100 and 400 kHz.

  This is needed for initialising SD cards. We set the whole SPCR register
  in one step, because this is faster than and'ing in bits.

  About dividers. We have:
  SPCR = 0x50; // normal mode: (F_CPU / 4), 2x mode: (F_CPU / 2)
  SPCR = 0x51; // normal mode: (F_CPU / 16), 2x mode: (F_CPU / 8)
  SPCR = 0x52; // normal mode: (F_CPU / 64), 2x mode: (F_CPU / 32)
  SPCR = 0x53; // normal mode: (F_CPU / 128), 2x mode: (F_CPU / 64)

  For now we always choose the /128 divider, because it fits nicely in all
  expected situations:
    F_CPU                    16 MHz    20 MHz
    SPI clock normal mode   125 kHz   156 kHz
    SPI clock 2x mode       250 kHz   312 kHz

  About the other bits:
  0x50 = (1 << SPE) | (1 << MSTR);
  This is Master SPI mode, SPI enabled, interrupts disabled, polarity mode 0
  (right for SD cards).
  See ATmega164/324/644/1284 data sheet, section 18.5.1, page 164.
*/
static void spi_speed_100_400(void) __attribute__ ((always_inline));
inline void spi_speed_100_400(void) {
  SPCR = 0x53;
}

/** Set SPI clock speed to maximum.
*/
static void spi_speed_max(void) __attribute__ ((always_inline));
inline void spi_speed_max(void) {
  SPCR = 0x50; // See list at spi_speed_100_400().
}

/** Exchange a byte over SPI.

  Yes, SPI is that simple and you can always only swap bytes. To retrieve
  a byte, simply send a dummy value, like: mybyte = spi_rw(0);

  As we operate in master mode, we don't have to fear to hang due to
  communications errors (e.g. missing a clock beat).

  Note: insisting on inlinig (attribute always_inline) costs about 80 bytes
        with the current SD card code.
*/
static uint8_t spi_rw(uint8_t) __attribute__ ((always_inline));
inline uint8_t spi_rw(uint8_t byte) {
  SPDR = byte;
  loop_until_bit_is_set(SPSR, SPIF);
  return SPDR;
}

#endif /* SPI */

#endif /* _SPI_H */
