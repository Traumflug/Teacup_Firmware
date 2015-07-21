
/** \file
  \brief SPI subsystem

  This is much simpler than Teacup's serial subsystem. No ring buffers, no
  "write string" functions. Usually using SPI directly is faster than fiddling
  with buffers. For example, reading or writing a byte can be done in as few
  as 20 clock cycles.

  Other than serial, SPI has to deal with multiple devices. Device selection
  happens before reading and writing, data exchange its self is the same for
  each device, then.
*/
#include "spi.h"

#include "arduino.h"
#include "pinio.h"


/** Initialise serial subsystem.

  Code copied from ATmega164/324/644/1284 data sheet, section 18.2, page 160,
  or moved here from mendel.c.
*/
void spi_init() {

  // Set SCK (clock) and MOSI line to output, ie. set USART in master mode.
  SET_OUTPUT(SCK);
  SET_OUTPUT(MOSI);
  SET_INPUT(MISO);
  // SS must be set as output to disconnect it from the SPI subsystem.
  // Too bad if something else tries to use this pin as digital input.
  // See ATmega164/324/644/1284 data sheet, section 18.3.2, page 162.
  // Not written there: this must apparently be done before setting the SPRC
  // register, else future R/W-operations may hang.
  SET_OUTPUT(SS);

  #ifdef SPI_2X
    SPSR = 0x01;
  #else
    SPSR = 0x00;
  #endif

  // This sets the whole SPRC register.
  spi_speed_100_400();
}
