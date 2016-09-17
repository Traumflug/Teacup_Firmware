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

#ifdef SPI

#define TEACUP_C_INCLUDE
#include "spi-avr.c"
// Each ARM needs it's own file
// #include "spi-lpc.c"
#include "spi-stm32.c"
#undef TEACUP_C_INCLUDE

#endif /* SPI */
