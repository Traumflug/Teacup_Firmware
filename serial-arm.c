
/** \file
  \brief Serial subsystem, ARM specific part.

  To be included from serial.c, for details see there.
*/

#if defined TEACUP_C_INCLUDE && defined __ARMEL__

#include "arduino.h"
#include "mbed-serial_api.h"


serial_t serial_line;

/** Initialise serial subsystem.

  Set up baud generator and interrupts, clear buffers.
*/
void serial_init() {
  mbed_serial_init(&serial_line, USBTX, USBRX);
  mbed_serial_baud(&serial_line, 115200);
  mbed_serial_format(&serial_line, 8, ParityNone, 1);
}

/** Check wether characters can be read.

  Other than the AVR implementation this returns not the number of characters
  in the line, but only wether there is at least one or not.
*/
uint8_t serial_rxchars(void) {
  return mbed_serial_readable(&serial_line);
}

/** Read one character.
*/
uint8_t serial_popchar(void) {
  return mbed_serial_getc(&serial_line);
}

/** Send one character.
*/
void serial_writechar(uint8_t data) {
  mbed_serial_putc(&serial_line, data);
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
