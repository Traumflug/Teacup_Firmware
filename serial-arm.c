
/** \file
  \brief Serial subsystem, ARM specific part.

  To be included from serial.c, for details see there.
*/

#if defined TEACUP_C_INCLUDE && defined __ARMEL__

#include "arduino.h"


/** Initialise serial subsystem.

  Set up baud generator and interrupts, clear buffers.
*/
void serial_init() {
}

/** Check how many characters can be read.
*/
uint8_t serial_rxchars(void) {
  return 0;
}

/** Read one character.
*/
uint8_t serial_popchar(void) {
  return 0;
}

/** Send one character.
*/
void serial_writechar(uint8_t data) {
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
