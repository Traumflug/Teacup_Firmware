
/*!
  \file
  \brief Pin definitions supervisor.

  Here we map to the pin definition file of the architecture at hand and also
  do some fundamental platform related stuff.
*/

#ifndef	_ARDUINO_H
#define	_ARDUINO_H

/*
  The platform-*.h headers must define the platform-specific pin definitions
  needed to communicate with each chip.
*/
#include "platform-avr.h"
#include "platform-arm.h"
#include "platform-sim.h"

#if ! defined DIO0_PIN && ! defined PIO0_1_PIN
  #error Pins for this chip not defined in arduino.h! If you write an \
         appropriate pin definition and have this firmware work on your chip, \
         please tell us via Github or the forum thread.
#endif

/*
  If these are undefined by the platform-specific headers, define them here
  with common default definitions.
 */
#ifndef PROGMEM
  #define PROGMEM
  #define PGM_P const char *
  #define PSTR(s) ((const PROGMEM char *)(s))
  #define pgm_read_byte(x) (*((uint8_t *)(x)))
  #define pgm_read_word(x) (*((uint16_t *)(x)))
  #define pgm_read_dword(x) (*((uint32_t *)(x)))
#endif

#ifndef BSS
  #define BSS __attribute__ ((__section__ (".bss")))
#endif

#endif /* _ARDUINO_H */
