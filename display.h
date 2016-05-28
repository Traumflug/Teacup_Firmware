
/** \file

  \brief Display broker.

  Here we map generic display calls to calls to the actually used display.
*/

#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <stdint.h>
#include "config_wrapper.h"
#include "displaybus.h"

#ifdef DISPLAY_BUS

  #if defined DISPLAY_TYPE_SSD1306

    #define DISPLAY_I2C_ADDRESS         0x78
    #define DISPLAY_LINES               4
    #define DISPLAY_SYMBOLS_PER_LINE    32

    #define DISPLAY

  #elif defined DISPLAY_TYPE_HD44780

    #define DISPLAY_LINES               2
    #define DISPLAY_SYMBOLS_PER_LINE    16

    #define DISPLAY

  #else

    #error Display type not yet supported.

  #endif /* DISPLAY_TYPE_... */

#endif /* DISPLAY_BUS */


#define DISPLAY_BUFFER_SIZE 128

/**
  Printable ASCII characters and our embedded fonts start at 0x20, so we can
  use 0x00..0x1F for storing control commands in the character queue. That's
  what genuine ASCII does, too, we just use our own code set.

  Queueing up actions together with actual characters not only postpones
  these actions to idle time, it's also necessary to keep them in the right
  order. Without it, writing a few characters, moving the cursor elsewhere and
  writing even more characters would result in all characters being written to
  the second position, because characters would wait in the queue while cursor
  movements were executed immediately.
*/
enum display_low_code {
  low_code_clear = 0x01,
  low_code_set_cursor
};

void display_init(void);
void display_tick(void);
void display_clear(void);

void display_greeting(void);
void display_clock(void);
void display_set_cursor(uint8_t line, uint8_t column);
void display_writechar(uint8_t data);

void display_writestr_P(PGM_P data_P);

#endif /* _DISPLAY_H */
