
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

  #elif defined DISPLAY_TYPE_LCD1302

    #error Display type LCD1302 not yet supported.

  #endif /* DISPLAY_TYPE_... */

#endif /* DISPLAY_BUS */


void display_init(void);
void display_clear(void);
void display_text_P(uint8_t line, uint8_t column, PGM_P message_P);

#endif /* _DISPLAY_H */
