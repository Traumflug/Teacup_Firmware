
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

  #elif defined DISPLAY_TYPE_LCD1302

    #error Display type LCD1302 not yet supported.

  #endif /* DISPLAY_TYPE_... */

#endif /* DISPLAY_BUS */


#define DISPLAY_BUFFER_SIZE 128


void display_init(void);
void display_tick(void);
void display_clear(void);

void display_greeting(void);
void display_clock(void);
void display_set_cursor(uint8_t line, uint8_t column);
void display_writechar(uint8_t data);

void display_writestr_P(PGM_P data_P);

#endif /* _DISPLAY_H */
