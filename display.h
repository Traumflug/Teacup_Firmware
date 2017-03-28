
/** \file
  \brief Display broker.

  Here we map generic display calls to calls to the actually used display.
*/

#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <stdint.h>
#include "config_wrapper.h"

#define DISPLAY_BUFFER_SIZE 128	// Defines a display queue. 

// Meta commands - we're hijacking the below characters to send LCD clear & SET CURSOR commands via 
//					display queue
#define LCD_CLEAR 		0xFE
#define LCD_SET_CURSOR 	0xFD

// Custom characters defined in the first 8 characters of the LCD
// directly from ultralcd_implementation_hitachi_HD44780.h in Marlin - thank you!
#define LCD_STR_REFRESH     0x00			// Do not use in strings. Only as a char by itself!!!
#define LCD_STR_BEDTEMP     0x01
#define LCD_STR_DEGREE      0x02
#define LCD_STR_THERMOMETER 0x03
#define LCD_STR_UPLEVEL     0x04
#define LCD_STR_FOLDER      0x05
#define LCD_STR_FEEDRATE    0x06
#define LCD_STR_CLOCK       0x07
#define LCD_STR_ARROW_RIGHT 0x7E  /* from the default character set */


//void display_clock(void);					// this used to be the screen refresh - now it is in lcd_menu
//void display_tick(void);					// also moved to lcd_menu

void display_init(void);					// display init - calls device speciffic init
void display_greeting(void);				// very spartan greeting

void display_writechar(uint8_t data); 		// enqueues a char/cmd into the display queue
void display_putchar(void);					// de-queues the char/cmds from the queue to the display 

void display_clear(void);					// try to avoid when possible - expensive command. requires 2ms blocking delay
void display_set_cursor(uint8_t line, uint8_t column);// set display cursor
void display_writestr_P(PGM_P data_P);		// writes a string from FLASH to the queue

#endif /* _DISPLAY_H */
