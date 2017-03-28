#ifndef _DISPLAY_HD44780_H
#define _DISPLAY_HD44780_H

#include "config_wrapper.h"

// Check for the necessary pins.
	#ifndef DISPLAY_RS_PIN
	  #error DISPLAY_TYPE_HD44780 defined, but not DISPLAY_RS_PIN.
	#endif
	#ifndef DISPLAY_E_PIN
	  #error DISPLAY_TYPE_HD44780 defined, but not DISPLAY_E_PIN.
	#endif
	#ifndef DISPLAY_D4_PIN
	  #error DISPLAY_TYPE_HD44780 defined, but not DISPLAY_D4_PIN.
	#endif
	#ifndef DISPLAY_D5_PIN
	  #error DISPLAY_TYPE_HD44780 defined, but not DISPLAY_D5_PIN.
	#endif
	#ifndef DISPLAY_D6_PIN
	  #error DISPLAY_TYPE_HD44780 defined, but not DISPLAY_D6_PIN.
	#endif
	#ifndef DISPLAY_D7_PIN
	  #error DISPLAY_TYPE_HD44780 defined, but not DISPLAY_D7_PIN.
	#endif

#define DISPLAY_LINES               4							// currently unused
#define DISPLAY_SYMBOLS_PER_LINE    20							// unused as well

// Interface functions. The .c file should never be included in other project files

void HD44780_init(void);										// Display init
void HD44780_cust_ch(uint8_t ch_code, const uint8_t *charmap);	// Define custom character
void HD44780_clrscr(void);										// Clear screen - try to avoid usage. Look at description in C file
void HD44780_set_cursor(uint8_t pos);							// Set cursor position

void HD44780_write(uint8_t data, uint8_t rs);					// do not use (for clarity reasons); use macros below instead
#define HD44780_char(data) HD44780_write(data, 1)				// send a character to the display
#define HD44780_cmd(cmd) HD44780_write(cmd, 0)					// send a command to the display

#endif /* _DISPLAY_HD44780_H */