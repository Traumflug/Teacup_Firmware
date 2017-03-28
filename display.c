
/** \file

  \brief Display broker.

  Here we map generic display calls to calls to the actually used display and
  also define functions common to all displays.
*/


#include "display.h"

// ***********************************************************************************
// This is where additional display drivers would be conditionally included/excluded
// currently hardwired to HD44780. should be easily changeable for additional display
// Just remember to treat all functions
// ***********************************************************************************

#ifdef DISPLAY

#include "ringbuffer.h"
#include "delay.h"

#include "display_hd44780.h"


const char greeting[21] PROGMEM = "Teacup Experimental ";
const char greet2[21]   PROGMEM = "2017/Mar/28 STECOR  ";

// Ringbuffer logic for buffer 'display'.
#define BUFSIZE DISPLAY_BUFFER_SIZE
volatile uint8_t displayhead = 0;
volatile uint8_t displaytail = 0;
volatile uint8_t displaybuf[BUFSIZE];

/* Custom characters defined in the first 8 characters of the LCD
Lifted directly from ultralcd_implementation_hitachi_HD44780.h in Marlin*/
const uint8_t refresh[8]		PROGMEM = {0x00,0x06,0x19,0x18,0x03,0x13,0x0C,0x00}; //thanks joris
const uint8_t bedTemp[8]		PROGMEM	= {0x00,0x1F,0x15,0x11,0x15,0x1F,0x00,0x00}; //thanks Sonny Mounicou
const uint8_t degree[8]			PROGMEM = {0x0C,0x12,0x12,0x0C,0x00,0x00,0x00,0x00};
const uint8_t thermometer[8]	PROGMEM = {0x04,0x0A,0x0A,0x0A,0x0A,0x11,0x11,0x0E};
const uint8_t uplevel[8]		PROGMEM = {0x04,0x0E,0x1F,0x04,0x1C,0x00,0x00,0x00}; //thanks joris
const uint8_t folder[8]			PROGMEM = {0x00,0x1C,0x1F,0x11,0x11,0x1F,0x00,0x00}; //thanks joris
const uint8_t feedrate[8]		PROGMEM = {0x1C,0x10,0x18,0x17,0x05,0x06,0x05,0x00}; //thanks Sonny Mounicou
const uint8_t clck[8]			PROGMEM = {0x00,0x0E,0x13,0x15,0x11,0x0E,0x00,0x00}; //thanks Sonny Mounicou

// Init display. Additional displays may be added
void display_init(void){
	HD44780_init();
	// Load our custom chars now
	HD44780_cust_ch(LCD_STR_REFRESH, refresh);
	HD44780_cust_ch(LCD_STR_BEDTEMP, bedTemp);
	HD44780_cust_ch(LCD_STR_DEGREE, degree);
	HD44780_cust_ch(LCD_STR_THERMOMETER, thermometer);
	HD44780_cust_ch(LCD_STR_UPLEVEL, uplevel);
	HD44780_cust_ch(LCD_STR_FOLDER, folder);
	HD44780_cust_ch(LCD_STR_FEEDRATE, feedrate);
	HD44780_cust_ch(LCD_STR_CLOCK, clck);
}

// We're just starting up, so we can be generous with the delays here... 
// really, we're just delaying initialization of the printer
// *** Note though - every time Pronterface or such like connect via serial
// the delays here add to the total time taken to connect, since connects
// reset the printer... 500 ms to admire the greeting is just fine :)
void display_greeting(void){			// This does NOT use the display message queue
	HD44780_clrscr();					
  
	display_writestr_P(greeting);    
	delay_ms(500);  					// Pause a bit...
}


/** Adds data to the display queue. Both commands and chars are sent using this
	First we check if the display buffer can take the data; if not, we need 
	space, so we call the dequeue routine - we only need one space in the queue
	*** remember, this is not in interrupt context, thus we can't have surprises :)
*/
void display_writechar(uint8_t data) {
  while ( !buf_canwrite(display)) 
    display_putchar();

    buf_push(display, data);
}

// Dequeue chars from the display queue and send them to the screen one at a time
void display_putchar(void){
	uint8_t data = 0;

	if (buf_canread(display)) {
		buf_pop(display, data);
		
	switch (data) {
		case LCD_CLEAR:
			HD44780_clrscr();
			break;
		case LCD_SET_CURSOR:				// Only 2-byte command
			buf_pop(display, data);
			HD44780_set_cursor(data);
			break;
		default: 							// Should be a printable character
			HD44780_char(data);
			break;
		}
  }
}


/**  Queue up a clear screen command. Avoid CLS in favor of blanking unused screen space
		(after all, we need to write something after a CLS, so blank unused space instead,
		that way it's not blocking)
*/
void display_clear(void) {
  display_writechar((uint8_t)LCD_CLEAR);
}

/** Sets the cursor to the given position.
  \param line   The vertical cursor position to set, in lines. First line is
                zero.
  \param column The horizontal cursor position to set. In characters on
                character based displays, in pixels on pixel based displays.
                First column is zero.
*/
void display_set_cursor(uint8_t line, uint8_t column) {
	uint8_t ln = 0;
	switch (line) {
	case 0:
		ln = 0x00;
		break;
	case 1:
		ln = 0x40;
		break;
	case 2:
		ln = 0x14;
		break;
	case 3:
		ln = 0x54;
	}
	ln += column;
	
	display_writechar(LCD_SET_CURSOR);
	display_writechar(ln);
}

// Write a string from FLASH into the display queue
// Use this instead of writing static content from RAM - very wasteful with our limited resources
void display_writestr_P(PGM_P data_P) {
  uint8_t r, i = 0;

  while ( (r = pgm_read_byte(&data_P[i])) ) {
    display_writechar(r);
    i++;
  }
}

#endif /* DISPLAY */
