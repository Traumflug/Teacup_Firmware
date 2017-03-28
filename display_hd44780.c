
/** \file
  \brief Code specific to the HD44780 display.
*/

#include "display_hd44780.h"
#include "delay.h"
#include "pinio.h"

#ifdef DISPLAY_TYPE_HD44780

/** \file
  \brief Parallel 4-bit subsystem

  This implements this custom interface often seen on LCD displays which
  uses 4 data lines (D4..D7) and 2 control lines:

  RS = Register Select: High for data input, Low for instruction input.
  E  = Enable: this line is pulsed to assert commands or data           
*/

static void e_pulse(void) {
  WRITE(DISPLAY_E_PIN, 1);
  delay_us(1);
  WRITE(DISPLAY_E_PIN, 0);
}

/**Send a byte to the display.
  \param data       The byte to be sent.
  \param rs   1 = Data
              0 = Command

  Other than other bus implementations we do not buffer here. Writing a byte
  takes just some 3 microseconds and there is nothing supporting such writes in
  hardware, so the overhead of buffering is most likely not worth the effort.
  
  *** Please use the HD44780_char(data)/HD44780_cmd(data) macros instead <- easier to comprehend
*/
void HD44780_write(uint8_t data, uint8_t rs) {
  WRITE(DISPLAY_RS_PIN, rs);            // Write data / instruction.
  // Output high nibble.
  WRITE(DISPLAY_D4_PIN, (data & 0x10) ? 1 : 0);
  WRITE(DISPLAY_D5_PIN, (data & 0x20) ? 1 : 0);
  WRITE(DISPLAY_D6_PIN, (data & 0x40) ? 1 : 0);
  WRITE(DISPLAY_D7_PIN, (data & 0x80) ? 1 : 0);
  e_pulse();
  // Output low nibble.
  WRITE(DISPLAY_D4_PIN, (data & 0x01) ? 1 : 0);
  WRITE(DISPLAY_D5_PIN, (data & 0x02) ? 1 : 0);
  WRITE(DISPLAY_D6_PIN, (data & 0x04) ? 1 : 0);
  WRITE(DISPLAY_D7_PIN, (data & 0x08) ? 1 : 0);
  e_pulse();
  delay_us(40); // <- CGRAM instruction exec time. Needed, or non-blocking delay for same amount of time needed in msg loop 
}

// create a custom char; max 8 can be defined
void HD44780_cust_ch(uint8_t ch_code, const uint8_t *charmap){
  HD44780_cmd(0x40 | (ch_code << 3));
  for (uint8_t i=0; i<8; i++)
	HD44780_char(pgm_read_byte(&charmap[i]));
}

// Initializes the display's controller 
void HD44780_init(void) {

  SET_OUTPUT(DISPLAY_RS_PIN);	WRITE(DISPLAY_RS_PIN, 0);
  SET_OUTPUT(DISPLAY_E_PIN);	WRITE(DISPLAY_E_PIN, 0);
  
  SET_OUTPUT(DISPLAY_D4_PIN);
  SET_OUTPUT(DISPLAY_D5_PIN);
  SET_OUTPUT(DISPLAY_D6_PIN);
  SET_OUTPUT(DISPLAY_D7_PIN);

  // Initial write is 8 bit; it is warmly recommended to read HD44780's datasheet
  WRITE(DISPLAY_D4_PIN, 1);
  WRITE(DISPLAY_D5_PIN, 1);
  WRITE(DISPLAY_D6_PIN, 0);
  WRITE(DISPLAY_D7_PIN, 0);
  
  delay_ms(16);					// Minimum initialisation time after power up.
  e_pulse();	delay_ms(5);	// Delay per datasheet
  // Repeat last command.
  e_pulse();	delay_us(120);  // Delay per datasheet
  // Repeat last command a third time.
  e_pulse();	delay_us(120);  // Delay per datasheet

  WRITE(DISPLAY_D4_PIN, 0); 	// Switch to 4 bit mode.
  e_pulse();	delay_us(120);	// Delay per datasheet
  
  HD44780_clrscr();				// clear display
  HD44780_cmd(0x06);			// Write left to right, no display shifting.
  HD44780_cmd(0x0C);			// Display ON, cursor not blinking.
  
}


// Clear Display
// **** This is an "expensive" command because it requires a blocking delay of 2000us else the display may corrupt
// since the per-char write time is ~50 us, a 4x20 display takes circa 4000us to over-write, However:
// 1. character writes do not require long blocking delays (only 40us per char)
// 2. we usually need to display data on the screen after a CLS, thus on a properly designed display screen
//    only 20-30 chars need to be blanked out, hence we're on par with CLS
// *** recommendation - DO NOT use CLS except for display Init (where the printer is idle anyway). For all other 
//     sreen updates, blank out unused space.
void HD44780_clrscr(void){
	HD44780_cmd(0x01);
	delay_ms(2);
}


// Move cursor to new position
void HD44780_set_cursor(uint8_t pos){
	HD44780_cmd(0x80 + pos);
}


#endif /* TEACUP_C_INCLUDE && DISPLAY_TYPE_HD44780 */
