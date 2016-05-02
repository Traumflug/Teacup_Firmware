
/** \file

  \brief Code specific to the HD44780 display.
*/

/**
  TODO list:

    - Procedures like display_clear() and display_set_cursor() should be queued
      up, too. Just like characters. Fonts start at 0x20, so 0x00..0x1F are
      available for command sequences. For example, setting the cursor could
      queue up 0x04 0x01 0x20 (3 bytes) to set the cursor to line 1, column 32.
      0x04 is the "command", bytes are queued up with display_writechar().

      This is necessary to enforce characters and cursor commands to happen in
      the right order. Currently, writing a few characters, moving the cursor
      elsewhere and writing even more characters results in all characters
      being written to the second position, because characters wait in the
      queue, while cursor movements are executed immediately.

      Code currently in display_set_cursor() would move to display_tick(), then.
*/

#include "display.h"

#if defined TEACUP_C_INCLUDE && defined DISPLAY_TYPE_HD44780

#include "displaybus.h"
#include "delay.h"
#include "sendf.h"
#include "dda.h"


/**
 * Initializes the display's controller configuring the way of
 * displaying data.
 */
void display_init(void) {

  // Minimum initialisation time after power up.
  delay_ms(15);

  displaybus_init(0);

  // Write left to right, no display shifting.
  displaybus_write(0x06, parallel_4bit_instruction);
  // Display ON, cursor not blinking.
  displaybus_write(0x0C, parallel_4bit_instruction);
}

/**
  Clear the screen. This display has a dedicated command for doing it.
*/
void display_clear(void) {
  displaybus_write(0x01, parallel_4bit_instruction);
}

/**
  Sets the cursor to the given position.

  \param line   The vertical cursor position to set, in lines. First line is
                zero. Line height is character height, which is currently
                fixed to 8 pixels.

  \param column The horizontal cursor position to set, in pixels. First
                column is zero.

  Use this for debugging purposes, only. Regular display updates happen in
  display_clock().
*/
void display_set_cursor(uint8_t line, uint8_t column) {
  // Currently unimplemented.
}

/**
  Show a nice greeting. Pure eye candy.
*/
void display_greeting(void) {

  display_clear();

  // We have only 16 characters at our disposal ...
  display_writestr_P(PSTR("Welcome @ Teacup"));

  // Forward this to the display immediately.
  while (buf_canread(display)) {
    display_tick();
  }

  // Allow the user to worship our work for a moment :-)
  delay_ms(5000);
}

/**
  Regular update of the display. Typically called once a second from clock.c.
*/
void display_clock(void) {

  display_clear();

  update_current_position();
  sendf_P(display_writechar, PSTR("X:%lq Y:%lq"),
          current_position.axis[X], current_position.axis[Y]);
}

/**
  Forwards a character from the display queue to display bus. As this is a
  character based display it's easy.
*/
void display_tick() {
  uint8_t data;

  if (displaybus_busy()) {
    return;
  }

  if (buf_canread(display)) {
    buf_pop(display, data);
    displaybus_write(data, parallel_4bit_data);
  }
}

#endif /* TEACUP_C_INCLUDE && DISPLAY_TYPE_HD44780 */
