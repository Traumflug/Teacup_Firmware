
/** \file

  \brief Code specific to the SSD1306 display.
*/

/**
  D'oh. Shortly before completing this code, the display hardware died. Shows
  just black, for whatever reason. Accordingly I can't test new code any longer
  and writing code without seeing what it does makes no sense.

  What already works:

    - I2C with a queue for small transmissions. Sufficient to queue up sending
      a rendered character. It's filled by displaybus_write() and drained by
      the I2C interrupt. Larger transmissions are handled fine, too, but cause
      wait cycles.

    - 128 byte queue holding characters to send. This queue is filled by
      display_writechar(). It's drained by display_tick(), which processes,
      renders and forwards these characters to the I2C queue.

    - Display initialisation.

    - Clearing the display.

    - Writing text with display_writestr_P().

    - Writing formatted text with sendf_P(display_writechar, ...).

    - Current state of code should clear the display at startup, show a
      greeting message and start displaying current X/Y/Z coordinates, updated
      once per second. All this not in a particularly pretty fashion, but
      working.

  TODO list:

    - Lot's of prettification. Like a nice background picture with the Teacup
      logo, like "Welcome to Teacup" as a greeting screen, like writing numbers
      to readable places and so on.

    - Allow different fonts. Already paraphrased in font.h and font.c. Needs
      a selection menu in Configtool, of course, the same way one can select
      display types.

    - It's a bit unclear wether this 'last_byte' flag to displaybus_write() is
      really ideal. Fact is, I2C transmissions need a start and an explicite
      ending. Also thinkable would be a displaybus_finalise() function
      which puts the marker in place. Saves a lot of shuffling parameters
      around.

      Yet another option would be to make sure the I2C send buffer is drained
      befpre sending the next transmission. I2C code already finalises a
      transmission on buffer drain, so only _reliable_ waiting needs an
      implementation.

      Each variant needs testing, which one gets away with the smallest code.
      Smallest code is likely the fastest code as well.

    - Here's an assistant to convert pictures/bitmaps into C code readable by
      the compiler: http://en.radzio.dxp.pl/bitmap_converter/
*/

#include "display.h"

#if defined TEACUP_C_INCLUDE && defined DISPLAY_TYPE_SSD1306

#include "displaybus.h"
#include "font.h"
#include "sendf.h"
#include "delay.h"
#include "dda.h"


static const uint8_t PROGMEM init_sequence[] = {
  0x00,             // Command marker.
  0xAE,             // Display off.
  0xD5, 0x80,       // Display clock divider (reset).
  0xA8, 0x1F,       // 1/32 duty.
  0x40 | 0x00,      // Start line (reset).
  0x20, 0x02,       // Page addressing mode (reset).
  0x22, 0x00, 0x03, // Start and end page in horiz./vert. addressing mode[1].
  0x21, 0x00, 0x7F, // Start and end column in horiz./vert. addressing mode.
  0xA0 | 0x00,      // No segment remap (reset).
  0xC0 | 0x00,      // Normal com pins mapping (reset).
  0xDA, 0x02,       // Sequental without remap com pins.
  0x81, 0x7F,       // Contrast (reset).
  0xDB, 0x20,       // Vcomh (reset).
  0xD9, 0xF1,       // Precharge period.
  0x8D, 0x14,       // Charge pump.
  0xA6,             // Positive display.
  0xA4,             // Resume display.
  0xAF              // Display on.
};
// [1] Do not set this to 0x00..0x07 on a 32 pixel high display, or vertical
//     addressing mode will mess up. 32 pixel high displays have only 4 pages
//     (0..3), still addressing logic accepts, but can't deal with the 0..7
//     meant for 64 pixel high displays.

/**
 * Initializes the display's controller configuring the way of
 * displaying data.
 */
void display_init(void) {
  uint8_t i;

  displaybus_init(DISPLAY_I2C_ADDRESS);

  for (i = 0; i < sizeof(init_sequence); i++) {
    // Send last byte with 'last_byte' set.
    displaybus_write(init_sequence[i], (i == sizeof(init_sequence) - 1));
  }
}

/**
  Show a nice greeting. Pure eye candy.
*/
void display_greeting(void) {

  display_clear();

  /**
    "Welcome to Teacup" is 64 pixel columns wide, entire display is
    128 columns, so we offset by 32 columns to get it to the center.
  */
  display_set_cursor(1, 32);

  display_writestr_P(PSTR("Welcome to Teacup"));

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

  display_set_cursor(0, 2);
  update_current_position();
  sendf_P(display_writechar, PSTR("X:%lq Y:%lq Z:%lq  F:%lu  "),
          current_position.axis[X], current_position.axis[Y],
          current_position.axis[Z], current_position.F);
}

/**
  Forwards a character or a control command from the display queue to the I2C
  queue.
*/
void display_tick() {
  uint16_t i, data, index;

  if (displaybus_busy()) {
    return;
  }

  /**
    Possible strategy for error recovery: after a failed, aborted I2C
    transmisson, 'i2c_state & I2C_INTERRUPTED' in i2c.c evaluates to true.

    Having a getter like displaybus_failed() would allow to test this condition
    here, so we could resend the previous data again, instead of grabbing a
    new byte from the buffer.
  */

  if (buf_canread(display)) {
    buf_pop(display, data);
    switch (data) {
      case low_code_clear:
        /**
          Clear the screen. As this display supports many sophisticated
          commands, but not a simple 'clear', we have to overwrite the entire
          memory with zeros, byte by byte.
        */
        // Set horizontal adressing mode.
        displaybus_write(0x00, 0);
        displaybus_write(0x20, 0);
        displaybus_write(0x00, 1);

        // Write 512 zeros.
        displaybus_write(0x40, 0);
        for (i = 0; i < 512; i++) {
          displaybus_write(0x00, (i == 511));
        }

        // Return to page adressing mode.
        displaybus_write(0x00, 0);
        displaybus_write(0x20, 0);
        displaybus_write(0x02, 1);
        break;

      case low_code_set_cursor:
        /**
          Set the cursor to the given position.

          This is a three-byte control command, so we fetch additional bytes
          from the queue and cross fingers they're actually there.
        */
        // Enter command mode.
        displaybus_write(0x00, 0);
        // Set line.
        buf_pop(display, data);
        displaybus_write(0xB0 | (data & 0x03), 0);
        // Set column.
        buf_pop(display, data);
        displaybus_write(0x00 | (data & 0x0F), 0);
        displaybus_write(0x10 | ((data >> 4) & 0x0F), 1);
        break;

      default:
        // Should be a printable character.
        index = data - 0x20;

        // Write pixels command.
        displaybus_write(0x40, 0);

        // Send the character bitmap.
        #ifdef FONT_IS_PROPORTIONAL
          for (i = 0; i < pgm_read_byte(&font[index].columns); i++) {
        #else
          for (i = 0; i < FONT_COLUMNS; i++) {
        #endif
            displaybus_write(pgm_read_byte(&font[index].data[i]), 0);
        }
        // Send space between characters.
        for (i = 0; i < FONT_SYMBOL_SPACE; i++) {
          displaybus_write(0x00, (i == FONT_SYMBOL_SPACE - 1));
        }
        break;
    }
  }
}

#endif /* TEACUP_C_INCLUDE && DISPLAY_TYPE_SSD1306 */
