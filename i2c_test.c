
/**
  Quick test for sending some visible data to a SSD1306 display connected
  via I2C. This means not to test the display, but the I2C implementation.

  To run this test, add this line to board or printer config.h:

    #define I2C

  With this done, a welcome message should appear on the display. Without
  these additions, the binary size should be as small as the commit before.
*/

#ifdef I2C_TEST

#include <string.h>
#include "config_wrapper.h"
#include "displaybus.h"
#include "font.h"


#define DISPLAY_I2C_ADDRESS        (0x3C << 1)

const uint8_t PROGMEM display_init[] = {
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

static void i2c_test(void) {
  uint16_t i;
  const char* message = "Welcome to Teacup";

  for (i = 0; i < sizeof(display_init); i++) {
    // Send last byte with 'last_byte' set.
    displaybus_write(pgm_read_byte(&display_init[i]),
                     (i == sizeof(display_init) - 1));
  }

  /**
    Clear the screen. As this display supports many sophisticated commands,
    but not a simple 'clear', we have to overwrite the entire memory with
    zeros, byte by byte.
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

  /**
    Setup cursor on display.

    "Welcome to Teacup" is 64 pixel columns wide, entire display is
    128 columns, so we offset by 32 columns to get it to the center.
  */
  displaybus_write(0x00, 0);
  // Line 1.
  displaybus_write(0xB0 | 1, 0);
  // Column 32.
  displaybus_write(0x00 | (32 & 0x0F), 0);
  displaybus_write(0x10 | ((32 >> 4) & 0x0F), 1);

  // Render text to bitmap to display.
  displaybus_write(0x40, 0);
  while (*message) {
    uint8_t index = (uint8_t)*message - 0x20;

    // Send the character bitmap.
    for (i = 0; i < pgm_read_byte(&font[index].columns); i++) {
      displaybus_write(pgm_read_byte(&font[index].data[i]), 0);
    }
    // Send space between characters.
    for (i = 0; i < FONT_SYMBOL_SPACE; i++) {
      displaybus_write(0x00, 0);
    }

    message++;
  }
  // Send another space for transmission end.
  displaybus_write(0x00, 1);
}

#endif /* I2C_TEST */
