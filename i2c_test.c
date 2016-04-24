
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
#include "display.h"
#include "font.h"


static void i2c_test(void) {
  /**
    "Welcome to Teacup" is 64 pixel columns wide, entire display is
    128 columns, so we offset by 32 columns to get it to the center.
  */
  display_set_cursor(1, 32);
  display_writestr_P(PSTR("Welcome to Teacup"));
}

#endif /* I2C_TEST */
