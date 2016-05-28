
/** \file

  \brief Code specific to the HD44780 display.
*/

#include "display.h"

#if defined TEACUP_C_INCLUDE && defined DISPLAY_TYPE_HD44780

#include "displaybus.h"
#include "delay.h"
#include "sendf.h"
#include "dda.h"
#include "temp.h"


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
  uint16_t temperature;

  display_clear();

  update_current_position();
  sendf_P(display_writechar, PSTR("X:%lq Y:%lq"),
          current_position.axis[X], current_position.axis[Y]);

  #ifdef HEATER_EXTRUDER
    display_set_cursor(1, 0);
    temperature = temp_get(TEMP_SENSOR_extruder);
    sendf_P(display_writechar, PSTR("%u.%su"),
            temperature >> 2, (uint8_t)(temperature & 3) * 25);
  #endif
  #ifdef HEATER_BED
    display_set_cursor(1, DISPLAY_SYMBOLS_PER_LINE / 2);
    temperature = temp_get(TEMP_SENSOR_bed);
    sendf_P(display_writechar, PSTR("%u.%su"),
            temperature >> 2, (uint8_t)(temperature & 3) * 25);
  #endif
}

/**
  Forwards a character or a control command from the display queue to display
  bus. As this is a character based display it's easy.
*/
void display_tick() {
  uint8_t data, command;

  if (displaybus_busy()) {
    return;
  }

  if (buf_canread(display)) {
    buf_pop(display, data);
    switch (data) {
      case low_code_clear:
        displaybus_write(0x01, parallel_4bit_instruction);
        break;

      case low_code_set_cursor:
        /**
          Set the cursor to the given position.

          This is a three-byte control command, so we fetch additional bytes
          from the queue and cross fingers they're actually there.
        */
        command = 0x80;    // "Set DDRAM Address" base command.

        /**
          Add address of line.

          As we have two lines only, this can be "calculated" without
          a multiplication.
        */
        buf_pop(display, data);
        if (data) {
          command += 0x40;
        }

        // Add column address.
        buf_pop(display, data);
        command += data;

        displaybus_write(command, parallel_4bit_instruction);
        break;

      default:
        // Should be a printable character.
        displaybus_write(data, parallel_4bit_data);
        break;
    }
  }
}

#endif /* TEACUP_C_INCLUDE && DISPLAY_TYPE_HD44780 */
