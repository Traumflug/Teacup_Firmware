
/** \file

  \brief Display broker.

  Here we map generic display calls to calls to the actually used display and
  also define functions common to all displays.
*/

#include "display.h"

#ifdef DISPLAY

// Ringbuffer logic for buffer 'display'.
#define BUFSIZE DISPLAY_BUFFER_SIZE

volatile uint8_t displayhead = 0;
volatile uint8_t displaytail = 0;
volatile uint8_t displaybuf[BUFSIZE];

#include "ringbuffer.h"


#define TEACUP_C_INCLUDE
  #include "display_ssd1306.c"
  #include "display_hd44780.c"
#undef TEACUP_C_INCLUDE


#include "delay.h"

/**
  Queue up a clear screen command. Very cheap operation on some displays, like
  the HD44780, rather expensive on others, like the SSD1306.
*/
void display_clear(void) {
  display_writechar((uint8_t)low_code_clear);
}

/**
  Sets the cursor to the given position.

  \param line   The vertical cursor position to set, in lines. First line is
                zero.

  \param column The horizontal cursor position to set. In characters on
                character based displays, in pixels on pixel based displays.
                First column is zero.

  Use this directly for debugging purposes, only. Regular display updates
  happen in display_clock().
*/
void display_set_cursor(uint8_t line, uint8_t column) {
  display_writechar((uint8_t)low_code_set_cursor);
  display_writechar((uint8_t)line);
  display_writechar((uint8_t)column);
}

/**
  Prints a character at the current cursor position.

  \param data The character to be displayed.

  This code is identical for all display buses and display types, because it
  just queues up the character.

  In case the buffer is full already it waits for a millisecond to allow
  data to be sent to the display, then it tries again. If it still fails then,
  it drops the character. This way we're fairly protected against data loss,
  still we guarantee to not hang forever.
*/
void display_writechar(uint8_t data) {

  if ( ! buf_canwrite(display)) {
    delay_ms(1);
  }
  if (buf_canwrite(display)) {
    buf_push(display, data);
  }
}

void display_writestr_P(PGM_P data_P) {
  uint8_t r, i = 0;

  // Yes, this is *supposed* to be assignment rather than comparison, so we
  // break when r is assigned zero.
  while ((r = pgm_read_byte(&data_P[i]))) {
    display_writechar(r);
    i++;
  }
}

#endif /* DISPLAY */
