
#ifndef _DISPLAY_SSD1306_H
#define _DISPLAY_SSD1306_H

#include "config_wrapper.h"


#ifdef DISPLAY_TYPE_SSD1306

#define DISPLAY
#define DISPLAY_I2C_ADDRESS        (0x3C << 1)


#define DISPLAY_LINES              4
#define DISPLAY_SYMBOLS_PER_LINE  16
#define HOTENDS_COUNT              1
#define HOTBED_ZONES               1

#define DISPLAY_PLACE_HOTEND       0, 0, 8
#define DISPLAY_PLACE_HOTBED       0, 8, 8
#define DISPLAY_PLACE_X            1, 0, 5
#define DISPLAY_PLACE_Y            1, 5, 5
#define DISPLAY_PLACE_Z            1, 10, 5
#define DISPLAY_PLACE_FEEDRATE     2, 0, 5
#define DISPLAY_PLACE_PROGRESS     2, 5, 5
#define DISPLAY_PLACE_TIMER        2, 10, 6
#define DISPLAY_PLACE_STATUS       3, 0, 16

typedef struct {
  uint16_t hotend_temp[HOTENDS_COUNT];
  uint16_t hotbed_temp[HOTBED_ZONES];
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint16_t feedrate;
  uint8_t progress; // in percents
  uint32_t timer; // in seconds
} DISPLAY_T;


void display_init(void);
void display_on(void);
void display_hotend(uint8_t index);
void display_hotbed(uint8_t index);
void display_point(uint16_t x, uint16_t y, uint16_t z);
void display_feedrate(uint16_t value);
void display_progress(uint8_t value);
void display_timer(uint32_t value);
void display_off(void);

void display_clear(void);
void display_text_P(uint8_t line, uint8_t column, PGM_P message_P);

#endif /* DISPLAY_TYPE_SSD1306 */

#endif /* _DISPLAY_SSD1306_H */
