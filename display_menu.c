#include "display_menu.h"
#include "key.h"
#include "display.h"
#include "sendf.h"
#include "sersendf.h"
#include "gcode_process.h"

typedef void (*func)(void);

typedef enum
{
  MAIN_MENU = 0x00,
    START_MENU,
      START,
      PAUSE,
      RESUME,
      STOP,
    COUNTER_MENU,
      RESET_COUNTER,
  NO_MENU
}
state_menu_t;

typedef struct
{
  int8_t row;     // 4 rows
  int8_t column;  // 128 columns
}
CURSOR_t;

typedef enum {
  DISPLAY_TEXT = 0x00,
  DISPLAY_FUNCTION
}
display_type_t;

typedef struct
{
  display_type_t type;
  func display_line;
  const char *text;
  CURSOR_t cursor;
  func menu_function;
  state_menu_t next;
}
MENU_t;

typedef struct
{
  MENU_t line[4];
}
DISPLAY_MENU_t;

void display_start(void) {
  sendf_P(display_writechar, PSTR("Start"));
}

// this code is optimized for font_8x8!
void display_speed(void) {
  uint16_t var = (next_target.target.f_multiplier * 25 + 12) / 64;  // unscale 256 == 100
  sendf_P(display_writechar, PSTR("S: "));
  if (var < 100)
    sendf_P(display_writechar, PSTR("  ")); // space = 2 columns, numbers have 6 columns - 1 'autospace'
  if (var < 10)
    sendf_P(display_writechar, PSTR("  "));
  sendf_P(display_writechar, PSTR("%u"), var);
}

void display_none(void) {}

const char Mainmenu_MSG [] PROGMEM = "Mainmenu";
const char Startmenu_MSG [] PROGMEM = "Startmenu";
const char none_MSG [] PROGMEM = "";

// this is our state machine for the complete display
// First element is the Display-type: DISPLAY_TEXT or DISPLAY_FUNCTION
// second is 0 for DISPLAY_TEXT or the name of the function
// third is 0 for DISPLAY_FUNCTION or the variable name for the text in PROGMEM
// fourth is for setting up the cursor. 
//        First element is -1 for automatic (0-3) or a number which will fit in the display
//        Second element is between 0 and 127 for a 128x-type display
// fifth element could be later used for a user defined function which could exetuded when 'clicked'
// last element is the next menu when 'clicked'
const DISPLAY_MENU_t PROGMEM state_menu_table[] = {
  {
    { // Mainmenu
      {DISPLAY_TEXT,      0,              Mainmenu_MSG,     {0, 0},   0, START_MENU},
      {DISPLAY_FUNCTION,  display_speed,  0,                {-1, 0},  0, START_MENU},
      {DISPLAY_FUNCTION,  display_start,  0,                {-1, 10}, 0, START_MENU},
      {DISPLAY_TEXT,      0,              none_MSG,         {3, 10},  0, NO_MENU}
    }
  },
  {
    {
      {DISPLAY_TEXT,      0,              Startmenu_MSG,    {0, 0},   0, START_MENU},
      {DISPLAY_FUNCTION,  display_speed,  0,                {-1, 0},  0, START_MENU},
      {DISPLAY_FUNCTION,  display_start,  0,                {-1, 10}, 0, START_MENU},
      {DISPLAY_TEXT,      0,              none_MSG,         {3, 10},  0, NO_MENU}
    }
  }
};

void display_menu(state_menu_t state) {
  uint8_t i;
  int8_t row, column;
  for (i = 0; i < 4; i++) {
    row = pgm_read_byte(&state_menu_table[state].line[i].cursor.row);
    column = pgm_read_byte(&state_menu_table[state].line[i].cursor.column);
    if (row >= 0)
      display_set_cursor(row, column);
    else
      display_set_cursor(i, column);
    if (pgm_read_byte(&state_menu_table[state].line[i].type) == DISPLAY_TEXT) {
      display_writestr_P((char *)pgm_read_word(&state_menu_table[state].line[i].text));
    }
    else if (pgm_read_byte(&state_menu_table[state].line[i].type) == DISPLAY_FUNCTION) {
      ((void (*)(void))(pgm_read_ptr(&state_menu_table[state].line[i].display_line)))();
    }
  }
}

void display_menu_clock() {
  if_key(KEY_5)
    display_menu(MAIN_MENU);
  else
    display_menu(START_MENU);
}
