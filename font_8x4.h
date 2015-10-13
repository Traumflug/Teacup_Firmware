
#ifndef _FONT_8x4_H
#define _FONT_8x4_H

#include <stdint.h>
#include "arduino.h"  // For PROGMEM.

#define FONT_ROWS 8
#define FONT_COLUMNS 4
#define FONT_SYMBOL_SPACE 1

typedef struct {
  uint8_t columns;
  uint8_t data[FONT_COLUMNS];
} symbol_t;

extern const symbol_t PROGMEM font_8x4[];

#endif /* _FONT_8x4_H */
