
#ifndef _FONT_H
#define _FONT_H

#include <stdint.h>
#include "arduino.h"  // For PROGMEM.

/**
  So far we have only one font and no choice for fonts in Configtool,
  so just paraphrase handling of distinct fonts.
*/

//#if defined DISPLAY_FONT_8X4

  #define FONT_ROWS             8
  #define FONT_COLUMNS          4
  #define FONT_SYMBOL_SPACE     1
  #define FONT_IS_PROPORTIONAL

//#elif defined DISPLAY_FONT_...another font

  // ... ...

//#endif

typedef struct {
  #ifdef FONT_IS_PROPORTIONAL
    uint8_t columns;
  #endif
  uint8_t data[FONT_COLUMNS];
} symbol_t;

extern const symbol_t PROGMEM font[];

#endif /* _FONT_H */
