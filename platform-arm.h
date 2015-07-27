#ifdef __ARMEL__

/**
  On ARM these attributes are simple types and type-casts.
*/

#define PROGMEM
#define PGM_P const char *
#define PSTR(s) ((const PROGMEM char *)(s))
#define pgm_read_byte(x) (*((uint8_t *)(x)))
#define pgm_read_word(x) (*((uint16_t *)(x)))
#define pgm_read_dword(x) (*((uint32_t *)(x)))

/*
  Include appropriate header for target chip.
*/
#if defined (__ARM_LPC1114__)
  #include "arduino_lpc1114.h"
#endif

#endif /* __ARMEL__ */
