/*
  pinio macros for AVR
*/

#if defined __AVR__

  #ifndef MASK
    /// MASKING- returns \f$2^PIN\f$
    #define MASK(PIN) (1 << PIN)
  #endif

  #include <avr/io.h>

  /// Read a pin.
  #define _READ(IO)        (IO ## _RPORT & MASK(IO ## _PIN))
  /// Write to a pin.
  #define _WRITE(IO, v)    do { if (v) { IO ## _WPORT |= MASK(IO ## _PIN); } \
                                else { IO ## _WPORT &= ~MASK(IO ## _PIN); } \
                           } while (0)
  /// Enable pullup resistor.
  #define _PULLUP_ON(IO)   _WRITE(IO, 1)
  /// Disable pullup resistor.
  #define _PULLUP_OFF(IO)  _WRITE(IO, 0)

  /**
    Setting pins as input/output: other than with ARMs, function of a pin
    on AVR isn't given by a dedicated function register, but solely by the
    on-chip peripheral connected to it. With the peripheral (e.g. UART, SPI,
    ...) connected, a pin automatically serves with this function. With the
    peripheral disconnected, it automatically returns to general I/O function.
  */
  /// Set pin as input.
  #define _SET_INPUT(IO)   do { IO ## _DDR &= ~MASK(IO ## _PIN); } while (0)
  /// Set pin as output.
  #define _SET_OUTPUT(IO)  do { IO ## _DDR |=  MASK(IO ## _PIN); } while (0)

#endif /* __AVR__ */
