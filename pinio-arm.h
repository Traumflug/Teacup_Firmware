/*
  pinio macros for ARM
*/

#if defined __ARMEL__

  #ifndef MASK
    /// MASKING- returns \f$2^PIN\f$
    #define MASK(PIN) (1 << PIN)
  #endif

  /**
    The LPC1114 supports bit-banding by mapping the bit mask to the address.
    See chapter 12 in the LPC111x User Manual. A read-modify-write cycle like
    on AVR costs 5 clock cycles, this implementation works with 3 clock cycles.
    Always assuming a well working optimiser.

    Reading and writing 12 bits would be sufficient. For reading we choose
    an uint32_t anyways, because casting to a smaller size is free, while
    doing the opposite requires zero-extending the value ("UXTH" in assembly).

    The macros here are a bit more complex, because arm-gcc lacks hardware
    support.
  */
  /// The bit-banding address.
  #define BITBAND(IO)      (IO ## _PORT + (MASK(IO ## _PIN) << 2))

  /// Read a pin.
  #define _READ(IO)        (*(volatile uint32_t *)BITBAND(IO))
  /// Write to a pin.
  #define _WRITE(IO, v)    do { if (v) { *(volatile uint16_t *)BITBAND(IO) = \
                                           MASK(IO ## _PIN); } \
                                else { *(volatile uint16_t *)BITBAND(IO) = 0; } \
                           } while (0)
  /// Enable pullup resistor.
  #define _PULLUP_ON(IO)   do { *(volatile uint32_t *)IO ## _IOREG = \
                                 (IO ## _OUTPUT | IO_MODEMASK_PULLUP); \
                          } while (0)
  /// Disable pullup resistor.
  #define _PULLUP_OFF(IO)  do { *(volatile uint32_t *)IO ## _IOREG = \
                                 (IO ## _OUTPUT | IO_MODEMASK_INACTIVE); \
                          } while (0)

  /**
    Set pins as input/output. On ARM, each pin has its own IOCON register,
    which allows to set its function and mode. We always set here standard
    GPIO behavior. Peripherals using these pins may have to change this and
    should do so in their own context.
  */
  /// Set pin as input.
  #define _SET_INPUT(IO)   do { *(volatile uint32_t *)IO ## _IOREG = \
                                  (IO ## _OUTPUT | IO_MODEMASK_REPEATER); \
                                *(volatile uint16_t *) \
                                  (IO ## _PORT + IO_DIR_OFFSET) &= \
                                  ~MASK(IO ## _PIN); \
                           } while (0)
  /// Set pin as output.
  #define _SET_OUTPUT(IO)  do { *(volatile uint32_t *)IO ## _IOREG = \
                                  IO ## _OUTPUT; \
                                *(volatile uint16_t *) \
                                  (IO ## _PORT + IO_DIR_OFFSET) |= \
                                  MASK(IO ## _PIN); \
                           } while (0)

#endif /* __ARMEL__ */
