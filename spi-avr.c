
/** \file
  \brief SPI routines, AVR specific part.
*/

#if defined TEACUP_C_INCLUDE && defined __AVR__

/** Initialise SPI subsystem.

  Code copied from ATmega164/324/644/1284 data sheet, section 18.2, page 160,
  or moved here from mendel.c.
*/
void spi_init() {

  // Set SCK (clock) and MOSI line to output, ie. set USART in master mode.
  SET_OUTPUT(SCK);
  SET_OUTPUT(MOSI);
  SET_INPUT(MISO);
  // SS must be set as output to disconnect it from the SPI subsystem.
  // Too bad if something else tries to use this pin as digital input.
  // See ATmega164/324/644/1284 data sheet, section 18.3.2, page 162.
  // Not written there: this must apparently be done before setting the SPRC
  // register, else future R/W-operations may hang.
  SET_OUTPUT(SS);

  #ifdef SPI_2X
    SPSR = 0x01;
  #else
    SPSR = 0x00;
  #endif

  // This sets the whole SPRC register.
  spi_speed_100_400();
}

#endif /* defined TEACUP_C_INCLUDE && defined __AVR__ */
