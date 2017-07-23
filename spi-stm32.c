
/** \file
  \brief SPI routines, STM32F411 specific part.
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32__

/** Initialise SPI subsystem.

20.3.3 Configuring the SPI in master mode
  In the master configuration, the serial clock is generated on the SCK pin.
  Procedure
  1. Select the BR[2:0] bits to define the serial clock baud rate (see SPI_CR1 register).
  2. Select the CPOL and CPHA bits to define one of the four relationships between the
    data transfer and the serial clock (see Figure 194). This step is not required when the
    TI mode is selected.
  3. Set the DFF bit to define 8- or 16-bit data frame format
  4. Configure the LSBFIRST bit in the SPI_CR1 register to define the frame format. This
    step is not required when the TI mode is selected.
  5. If the NSS pin is required in input mode, in hardware mode, connect the NSS pin to a
    high-level signal during the complete byte transmit sequence. In NSS software mode,
    set the SSM and SSI bits in the SPI_CR1 register. If the NSS pin is required in output
    mode, the SSOE bit only should be set. This step is not required when the TI mode is
    selected.
  6. Set the FRF bit in SPI_CR2 to select the TI protocol for serial communications.
  7. The MSTR and SPE bits must be set (they remain set only if the NSS pin is connected
    to a high-level signal).

  In this configuration the MOSI pin is a data output and the MISO pin is a data input.
  Transmit sequence
  The transmit sequence begins when a byte is written in the Tx Buffer.
  The data byte is parallel-loaded into the shift register (from the internal bus) during the first
  bit transmission and then shifted out serially to the MOSI pin MSB first or LSB first
  depending on the LSBFIRST bit in the SPI_CR1 register. The TXE flag is set on the transfer
  of data from the Tx Buffer to the shift register and an interrupt is generated if the TXEIE bit in
  the SPI_CR2 register is set.
  Receive sequence
  For the receiver, when data transfer is complete:
  • The data in the shift register is transferred to the RX Buffer and the RXNE flag is set
  • An interrupt is generated if the RXNEIE bit is set in the SPI_CR2 register
  At the last sampling clock edge the RXNE bit is set, a copy of the data byte received in the
  shift register is moved to the Rx buffer. When the SPI_DR register is read, the SPI
  peripheral returns this buffered value.
  Clearing the RXNE bit is performed by reading the SPI_DR register.
  A continuous transmit stream can be maintained if the next data to be transmitted is put in
  the Tx buffer once the transmission is started. Note that TXE flag should be ‘1 before any
  attempt to write the Tx buffer is made.
  Note: When a master is communicating with SPI slaves which need to be de-selected between
  transmissions, the NSS pin must be configured as GPIO or another GPIO must be used and
  toggled by software. 
*/

void set_spi_mode(uint8_t mode) {
  uint32_t temp_reg;
  temp_reg = SPI2->CR1;
  switch (mode) {
    case 0:
      temp_reg &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
      break;
    case 1:
      temp_reg &= ~(SPI_CR1_CPHA);
      temp_reg |= SPI_CR1_CPOL;
      break;
    case 2:
      temp_reg |= SPI_CR1_CPHA;
      temp_reg &= ~(SPI_CR1_CPOL);
      break;
    case 3:
      temp_reg |= SPI_CR1_CPHA | SPI_CR1_CPOL;
      break;
  }
  SPI2->CR1 = temp_reg;
}

void spi_init() {
  /*
    Set SCK (clock) and MOSI line to output.
    Setting this hard to SPI2. SPI1 and SPI4 are used for JTAG and other things.
    SPI2 is APB1-clock.
   
    SPI1, SPI4, SPI5  -> APB2 -> 100MHz
    SPI2, SPI3        -> APB1 -> 50MHz
  */

  // Enable SPI2 clock
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

  #define SCK_SPI2 PB_13
  #define MOSI_SPI2 PB_15
  #define MISO_SPI2 PB_14

  // Alternate function mapping for SPI2
  // DM00115249.pdf datasheet (page 47; table 9)
  SET_AFR(SCK_SPI2, 5);
  SET_AFR(MOSI_SPI2, 5);
  SET_AFR(MISO_SPI2, 5);

  // Alternate function mode
  SET_MODE(SCK_SPI2, 2);
  SET_MODE(MOSI_SPI2, 2);
  SET_MODE(MISO_SPI2, 2);

  // High speed == 3
  SET_OSPEED(SCK_SPI2, 3);
  SET_OSPEED(MOSI_SPI2, 3);
  SET_OSPEED(MISO_SPI2, 3);

  #ifdef TMC2130
    set_spi_mode(3);
  #else
    set_spi_mode(0);
  #endif

  spi_speed_100_400();

  // SSM:  software slave management (no NSS)
  // SSI:  internal slave select
  // MSTR: master mdoe
  // SPE:  enable SPI
  SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
  SPI2->CR1 |= SPI_CR1_SPE;
}

void spi_speed_100_400() {
  SPI2->CR1 |= SPI_CR1_BR;  // 1/256
}

void spi_speed_max() {
  SPI2->CR1 &= ~(SPI_CR1_BR);   // reset BR
  SPI2->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0;    // 1/16 -> SPI2 = 3.125MHz
}

#endif /* TEACUP_C_INCLUDE && defined __ARM_STM32__ */
