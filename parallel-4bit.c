
/** \file
  \brief Parallel 4-bit subsystem

  This implements this custom interface often seen on LCD displays which
  uses 4 data lines (D0..D3) and 3 control lines:

    RS = Register Select: High for data input, Low for instruction input.

    RW = Read/Write:      High for read, Low for write.

    E = Enable:           A line for triggering a read or a write. Its detailed
                          usage is a bit complicated.

    This implementation was written in the hope to be exchangeable with using
    I2C or SPI as display bus for the same display.

    Other than the I2C implementation, this one uses no send buffer. Bytes can
    be sent quickly enough to allow sending them immediately.
*/

#include "parallel-4bit.h"

#ifdef DISPLAY_BUS_4BIT

#include "delay.h"
#include "pinio.h"

// Check for the necessary pins.
#ifndef DISPLAY_RS_PIN
  #error DISPLAY_BUS_4BIT defined, but not DISPLAY_RS_PIN.
#endif
#ifndef DISPLAY_RW_PIN
  #error DISPLAY_BUS_4BIT defined, but not DISPLAY_RW_PIN.
#endif
#ifndef DISPLAY_E_PIN
  #error DISPLAY_BUS_4BIT defined, but not DISPLAY_E_PIN.
#endif
#ifndef DISPLAY_D4_PIN
  #error DISPLAY_BUS_4BIT defined, but not DISPLAY_D4_PIN.
#endif
#ifndef DISPLAY_D5_PIN
  #error DISPLAY_BUS_4BIT defined, but not DISPLAY_D5_PIN.
#endif
#ifndef DISPLAY_D6_PIN
  #error DISPLAY_BUS_4BIT defined, but not DISPLAY_D6_PIN.
#endif
#ifndef DISPLAY_D7_PIN
  #error DISPLAY_BUS_4BIT defined, but not DISPLAY_D7_PIN.
#endif


static void e_pulse(void) {
  WRITE(DISPLAY_E_PIN, 1);
  delay_us(1);
  WRITE(DISPLAY_E_PIN, 0);
}

/**
  Inititalise the subsystem.
*/
void parallel_4bit_init(void) {

  SET_OUTPUT(DISPLAY_RS_PIN);
  WRITE(DISPLAY_RS_PIN, 0);

  SET_OUTPUT(DISPLAY_RW_PIN);
  WRITE(DISPLAY_RW_PIN, 0);

  SET_OUTPUT(DISPLAY_E_PIN);
  WRITE(DISPLAY_E_PIN, 0);

  /**
    Perform a reset.
  */
  SET_OUTPUT(DISPLAY_D4_PIN);
  SET_OUTPUT(DISPLAY_D5_PIN);
  SET_OUTPUT(DISPLAY_D6_PIN);
  SET_OUTPUT(DISPLAY_D7_PIN);

  // Initial write is 8 bit.
  WRITE(DISPLAY_D4_PIN, 1);
  WRITE(DISPLAY_D5_PIN, 1);
  WRITE(DISPLAY_D6_PIN, 0);
  WRITE(DISPLAY_D7_PIN, 0);
  e_pulse();
  delay_ms(5);    // Delay, busy flag can't be checked here.

  // Repeat last command.
  e_pulse();
  delay_us(100);  // Delay, busy flag can't be checked here.

  // Repeat last command a third time.
  e_pulse();
  delay_us(100);  // Delay, busy flag can't be checked here.

  // Now configure for 4 bit mode.
  WRITE(DISPLAY_D4_PIN, 0);
  e_pulse();
  delay_us(100);  // Some displays need this additional delay.
}

/**
  Read a byte from the bus. Doing so is e.g. required to detect wether the
  display is busy.

  \param rs   1: Read data.
              0: Read busy flag / address counter.

  \return Byte read from LCD controller.
*/
static uint8_t parallel_4bit_read(uint8_t rs) {
  uint8_t data;

  if (rs)
    WRITE(DISPLAY_RS_PIN, 1);           // Read data.
  else
    WRITE(DISPLAY_RS_PIN, 0);           // Read busy flag.
  WRITE(DISPLAY_RW_PIN, 1);             // Read mode.

  SET_INPUT(DISPLAY_D4_PIN);
  SET_INPUT(DISPLAY_D5_PIN);
  SET_INPUT(DISPLAY_D6_PIN);
  SET_INPUT(DISPLAY_D7_PIN);

  data = 0;

  // Read high nibble.
  WRITE(DISPLAY_E_PIN, 1);
  delay_us(1);
  if (READ(DISPLAY_D4_PIN)) data |= 0x10;
  if (READ(DISPLAY_D5_PIN)) data |= 0x20;
  if (READ(DISPLAY_D6_PIN)) data |= 0x40;
  if (READ(DISPLAY_D7_PIN)) data |= 0x80;
  WRITE(DISPLAY_E_PIN, 0);

  delay_us(1);

  // Read low nibble.
  WRITE(DISPLAY_E_PIN, 1);
  delay_us(1);
  if (READ(DISPLAY_D4_PIN)) data |= 0x01;
  if (READ(DISPLAY_D5_PIN)) data |= 0x02;
  if (READ(DISPLAY_D6_PIN)) data |= 0x04;
  if (READ(DISPLAY_D7_PIN)) data |= 0x08;
  WRITE(DISPLAY_E_PIN, 0);

  return data;
}

/**
  Report wether the bus or the connected display is busy.

  \return Wether the bus is busy, which means that eventual new transactions
          would have to wait.
*/
uint8_t parallel_4bit_busy(void) {
  uint8_t status;

  status = parallel_4bit_read(0);

  return status & 0x80;
}

/**
  Send a byte to the bus partner.

  \param data       The byte to be sent.

  \param rs   1 = parallel_4bit_data:        Write data.
              0 = parallel_4bit_instruction: Write instruction.

  Other than other bus implementations we do not buffer here. Writing a byte
  takes just some 3 microseconds and there is nothing supporting such writes in
  hardware, so the overhead of buffering is most likely not worth the effort.
*/
void parallel_4bit_write(uint8_t data, enum rs_e rs) {

  // Wait for the display to become ready.
  while (parallel_4bit_busy());

  // Setup for writing.
  WRITE(DISPLAY_RS_PIN, rs);            // Write data / instruction.
  WRITE(DISPLAY_RW_PIN, 0);             // Write mode.

  SET_OUTPUT(DISPLAY_D4_PIN);
  SET_OUTPUT(DISPLAY_D5_PIN);
  SET_OUTPUT(DISPLAY_D6_PIN);
  SET_OUTPUT(DISPLAY_D7_PIN);

  // Output high nibble.
  WRITE(DISPLAY_D4_PIN, (data & 0x10) ? 1 : 0);
  WRITE(DISPLAY_D5_PIN, (data & 0x20) ? 1 : 0);
  WRITE(DISPLAY_D6_PIN, (data & 0x40) ? 1 : 0);
  WRITE(DISPLAY_D7_PIN, (data & 0x80) ? 1 : 0);
  e_pulse();

  // Output low nibble.
  WRITE(DISPLAY_D4_PIN, (data & 0x01) ? 1 : 0);
  WRITE(DISPLAY_D5_PIN, (data & 0x02) ? 1 : 0);
  WRITE(DISPLAY_D6_PIN, (data & 0x04) ? 1 : 0);
  WRITE(DISPLAY_D7_PIN, (data & 0x08) ? 1 : 0);
  e_pulse();
}

#endif /* DISPLAY_BUS_4BIT */
