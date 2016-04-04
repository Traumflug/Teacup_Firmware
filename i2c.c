
/** \file
  \brief I2C / TWI subsystem

  "TWI", "Two Wire Interface", is Atmel's name for the (patented) I2C bus
  system. I2C is technically pretty sophisticated, it also takes the current
  state of these two wires as part of the protocol into account. Like SPI it's
  a master/slave system with a clock signal on the wire. Unlike SPI,
  communication partners aren't choosen by setting a pin, but by by
  transferring an address byte before the actual data.

  Accordingly, code has to deal with states, transmissions have a start and
  an end, and actions on the bus can result in different states, like success
  or failure.

  avr-libc comes with good sample code:

    http://www.nongnu.org/avr-libc/examples/twitest/twitest.c

  For technical details see section 22 of atmega328 datasheet.
*/

#include "i2c.h"

#ifdef I2C

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "delay.h"
#include "pinio.h"
#include "memory_barrier.h"


#if defined I2C_MASTER_MODE && defined I2C_SLAVE_MODE
  #error Cant be I2C master and slave at the same time.
#endif

#ifdef I2C_SLAVE_MODE
  #define I2C_MODE 1
#else
  #define I2C_MODE 0
#endif

#define I2C_MODE_MASK         0b00001100
// Start-Addr_R-Read-Stop: just read mode.
#define I2C_MODE_SARP         0b00000000
// Start-Addr_W-Write-Stop: just write mode.
#define I2C_MODE_SAWP         0b00000100
// Start-Addr_W-WrPageAdr-rStart-Addr_R-Read-Stop.
#define I2C_MODE_ENHA         0b00001000
// Transponder is busy.
#define I2C_MODE_BUSY         0b01000000
// Transponder is free.
#define I2C_MODE_FREE         0b10111111

// Transmission interrupted.
#define I2C_INTERRUPTED       0b10000000
// Transmission not interrupted.
#define I2C_NOINTERRUPTED     0b01111111

#define I2C_ERROR_BUS_FAIL    0b00000001
#define I2C_ERROR_NACK        0b00000010
#define I2C_ERROR_NO_ANSWER   0b00010000
#define I2C_ERROR_LOW_PRIO    0b00100000


// Address of the device that is communicated with.
uint8_t i2c_address;
// State of TWI component of MCU.
volatile uint8_t i2c_state = I2C_MODE_FREE;

#ifdef I2C_EEPROM_SUPPORT
  // For SAWSARP mode (see ENHA in i2c.h).
  uint8_t i2c_page_address[I2C_PAGE_ADDRESS_SIZE];
  // Index into the page address buffer.
  uint8_t i2c_page_index;
  // Count of bytes in page address.
  uint8_t i2c_page_count;
#endif /* I2C_EEPROM_SUPPORT */

#ifdef I2C_SLAVE_MODE
  uint8_t i2c_in_buffer[I2C_SLAVE_RX_BUFFER_SIZE];
  uint8_t i2c_out_buffer[I2C_SLAVE_TX_BUFFER_SIZE];
#endif /* I2C_SLAVE_MODE */

// Ringbuffer logic for buffer 'send'.
#define BUFSIZE I2C_BUFFER_SIZE

volatile uint8_t sendhead = 0;
volatile uint8_t sendtail = 0;
volatile uint8_t sendbuf[BUFSIZE];

#include "ringbuffer.h"


/**
  Inititalise the I2C/TWI subsystem.

  \param address Address the system should listen to in slave mode, unused
                 when configured for master mode. In master mode, receiver
                 address is given to the send command.
*/
void i2c_init(uint8_t address) {

  #ifdef I2C_MASTER_MODE
    #ifdef I2C_ENABLE_PULLUPS
      SET_INPUT(SCL);
      PULLUP_ON(SCL);
      SET_INPUT(SDA);
      PULLUP_ON(SDA);
    #endif /* I2C_ENABLE_PULLUPS */

    /**
      TWI Bit Rate register
      SCL_freq = CPU_freq / (16 + 2 * TWBR)
      See: page 235 of atmega328 datasheet.
    */
    TWBR = ((F_CPU / I2C_BITRATE) - 16) / 2;

    /**
      TWI Status Register
      Lower two bits set the prescaler value.
      See: page 236 of atmega328 datasheet.
    */
    TWSR = 0x00;
  #endif /* I2C_MASTER_MODE */

  #ifdef I2C_SLAVE_MODE
    i2c_address = address;
    TWAR = i2c_address; // We listen to broadcasts if lowest bit is set.
    TWCR = (0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
  #endif
}

/**
  Send a byte to the I2C partner.

  \param address    I2C address of the communications partner.

  \param data       The byte to be buffered/sent.

  \param last_byte  Wether this is the last byte of a transaction.

  Unlike many other protocols (serial, SPI), I2C has an explicite transmission
  start and transmission end. Transmission start is detected automatically,
  but end of the transmission has to be told by the invoking code.

  Data is buffered, so this returns quickly for small amounts of data. Large
  amounts don't get lost, but this function has to wait until sufficient
  previous data was sent.

  TODO: for now this function assumes that the buffer drains between distinct
        transmissions. This means, last_byte is ignored and transmission is
        ended as soon as the buffer drains.
*/
void i2c_write(uint8_t address, uint8_t data, uint8_t last_byte) {

  if ( ! (i2c_state & I2C_MODE_BUSY)) {
    // No transmission ongoing, start one.
    i2c_address = address;

    // Just send.
    i2c_state = I2C_MODE_SAWP;

    // Start transmission.
    TWCR = (1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
    i2c_state |= I2C_MODE_BUSY;
  }

  while ( ! buf_canwrite(send))
    delay_us(10);

  ATOMIC_START
    buf_push(send, data);
  ATOMIC_END
}

/**
  This is the interrupt for all the I2C things.

  A few words about TWCR bits:

    Bit 7 (TWINT) is used to run TWI.

    Bit 6 (TWEA) is used to send ACK (if set) in cases of

      a) device's own slave address has been received;
      b) general call has been received;
      c) a data byte has been received.

    Bit 5 (TWSTA) is 1 if app wants to be a master, don't forget to
                  clear this bit.

    Bit 4 (TWSTO) is generated STOP in master mode if set (cleared
                  automaticly), recovered from error condition in slave
                  mode if set.

    Bit 3 (TWWC) is write collision flag. Sets on try to writeto TWDR when
                 TWINT is low.

    Bit 2 (TWEN) activates SDA/SCL pins if set. Set to 0 to disable TWI.

    Bit 1 (Reserved).

    Bit 0 (TWIE) enables TWI interrupt.
*/
ISR(TWI_vect) {

  switch (TWSR & TW_STATUS_MASK) {
    case TW_BUS_ERROR:
      // A hardware error was detected.
      i2c_state |= I2C_ERROR_BUS_FAIL;
      // Send stop condition.
      TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(0<<TWIE);
      break;
    case TW_START:
      // Start happens, send a target address.
      if ((i2c_state & I2C_MODE_MASK) == I2C_MODE_SARP) {
        i2c_address |= 0x01;
      } else {
        i2c_address &= 0xFE;
      }
      TWDR = i2c_address;
      TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      break;
    case TW_REP_START:
      // Start happens, send a target address.
      if ((i2c_state & I2C_MODE_MASK) == I2C_MODE_ENHA) {
        i2c_address |= 0x01;
      } else {
        i2c_address &= 0xFE;
      }
      TWDR = i2c_address;
      TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      break;
    case TW_MT_SLA_ACK:
      // SLA+W was sent, then ACK received.
      if ((i2c_state & I2C_MODE_MASK) == I2C_MODE_SAWP && buf_canread(send)) {
        buf_pop(send, TWDR);
        TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      }
      #ifdef I2C_EEPROM_SUPPORT
        if ((i2c_state & I2C_MODE_MASK) == I2C_MODE_ENHA) {
          TWDR = i2c_page_address[i2c_page_index++];
          TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
        }
      #endif
      break;
    case TW_MT_SLA_NACK:
      // SLA+W was sent, got NACK, so slave is busy or out of bus.
      i2c_state |= I2C_ERROR_NO_ANSWER;
      // Send stop condition.
      TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(0<<TWIE);
      break;
    case TW_MT_DATA_ACK:
      // A byte was sent, got ACK.
      if ((i2c_state & I2C_MODE_MASK) == I2C_MODE_SAWP) {
        if (buf_canread(send)) {
          // Send the next byte.
          buf_pop(send, TWDR);
          TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
        } else {
          // Last byte, send stop condition.
          i2c_state = I2C_MODE_FREE;
          TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(0<<TWIE);
        }
      }
      #ifdef I2C_EEPROM_SUPPORT
        if ((i2c_state & I2C_MODE_MASK) == I2C_MODE_ENHA) {
          // It was a page address byte.
          if (i2c_page_index == i2c_page_count) {
            // It was the last byte of the page address.
            TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
          } else {
            // Send the next page address byte.
            TWDR = i2c_page_address[i2c_page_index++];
            TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
          }
        }
      #endif
      break;
    case TW_MT_DATA_NACK:
      // Byte was sent but got NACK, there are two possible reasons:
      //  - a slave stops transmission and it is ok, or
      //  - a slave became crazy.
      i2c_state |= I2C_ERROR_NACK;
      // Send stop condition.
      TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(0<<TWIE);
      break;
    case TW_MT_ARB_LOST:  // Collision, identical to TW_MR_ARB_LOST.
      // It looks like there is another master on the bus.
      i2c_state |= I2C_ERROR_LOW_PRIO;
      // Setup all again.
      sendtail = sendhead;
      #ifdef I2C_EEPROM_SUPPORT
        i2c_page_index = 0;
      #endif
      // Try to resend when the bus became free.
      TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      break;
#if 0
// Note: we currently have no receive buffer, so we can't receive anything.
    case TW_MR_SLA_ACK:
      // SLA+R was sent, got АСК, then received a byte.
      if (i2c_index + 1 == i2c_byte_count) {
        // Last byte fitting into the buffer. Request a byte, then send NACK
        // to slave and it will release the bus.
        TWCR = (1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      } else {
        // Or just receive a byte and sent ACK.
        TWCR = (1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      }
      break;
    case TW_MR_SLA_NACK:
      // SLA+R was sent, got NАСК, it seems the slave is busy.
      i2c_state |= I2C_ERROR_NO_ANSWER;
      // Send stop condition.
      TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(0<<TWIE);
      break;
    case TW_MR_DATA_ACK:
      i2c_buffer[i2c_index++] = TWDR;
      // TODO: Add BUFFER OVERFLOW check.
      if (i2c_index + 1 == i2c_byte_count) {
        // Last byte wait the processing.
        TWCR = (1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      } else {
        // Request the next byte.
        TWCR = (1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      }
      break;
    case TW_MR_DATA_NACK:
      // Last byte received, send NACK to make the slave to release the bus.
      i2c_buffer[i2c_index] = TWDR;
      // Send stop condition.
      TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(0<<TWIE);
      break;
#endif

    #ifdef I2C_SLAVE_MODE

    case TW_SR_ARB_LOST_SLA_ACK:
    case TW_SR_ARB_LOST_GCALL_ACK:
      // Another master on the bus sent some bytes, receive them.
      i2c_state |= I2C_ERROR_LOW_PRIO;
      // Restore the transfer.
      i2c_index = 0;
      #ifdef I2C_EEPROM_SUPPORT
        i2c_page_index = 0;
      #endif
    case TW_SR_SLA_ACK:
    case TW_SR_GCALL_ACK:
      i2c_state |= I2C_MODE_BUSY; // Lock bus.
      i2c_index = 0;
      if (I2C_SLAVE_RX_BUFFER_SIZE == 1) {
        // We should take alone byte and send NACK.
        TWCR = (1<<TWINT)|(0<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      } else {
        // Get a byte and send ACK.
        TWCR = (1<<TWINT)|(1<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      }
      break;
    case TW_SR_DATA_ACK:
    case TW_SR_GCALL_DATA_ACK:
      i2c_in_buffer[i2c_index++] = TWDR;
      if (i2c_index == I2C_SLAVE_RX_BUFFER_SIZE - 1) {
        // Room for only one byte left, send NACK.
        TWCR = (1<<TWINT)|(0<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      } else {
        // We have room for more bytes, send ACK.
        TWCR = (1<<TWINT)|(1<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      }
      break;
    case TW_SR_DATA_NACK:
    case TW_SR_GCALL_DATA_NACK:
      i2c_in_buffer[i2c_index] = TWDR;
      // Send stop condition.
      TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(0<<TWIE);
      break;
    case TW_SR_STOP:
      // We got a Restart. What we will do?
      // Here we can do additional logic but we don't need it at this time.
      // Just ignore it now.
      TWCR = (1<<TWINT)|(1<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      break;
    case TW_ST_ARB_LOST_SLA_ACK:
      // Got own address on read from another master.
      i2c_state |= I2C_ERROR_LOW_PRIO | I2C_INTERRUPTED;

      // Reinit.
      i2c_index = 0;
      #ifdef I2C_EEPROM_SUPPORT
        i2c_page_index = 0;
      #endif
    case TW_ST_SLA_ACK:
      // We have got own address on read.
      i2c_index = 0;
      TWDR = i2c_out_buffer[i2c_index];
      if (I2C_SLAVE_TX_BUFFER_SIZE == 1) {
        // If it is last byte, we hope to receive NACK.
        TWCR = (1<<TWINT)|(0<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      } else {
        // Wait for ACK.
        TWCR = (1<<TWINT)|(1<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      }
      break;
    case TW_ST_DATA_ACK:
      // Send byte and got ACK, then send next byte to master.
      TWDR = i2c_out_buffer[++i2c_index];
      if (I2C_SLAVE_TX_BUFFER_SIZE - 1 == i2c_index) {
        // It was last byte, send it and wait for NACK.
        TWCR = (1<<TWINT)|(0<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      } else {
        // Send byte and wait for ACK.
        TWCR = (1<<TWINT)|(1<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWEN)|(1<<TWIE);
      }
      break;
    case TW_ST_DATA_NACK:
      // We sent the last byte and received NACK or ACK (doesn't matter here).
      if (i2c_state & I2C_INTERRUPTED) {
        // There was interrupted master transfer.
        i2c_state &= I2C_NOINTERRUPTED;
        // Generate start as the bus became free.
        TWCR = (1<<TWINT)|(1<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(1<<TWEN)|(1<<TWIE);
      } else {
        // Send stop condition.
        TWCR = (1<<TWINT)|(I2C_MODE<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(1<<TWEN)|(0<<TWIE);
      }
      break;

    #endif /* I2C_SLAVE_MODE */

    default:
      break;
    }
}

#endif /* I2C */
