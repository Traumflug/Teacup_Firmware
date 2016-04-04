
/** \file
  \brief Serial subsystem, AVR specific part.

  To be included from serial.c, for details see there.
*/

#if defined TEACUP_C_INCLUDE && defined __AVR__

#include <avr/interrupt.h>
#include "memory_barrier.h"
#include "arduino.h"
#include "pinio.h"

/** \def BUFSIZE

  Size of TX and RX buffers. MUST be a \f$2^n\f$ value.

  Unlike ARM MCUs, which come with a hardware buffer, AVRs require a read and
  transmit buffer implemented in software. This buffer not only raises
  reliability, it also allows transmitting characters from interrupt context.
*/
#define BUFSIZE     64

/** \def ASCII_XOFF

  ASCII XOFF character.
*/
#define ASCII_XOFF  19

/** \def ASCII_XON

  ASCII XON character.
*/
#define ASCII_XON   17


#ifndef USB_SERIAL

/** RX buffer.

  rxhead is the head pointer and points to the next available space.

  rxtail is the tail pointer and points to last character in the buffer.
*/
volatile uint8_t rxhead = 0;
volatile uint8_t rxtail = 0;
volatile uint8_t rxbuf[BUFSIZE];

/** TX buffer.

  Same mechanism as RX buffer.
*/
volatile uint8_t txhead = 0;
volatile uint8_t txtail = 0;
volatile uint8_t txbuf[BUFSIZE];

#include "ringbuffer.h"

#ifdef XONXOFF
#define FLOWFLAG_STATE_XOFF 0
#define FLOWFLAG_SEND_XON   1
#define FLOWFLAG_SEND_XOFF  2
#define FLOWFLAG_STATE_XON  4
// initially, send an XON
volatile uint8_t flowflags = FLOWFLAG_SEND_XON;
#endif


/** Initialise serial subsystem.

  Set up baud generator and interrupts, clear buffers.
*/
void serial_init() {

  #if BAUD > 38401
    UCSR0A = MASK(U2X0);
    UBRR0 = (((F_CPU / 8) / BAUD) - 0.5);
  #else
    UCSR0A = 0;
    UBRR0 = (((F_CPU / 16) / BAUD) - 0.5);
  #endif

  UCSR0B = MASK(RXEN0) | MASK(TXEN0);
  UCSR0C = MASK(UCSZ01) | MASK(UCSZ00);

  UCSR0B |= MASK(RXCIE0) | MASK(UDRIE0);
}

/** Receive interrupt.

  We have received a character, stuff it in the RX buffer if we can, or drop
  it if we can't. Using the pragma inside the function is incompatible with
  Arduinos' gcc.
*/
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#ifdef USART_RX_vect
ISR(USART_RX_vect)
#else
ISR(USART0_RX_vect)
#endif
{
  if (buf_canwrite(rx))
    buf_push(rx, UDR0);
  else {
    // Not reading the character makes the interrupt logic to swamp us with
    // retries, so better read it and throw it away.
    //#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    uint8_t trash;
    //#pragma GCC diagnostic pop

    trash = UDR0;
  }

  #ifdef XONXOFF
    if (flowflags & FLOWFLAG_STATE_XON && buf_canwrite(rx) <= 16) {
      // The buffer has only 16 free characters left, so send an XOFF.
      // More characters might come in until the XOFF takes effect.
      flowflags = FLOWFLAG_SEND_XOFF | FLOWFLAG_STATE_XON;
      // Enable TX interrupt so we can send this character.
      UCSR0B |= MASK(UDRIE0);
    }
  #endif
}
#pragma GCC diagnostic pop

/** Transmit buffer ready interrupt.

  Provide the next character to transmit if we can, otherwise disable this
  interrupt.
*/
#ifdef USART_UDRE_vect
ISR(USART_UDRE_vect)
#else
ISR(USART0_UDRE_vect)
#endif
{
  #ifdef XONXOFF
    if (flowflags & FLOWFLAG_SEND_XON) {
      UDR0 = ASCII_XON;
      flowflags = FLOWFLAG_STATE_XON;
    }
    else if (flowflags & FLOWFLAG_SEND_XOFF) {
      UDR0 = ASCII_XOFF;
      flowflags = FLOWFLAG_STATE_XOFF;
    }
    else
  #endif

  if (buf_canread(tx))
    buf_pop(tx, UDR0);
  else
    UCSR0B &= ~MASK(UDRIE0);
}

/** Check how many characters can be read.
*/
uint8_t serial_rxchars() {
  return buf_canread(rx);
}

/** Read one character.
*/
uint8_t serial_popchar() {
  uint8_t c = 0;

  // It's imperative that we check, because if the buffer is empty and we
  // pop, we'll go through the whole buffer again.
  if (buf_canread(rx))
    buf_pop(rx, c);

  #ifdef XONXOFF
    if ((flowflags & FLOWFLAG_STATE_XON) == 0 && buf_canread(rx) <= 16) {
      // The buffer has (BUFSIZE - 16) free characters again, so send an XON.
      flowflags = FLOWFLAG_SEND_XON;
      UCSR0B |= MASK(UDRIE0);
    }
  #endif

  return c;
}

/** Send one character.
*/
void serial_writechar(uint8_t data) {

  // Check if interrupts are enabled.
  if (SREG & MASK(SREG_I)) {
    // If they are, we should be ok to block since the tx buffer is emptied
    // from an interrupt.
    for ( ; buf_canwrite(tx) == 0; ) ;
    buf_push(tx, data);
  }
  else {
    // Interrupts are disabled -- maybe we're in one?
    // Anyway, instead of blocking, only write if we have room.
    if (buf_canwrite(tx))
      buf_push(tx, data);
  }

  // Enable TX interrupt so we can send this character.
  UCSR0B |= MASK(UDRIE0);
}
#endif /* USB_SERIAL */

#endif /* defined TEACUP_C_INCLUDE && defined __AVR__ */
