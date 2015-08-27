
/** \file
  \brief Serial subsystem, ARM specific part.

  To be included from serial.c, for more details see there.

  Other than AVRs, ARMs feature a serial buffer in hardware, so we can get
  away without a software buffer and also without(!) interrupts.

  Code here is heavily inspired by serial_api.c of MBED, found in
  mbed/libraries/mbed/targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/.
*/

#if defined TEACUP_C_INCLUDE && defined __ARMEL__

#include "arduino.h"
#ifdef __LPC1114__
  #include "mbed-pinmap.h"
#else
  #include "cmsis-pio_sam3x8e.h"
  #include "cmsis-sam3x8e.h"
#endif

#ifdef XONXOFF
  #error XON/XOFF protocol not yet implemented for ARM. \
         See serial-avr.c for inspiration.
#endif

#ifdef __LPC1114__
  LPC_UART_TypeDef *port = LPC_UART;
#else
  Uart *port = UART;
#endif

/** Initialise serial subsystem.

  Set up baud generator and interrupts, clear buffers.
*/
void serial_init() {

#ifdef __LPC1114__
  // Turn on UART power.
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 12);

  // Enable fifos and default RX trigger level.
  port->FCR = 1 << 0  // FIFO Enable - 0 = Disabled, 1 = Enabled.
            | 0 << 1  // Rx Fifo Reset.
            | 0 << 2  // Tx Fifo Reset.
            | 0 << 6; // Rx irq trigger level.
                      // 0 = 1 char, 1 = 4 chars, 2 = 8 chars, 3 = 14 chars.

  // Disable IRQs.
  port->IER = 0 << 0  // Rx Data available irq enable.
            | 0 << 1  // Tx Fifo empty irq enable.
            | 0 << 2; // Rx Line Status irq enable.

  // Baud rate calculation - - - TO BE REFINED, we can calculate all this
  // in the preprocessor or even hardcode it, because baud rate never changes.
  {
    uint32_t baudrate = BAUD;

    LPC_SYSCON->UARTCLKDIV = 0x1;
    uint32_t PCLK = SystemCoreClock;
    // First we check to see if the basic divide with no DivAddVal/MulVal
    // ratio gives us an integer result. If it does, we set DivAddVal = 0,
    // MulVal = 1. Otherwise, we search the valid ratio value range to find
    // the closest match. This could be more elegant, using search methods
    // and/or lookup tables, but the brute force method is not that much
    // slower, and is more maintainable.
    uint16_t DL = PCLK / (16 * baudrate);

    uint8_t DivAddVal = 0;
    uint8_t MulVal = 1;
    int hit = 0;
    uint16_t dlv;
    uint8_t mv, dav;
    if ((PCLK % (16 * baudrate)) != 0) {     // Checking for zero remainder
        int err_best = baudrate, b;
        for (mv = 1; mv < 16 && !hit; mv++)
        {
            for (dav = 0; dav < mv; dav++)
            {
                // baudrate = PCLK / (16 * dlv * (1 + (DivAdd / Mul))
                // solving for dlv, we get dlv = mul * PCLK / (16 * baudrate * (divadd + mul))
                // mul has 4 bits, PCLK has 27 so we have 1 bit headroom which can be used for rounding
                // for many values of mul and PCLK we have 2 or more bits of headroom which can be used to improve precision
                // note: X / 32 doesn't round correctly. Instead, we use ((X / 16) + 1) / 2 for correct rounding

                if ((mv * PCLK * 2) & 0x80000000) // 1 bit headroom
                    dlv = ((((2 * mv * PCLK) / (baudrate * (dav + mv))) / 16) + 1) / 2;
                else // 2 bits headroom, use more precision
                    dlv = ((((4 * mv * PCLK) / (baudrate * (dav + mv))) / 32) + 1) / 2;

                // datasheet says if DLL==DLM==0, then 1 is used instead since divide by zero is ungood
                if (dlv == 0)
                    dlv = 1;

                // datasheet says if dav > 0 then DL must be >= 2
                if ((dav > 0) && (dlv < 2))
                    dlv = 2;

                // integer rearrangement of the baudrate equation (with rounding)
                b = ((PCLK * mv / (dlv * (dav + mv) * 8)) + 1) / 2;

                // check to see how we went
                b = b - baudrate;
                if (b < 0) b = -b;
                if (b < err_best)
                {
                    err_best  = b;

                    DL        = dlv;
                    MulVal    = mv;
                    DivAddVal = dav;

                    if (b == baudrate)
                    {
                        hit = 1;
                        break;
                    }
                }
            }
        }
    }

    // set LCR[DLAB] to enable writing to divider registers
    port->LCR |= (1 << 7);

    // set divider values
    port->DLM = (DL >> 8) & 0xFF;
    port->DLL = (DL >> 0) & 0xFF;
    port->FDR = (uint32_t) DivAddVal << 0
                   | (uint32_t) MulVal    << 4;

    // clear LCR[DLAB]
    port->LCR &= ~(1 << 7);

  } /* End of baud rate calculation. */

  // Serial format.
  port->LCR = (8 - 5)  << 0  // 8 data bits.
            | (1 - 1)  << 2  // 1 stop bit.
            | 0        << 3  // Parity disabled.
            | 0        << 4; // 0 = odd parity, 1 = even parity.

  // Pinout the UART.
  pin_function(USBTX, 0x01);
  pin_mode(USBTX, PullUp);
  pin_function(USBRX, 0x01);
  pin_mode(USBRX, PullUp);
#else
  uint32_t ul_sr;
 
  // ==> Pin configuration
  // Disable interrupts on Rx and Tx
  PIOA->PIO_IDR = PIO_PA8A_URXD | PIO_PA9A_UTXD;
 
  // Disable the PIO of the Rx and Tx pins so that the peripheral controller can use them
  PIOA->PIO_PDR = PIO_PA8A_URXD | PIO_PA9A_UTXD;
 
  // Read current peripheral AB select register and set the Rx and Tx pins to 0 (Peripheral A function)
  ul_sr = PIOA->PIO_ABSR;
  PIOA->PIO_ABSR &= ~(PIO_PA8A_URXD | PIO_PA9A_UTXD) & ul_sr;
 
  // Enable the pull up on the Rx and Tx pin
  PIOA->PIO_PUER = PIO_PA8A_URXD | PIO_PA9A_UTXD;
 
  // ==> Actual uart configuration
  // Enable the peripheral uart controller
  PMC->PMC_PCER0 = 1 << ID_UART;
 
  // Reset and disable receiver & transmitter
  port->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;
 
  // Set the baudrate to 115200
  port->UART_BRGR = 45; // 84000000 / 16 * x = BaudRate (write x into UART_BRGR)
 
  // No Parity
  port->UART_MR = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO;
 
  // Disable PDC channel requests
  port->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
 
  // Disable / Enable interrupts on end of receive
  port->UART_IDR = 0xFFFFFFFF;
  NVIC_EnableIRQ((IRQn_Type) ID_UART);
  port->UART_IER = UART_IER_RXRDY;
 
  // Enable receiver and trasmitter
  port->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
#endif
}

/** Check wether characters can be read.

  Other than the AVR implementation this returns not the number of characters
  in the line, but only wether there is at least one or not.
*/
uint8_t serial_rxchars(void) {
  #ifdef __LPC1114__
    return port->LSR & 0x01;
  #else
    return (port->UART_SR & UART_SR_RXRDY) > 0;
  #endif
}

/** Read one character.
*/
uint8_t serial_popchar(void) {
  uint8_t c = 0;

  if (serial_rxchars())
    #ifdef __LPC1114__
      c = port->RBR;
    #else
      c = port->UART_RHR;
    #endif

  return c;
}

/** Send one character.

  If the queue is full, too bad. Do NOT block.
*/
void serial_writechar(uint8_t data) {
  #ifdef __LPC1114__
    port->THR = data;
  #else
    port->UART_THR = data;
  #endif
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */