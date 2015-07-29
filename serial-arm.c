
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
#include "cmsis-lpc11xx.h"
#include "delay.h"
#include "sersendf.h"

#ifdef XONXOFF
  #error XON/XOFF protocol not yet implemented for ARM. \
         See serial-avr.c for inspiration.
#endif


/** Initialise serial subsystem.

  Set up baud generator and interrupts, clear buffers.
*/
void serial_init() {

  // Turn on UART power.
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 12);

  // Enable fifos and default RX trigger level.
  LPC_UART->FCR = 1 << 0  // FIFO Enable - 0 = Disabled, 1 = Enabled.
                | 0 << 1  // Rx Fifo Reset.
                | 0 << 2  // Tx Fifo Reset.
                | 0 << 6; // Rx irq trigger level.
                          // 0 = 1 char, 1 = 4 chars, 2 = 8 chars, 3 = 14 chars.

  // Disable IRQs.
  LPC_UART->IER = 0 << 0  // Rx Data available irq enable.
                | 0 << 1  // Tx Fifo empty irq enable.
                | 0 << 2; // Rx Line Status irq enable.

  LPC_SYSCON->UARTCLKDIV = 0x1;

  /**
    Calculating the neccessary values for a proper baud rate is pretty complex
    and, as system clock and baudrate are known at compile time and never
    changed at runtime, unneccessary.

    However, how to get these values? Well, we do kind of an easter-egg. If
    parameters are not known, we calculate them at runtime anyways, and also
    report them to the user. So she can insert them here and after doing so,
    whoops, serial fast and binary small :-)

    Replacing this calculation with fixed values makes the binary a whopping
    564 bytes smaller.
  */
  #if (__SYSTEM_CLOCK == 48000000UL) && (BAUD == 115200)
    #define UART_DLM 0x00
    #define UART_DLL 0x17
    #define UART_FDR 0xF2
  //#elif (__SYSTEM_CLOCK == xxx) && (BAUD == xxx)
    // Define more combinations here, Teacup reports the neccessary values
    // at startup time.
  #endif

  #ifdef UART_DLM
    // Set LCR[DLAB] to enable writing to divider registers.
    LPC_UART->LCR |= (1 << 7);

    // Set divider values.
    LPC_UART->DLM = UART_DLM;
    LPC_UART->DLL = UART_DLL;
    LPC_UART->FDR = UART_FDR;

    // Clear LCR[DLAB].
    LPC_UART->LCR &= ~(1 << 7);
  #else
    /**
      Calculate baud rate at runtime and later report it to the user. This
      code is taken as-is from MBEDs serial_api.c, just reformatted whitespace
      and comments.
    */
    uint32_t baudrate = BAUD;
    uint32_t PCLK = __SYSTEM_CLOCK;

    /**
      First we check to see if the basic divide with no DivAddVal/MulVal
      ratio gives us an integer result. If it does, we set DivAddVal = 0,
      MulVal = 1. Otherwise, we search the valid ratio value range to find
      the closest match. This could be more elegant, using search methods
      and/or lookup tables, but the brute force method is not that much
      slower, and is more maintainable.
    */
    uint16_t DL = PCLK / (16 * baudrate);

    uint8_t DivAddVal = 0;
    uint8_t MulVal = 1;
    int hit = 0;
    uint16_t dlv;
    uint8_t mv, dav;

    if ((PCLK % (16 * baudrate)) != 0) {     // Checking for zero remainder.
      int err_best = baudrate, b;

      for (mv = 1; mv < 16 && ! hit; mv++) {
        for (dav = 0; dav < mv; dav++) {
          /**
              baudrate = PCLK / (16 * dlv * (1 + (DivAdd / Mul))

            solving for dlv, we get

              dlv = mul * PCLK / (16 * baudrate * (divadd + mul))

            mul has 4 bits, PCLK has 27 so we have 1 bit headroom which can be
            used for rounding. For many values of mul and PCLK we have 2 or
            more bits of headroom which can be used to improve precision.

            Note: X / 32 doesn't round correctly. Instead, we use
                  ((X / 16) + 1) / 2 for correct rounding.
          */
          if ((mv * PCLK * 2) & 0x80000000) // 1 bit headroom.
            dlv = ((((2 * mv * PCLK) /
                     (baudrate * (dav + mv))) / 16) + 1) / 2;
          else  // 2 bits headroom, use more precision.
            dlv = ((((4 * mv * PCLK) /
                     (baudrate * (dav + mv))) / 32) + 1) / 2;

          // Datasheet says, if DLL == DLM == 0, then 1 is used instead,
          // since divide by zero is ungood.
          if (dlv == 0)
            dlv = 1;

          // Datasheet says if dav > 0 then DL must be >= 2.
          if ((dav > 0) && (dlv < 2))
            dlv = 2;

          // Integer rearrangement of the baudrate equation (with rounding).
          b = ((PCLK * mv / (dlv * (dav + mv) * 8)) + 1) / 2;

          // Check to see how we went.
          b = b - baudrate;
          if (b < 0) b = -b;
          if (b < err_best) {
            err_best  = b;

            DL        = dlv;
            MulVal    = mv;
            DivAddVal = dav;

            if (b == baudrate) {
              hit = 1;
              break;
            }
          }
        }
      }
    }

    // Set results like above.
    LPC_UART->LCR |= (1 << 7);
    LPC_UART->DLM = (DL >> 8) & 0xFF;
    LPC_UART->DLL = (DL >> 0) & 0xFF;
    LPC_UART->FDR = (uint32_t) DivAddVal << 0
                  | (uint32_t) MulVal    << 4;
    LPC_UART->LCR &= ~(1 << 7);
  #endif /* UART_DLM, ! UART_DLM */

  // Serial format.
  LPC_UART->LCR = (8 - 5)  << 0  // 8 data bits.
                | (1 - 1)  << 2  // 1 stop bit.
                | 0        << 3  // Parity disabled.
                | 0        << 4; // 0 = odd parity, 1 = even parity.

  // Pinout the UART. No need to set GPIO stuff, like data direction.
  LPC_IOCON->RXD_CMSIS = 0x01 << 0  // Function RXD.
                       | 0x02 << 3; // Pullup enabled.
  LPC_IOCON->TXD_CMSIS = 0x01 << 0  // Function TXD.
                       | 0x00 << 3; // Pullup inactive.

  #ifndef UART_DLM
    /**
      Baud rate settings were calculated at runtime, report them to the user
      to allow her to insert them above. This is possible, because we just
      completed setting up the serial port. Be generous with delays, on new
      hardware delays might be too quick as well.

      Uhm, yes, lots of code. 1420 bytes binary size together with the baudrate
      calculation above. But it's #ifdef'd out when parameters are set above
      and doing the calculation always at runtime would always add 400 bytes
      binary size.
    */
    delay_ms(500);
    serial_writestr_P(PSTR("\nSerial port parameters were calculated at "));
    serial_writestr_P(PSTR("runtime.\nInsert these values to the list of "));
    serial_writestr_P(PSTR("known settings in serial-arm.c:\n"));
    sersendf_P(PSTR("  UART_DLM %sx\n"), (DL >> 8) & 0xFF);
    sersendf_P(PSTR("  UART_DLL %sx\n"), (DL >> 0) & 0xFF);
    sersendf_P(PSTR("  UART_FDR %sx\n"), (DivAddVal << 0) | (MulVal << 4));
    serial_writestr_P(PSTR("Doing so will speed up serial considerably.\n\n"));
  #endif
}

/** Check wether characters can be read.

  Other than the AVR implementation this returns not the number of characters
  in the line, but only wether there is at least one or not.
*/
uint8_t serial_rxchars(void) {
  return LPC_UART->LSR & 0x01;
}

/** Read one character.
*/
uint8_t serial_popchar(void) {
  uint8_t c = 0;

  if (serial_rxchars())
    c = LPC_UART->RBR;

  return c;
}

/** Send one character.

  If the queue is full, we wait as long as sending a character takes
  (87 microseconds at 115200 baud) and send then blindly. This way we can
  send arbitrarily long messages without slowing down short messages or even
  blocking.
*/
void serial_writechar(uint8_t data) {
  if ( ! (LPC_UART->LSR & (0x01 << 5)))       // Queue full?
    delay_us((1000000 / BAUD * 10) + 1);

  #ifndef UART_DLM                        // Longer delays for fresh hardware.
    delay_ms(100);
  #endif

  LPC_UART->THR = data;
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
