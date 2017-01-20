
/** \file
  \brief Serial subsystem, ARM specific part.

  To be included from serial.c, for more details see there.

  Other than AVRs, ARMs feature a serial buffer in hardware, so we can get
  away without a software buffer and also without(!) interrupts.

  Code here is heavily inspired by serial_api.c of MBED
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__

#include "arduino.h"
#include "pinio.h"
#include "delay.h"

#ifdef XONXOFF
  #error XON/XOFF protocol not yet implemented for ARM. \
         See serial-avr.c for inspiration.
#endif

void serial_init()
{
    uint32_t tempreg;
    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure the UART pins
    // AF 4bits per channel
    // Alternate functions from DM00115249.pdf datasheet (page 47; table 9)
    SET_AFR(TXD, 0x7);
    SET_AFR(RXD, 0x7);

    // Set pins to alternate function mode
    SET_MODE(TXD, 0x2);
    SET_MODE(RXD, 0x2);

    SET_OSPEED(TXD, 0x3);
    SET_OSPEED(RXD, 0x3);

    PULL_OFF(TXD);
    PULL_OFF(RXD);
    
    /* Disable the peripheral */
    USART2->CR1 &=  ~USART_CR1_UE;
  
    /* Clear M, PCE, PS, TE and RE bits */
    tempreg = USART2->CR1;
    tempreg &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | \
                                   USART_CR1_RE | USART_CR1_OVER8);

    /* Configure the UART Word Length, Parity and mode:*/
    tempreg |= USART_CR1_RE | USART_CR1_TE;
    USART2->CR1 = tempreg;

    /* 19.3.4 Fractional baud rate generation => reference manual for STM32F411
    Set BRR for 115,200 Hz 
    div = 48MHz/(16*BAUD)
    Mantisse = int(div) << 8
    Divisor = int((div - int(div))*16)
    BRR = Mantisse + Divisor
    */
    #if !defined BAUD
    #define BAUD 115200
    #endif

    #define SERIAL_APBCLK (__SYSTEM_CLOCK/2)

    #define INT_DIVIDER ((25 * SERIAL_APBCLK) / (4 * BAUD))
    #define BAUD_H ((INT_DIVIDER / 100) << 4)
    #define FRACT_DIVIDER (INT_DIVIDER - (100 * (BAUD_H >> 4)))
    #define BAUD_L (((((FRACT_DIVIDER * 16) + 50) / 100)) & 0X0F)

    USART2->BRR = BAUD_H | BAUD_L;

    /* Clear STOP[13:12] bits */
    tempreg = USART2->CR2;
    tempreg &= ~(USART_CR2_STOP);
  
    /* In asynchronous mode, the following bits must be kept cleared: 
     - LINEN and CLKEN bits in the USART_CR2 register,
     - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
    tempreg &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
    USART2->CR2 = tempreg;

    tempreg = USART2->CR3;
    tempreg &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

    /* Clear CTSE and RTSE bits */
    tempreg &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
    USART2->CR3 = tempreg;
  
    /* Enable the peripheral */
    USART2->CR1 |=  USART_CR1_UE;
}

/** Check wether characters can be read.

  Other than the AVR implementation this returns not the number of characters
  in the line, but only wether there is at least one or not.
*/
uint8_t serial_rxchars(void) {
  return USART2->SR & USART_SR_RXNE;
}

/** Read one character.
*/
uint8_t serial_popchar(void) {
  uint8_t c = 0;

  if (serial_rxchars())
    c = (uint8_t)(USART2->DR & 0x1FF);

  return c;
}

/** Check wether characters can be written
*/
uint8_t serial_txchars(void) {
  return USART2->SR &USART_SR_TXE;
}
/** Send one character.
*/
void serial_writechar(uint8_t data) {
  if ( !serial_txchars())       // Queue full?
    delay_us((1000000 / BAUD * 10) + 7);
  USART2->DR = (uint32_t)(data & 0x1FF);
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__ */