
/** \file
  \brief Serial subsystem, ARM specific part.

  To be included from serial.c, for more details see there.

  Other than AVRs, ARMs feature a serial buffer in hardware, so we can get
  away without a software buffer and also without(!) interrupts.

  Code here is heavily inspired by serial_api.c of MBED
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32__

#include "arduino.h"
#include "pinio.h"
#include "delay.h"

#ifdef XONXOFF
  #error XON/XOFF protocol not yet implemented for ARM. \
         See serial-avr.c for inspiration.
#endif

#define UART_SERIAL USART2

void init_serial1(void) {
  // Enable USART1 clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  // Configure the UART pins
  // AF 4bits per channel
  // Alternate functions from DM00115249.pdf datasheet (page 47; table 9)
  SET_AFR(TX_UART1, 0x7);
  SET_AFR(RX_UART1, 0x7);

  // Set pins to alternate function mode
  SET_MODE(TX_UART1, 0x2);
  SET_MODE(RX_UART1, 0x2);

  SET_OSPEED(TX_UART1, 0x3);
  SET_OSPEED(RX_UART1, 0x3);

  PULL_OFF(TX_UART1);
  PULL_OFF(RX_UART1);
}

void init_serial2(void) {
  // Enable USART2 clock
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  // Configure the UART pins
  // AF 4bits per channel
  // Alternate functions from DM00115249.pdf datasheet (page 47; table 9)
  SET_AFR(TX_UART2, 0x7);
  SET_AFR(RX_UART2, 0x7);

  // Set pins to alternate function mode
  SET_MODE(TX_UART2, 0x2);
  SET_MODE(RX_UART2, 0x2);

  SET_OSPEED(TX_UART2, 0x3);
  SET_OSPEED(RX_UART2, 0x3);

  PULL_OFF(TX_UART2);
  PULL_OFF(RX_UART2);
}

void init_serial6(void) {
  // Enable USART6 clock
  RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

  // Configure the UART pins
  // AF 4bits per channel
  // Alternate functions from DM00115249.pdf datasheet (page 47; table 9)
  SET_AFR(TX_UART6, 0x8);
  SET_AFR(RX_UART6, 0x8);

  // Set pins to alternate function mode
  SET_MODE(TX_UART6, 0x2);
  SET_MODE(RX_UART6, 0x2);

  SET_OSPEED(TX_UART6, 0x3);
  SET_OSPEED(RX_UART6, 0x3);

  PULL_OFF(TX_UART6);
  PULL_OFF(RX_UART6);
}


void init_uart(USART_TypeDef *usartx) {

  if (usartx == USART1)
    init_serial1();
  else if (usartx == USART2)
    init_serial2();
  else if (usartx == USART6)
    init_serial6();

  uint32_t tempreg;

  /* Disable the peripheral */
  usartx->CR1 &=  ~USART_CR1_UE;

  /* Clear M, PCE, PS, TE and RE bits */
  tempreg = usartx->CR1;
  tempreg &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE |
               USART_CR1_RE | USART_CR1_OVER8);

  /* Configure the UART Word Length, Parity and mode:*/
  tempreg |= USART_CR1_RE | USART_CR1_TE;
  usartx->CR1 = tempreg;

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

  #define SERIAL_APB1CLK (_APB1_CLOCK)

  #define INT_DIVIDER ((25UL * SERIAL_APB1CLK) / (4 * BAUD))
  #define BAUD_H ((INT_DIVIDER / 100) << 4)
  #define FRACT_DIVIDER (INT_DIVIDER - (100 * (BAUD_H >> 4)))
  #define BAUD_L ((((FRACT_DIVIDER * 16) + 50) / 100) & 0X0F)

  #define SERIAL_APB2CLK (_APB2_CLOCK)

  #define INT_DIVIDER2 ((25UL * SERIAL_APB2CLK) / (4 * BAUD))
  #define BAUD_H2 ((INT_DIVIDER2 / 100) << 4)
  #define FRACT_DIVIDER2 (INT_DIVIDER2 - (100 * (BAUD_H2 >> 4)))
  #define BAUD_L2 ((((FRACT_DIVIDER2 * 16) + 50) / 100) & 0X0F)

  // USART2 is on APB1, USART1 and USART6 on APB2
  if (usartx == USART2)
    usartx->BRR = BAUD_H | BAUD_L;
  else
    usartx->BRR = BAUD_H2 | BAUD_L2;

  /* Clear STOP[13:12] bits */
  tempreg = usartx->CR2;
  tempreg &= ~(USART_CR2_STOP);

  /* In asynchronous mode, the following bits must be kept cleared: 
   - LINEN and CLKEN bits in the USART_CR2 register,
   - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  tempreg &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  usartx->CR2 = tempreg;

  tempreg = usartx->CR3;
  tempreg &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);

  /* Clear CTSE and RTSE bits */
  tempreg &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
  usartx->CR3 = tempreg;

  /* Enable the peripheral */
  usartx->CR1 |=  USART_CR1_UE;
}

void serial_init(){
  // Expand this list by adding UARTs
  // For example you can add extra USART for debugging.
  // In that case expand also the serial_XXcharS() below with new fuctions.
  init_uart(UART_SERIAL);
}

/** Check wether characters can be read.

  Other than the AVR implementation this returns not the number of characters
  in the line, but only wether there is at least one or not.
*/
uint8_t uartx_rxchars(USART_TypeDef* uart) {
  return uart->SR & USART_SR_RXNE;
}

/** Read one character.
*/
uint8_t uartx_popchar(USART_TypeDef* uart) {
  uint8_t c = 0;

  if (uartx_rxchars(uart))
    c = (uint8_t)(uart->DR & 0x1FF);

  return c;
}

/** Check wether characters can be written
*/
uint8_t uartx_txchars(USART_TypeDef* uart) {
  return uart->SR &USART_SR_TXE;
}
/** Send one character.
*/
void uartx_writechar(USART_TypeDef* uart, uint8_t data) {
  while ( !uartx_txchars(uart));       // Queue full?
  uart->DR = (uint32_t)(data & 0x1FF);
}

uint8_t serial_rxchars(void) {
  return uartx_rxchars(UART_SERIAL);
}
uint8_t serial_popchar(void) {
  return uartx_popchar(UART_SERIAL);
}
uint8_t serial_txchars(void) {
  return uartx_rxchars(UART_SERIAL);
}
void serial_writechar(uint8_t data) {
  uartx_writechar(UART_SERIAL, data);
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARM_STM32__ */