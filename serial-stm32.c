
/** \file
  \brief Serial subsystem, ARM specific part.

  To be included from serial.c, for more details see there.

  Other than AVRs, ARMs feature a serial buffer in hardware, so we can get
  away without a software buffer and also without(!) interrupts.

  Code here is heavily inspired by serial_api.c of MBED
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__
#include "arduino.h"

#ifdef XONXOFF
  #error XON/XOFF protocol not yet implemented for ARM. \
         See serial-avr.c for inspiration.
#endif

USART_TypeDef *port = USART2;

void serial_init()
{
    // Enable TX/RX clock (GPIOA)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure the UART pins
    // AF 4bits per channel
    // Alternate functions from DM00115249.pdf datasheet (page 47; table 9)
    TXD_PORT->AFR[0] |= (((uint8_t)0x07) << ((TXD_PIN) << 2));
    RXD_PORT->AFR[0] |= (((uint8_t)0x07) << ((RXD_PIN) << 2));

    // MODER 2bits per channel
    TXD_PORT->MODER |= (GPIO_MODE_AF_PP << ((TXD_PIN) << 1));          // set bit2: alternate function
    RXD_PORT->MODER |= (GPIO_MODE_AF_PP << ((RXD_PIN) << 1));

    TXD_PORT->OSPEEDR |= GPIO_SPEED_HIGH << ((TXD_PIN) << 1);
    RXD_PORT->OSPEEDR |= GPIO_SPEED_HIGH << ((RXD_PIN) << 1);

    TXD_PORT->PUPDR |= (GPIO_PULLUP << ((TXD_PIN) << 1));    //Pullup
    RXD_PORT->PUPDR |= (GPIO_PULLUP << ((RXD_PIN) << 1));    //Pullup? LPC has No Pull-up or Pull-down activation
    
    /* Disable the peripheral */
    port->CR1 &=  ~USART_CR1_UE;
  
    /* Set the UART Communication parameters */
    /*-------------------------- USART CR2 Configuration -----------------------*/
    /* Clear STOP[13:12] bits */
    port->CR2 &= ~(USART_CR2_STOP);

    /* Configure the UART Stop Bits: Set STOP[13:12] bits according to huart->Init.StopBits value */
    port->CR2 |= UART_STOPBITS_1;

    /*-------------------------- USART CR1 Configuration -----------------------*/
    /* Clear M, PCE, PS, TE and RE bits */
    port->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | \
                                   USART_CR1_RE | USART_CR1_OVER8);

    /* Configure the UART Word Length, Parity and mode: 
     Set the M bits according to huart->Init.WordLength value 
     Set PCE and PS bits according to huart->Init.Parity value
     Set TE and RE bits according to huart->Init.Mode value
     Set OVER8 bit according to huart->Init.OverSampling value */
    port->CR1 |= UART_WORDLENGTH_8B | UART_PARITY_NONE | UART_MODE_TX_RX | UART_OVERSAMPLING_16;

    /*-------------------------- USART CR3 Configuration -----------------------*/  
    /* Clear CTSE and RTSE bits */
    port->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);

    /* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */
    port->CR3 |= UART_HWCONTROL_NONE;

    /* 19.3.4 Fractional baud rate generation => reference manual for STM32F411
    Set BRR for 115,200 Hz 
    div = 48MHz/(16*BAUD)
    Mantisse = int(div) << 8
    Divisor = int((div - int(div))*16)
    BRR = Mantisse + Divisor
    */
    #define BAUD 115200

    #define SERIAL_APBCLK (__SYSTEM_CLOCK/2)

    #define INT_DIVIDER ((25 * SERIAL_APBCLK) / (4 * BAUD))
    #define BAUD_H ((INT_DIVIDER / 100) << 4)
    #define FRACT_DIVIDER (INT_DIVIDER - (100 * (BAUD_H >> 4)))
    #define BAUD_L (((((FRACT_DIVIDER * 16) + 50) / 100)) & 0X0F)

    port->BRR = BAUD_H | BAUD_L;
  
    /* In asynchronous mode, the following bits must be kept cleared: 
     - LINEN and CLKEN bits in the USART_CR2 register,
     - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
    port->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
    port->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
  
    /* Enable the peripheral */
    port->CR1 |=  USART_CR1_UE;
}

/** Check wether characters can be read.

  Other than the AVR implementation this returns not the number of characters
  in the line, but only wether there is at least one or not.
*/
uint8_t serial_rxchars(void) {
  return port->SR & UART_FLAG_RXNE;
}

/** Read one character.
*/
uint8_t serial_popchar(void) {
  uint8_t c = 0;

  if (serial_rxchars())
    c = (uint8_t)(port->DR & 0x1FF);

  return c;
}

/** Check wether characters can be written
*/
uint8_t serial_txchars(void) {
  return port->SR &UART_FLAG_TXE;
}
/** Send one character.
*/
void serial_writechar(uint8_t data) {
  while (!serial_txchars());
  port->DR = (uint32_t)(data & 0x1FF);
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__ */