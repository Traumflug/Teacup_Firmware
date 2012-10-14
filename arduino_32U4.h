// teensy pin assignments 

// UART  xxx checks this against USBs?
#define RXD          DIO7
#define TXD          DIO8
//#define RXD0         DIO7
//#define TXD0         DIO8

#define RXD1         DIO7
#define TXD1         DIO8

// ATmega32U4 doesn't have a serial port 0, maybe its a USB?
// do some munging of port 0 to port 1 the help serial.c
#define UCSR0A       UCSR1A
#define U2X0         U2X1
#define UBRR0        UBRR1
#define UCSR0B       UCSR1B
#define RXEN0        RXEN1
#define TXEN0        TXEN1
#define UCSR0C       UCSR1C
#define UCSZ01       UCSZ11
#define UCSZ00       UCSZ10
#define RXCIE0       RXCIE1
#define UDRIE0       UDRIE1
#define USART0_UDRE_vect USART1_UDRE_vect
#define UDR0         UDR1
#define UCSR0B       UCSR1B

// SPI
#define SCK          DIO1
#define MISO         DIO3
#define MOSI         DIO2
#define SS           DIO0

// TWI (I2C)
#define SCL          DIO5
#define SDA          DIO6

// timers and PWM
// The 32U4 PWMs are fairly restricted. If you avoid timer 1 for the clock, 
// two pairs of the PWMs are complements of each other:
// DIO11 & DIO12 on ~OC4D & OC4D and DIO14 & DIO15 on ~OC4B & OC4B
#define TIMER4_IS_10_BIT
#define OC0A         DIO4
#define OC0B         DIO5
#define OC3A         DIO9  // 10 bit timer on OC4A, inverse of PWM on DIO10
#define OC4A         D1O10
//#define OC4B         DIO14  // inverse of PWM on DIO15
#define OC4B         DIO15
//#define OC4D         DIO11  // inverse of PWM on DIO12  
#define OC4D         DIO12


// change for your board
#define DEBUG_LED        DIO22 /* led D11 red */
#define DEBUG_LED_PIN    DIO22

/*
pins
*/
#define DIO0_PIN     PINB0
#define DIO0_RPORT   PINB
#define DIO0_WPORT   PORTB
#define DIO0_PWM     NULL
#define DIO0_DDR     DDRB

#define DIO1_PIN     PINB1
#define DIO1_RPORT   PINB
#define DIO1_WPORT   PORTB
#define DIO1_PWM     NULL
#define DIO1_DDR     DDRB

#define DIO2_PIN     PINB2
#define DIO2_RPORT   PINB
#define DIO2_WPORT   PORTB
#define DIO2_PWM     NULL
#define DIO2_DDR     DDRB

#define DIO3_PIN     PINB3
#define DIO3_RPORT   PINB
#define DIO3_WPORT   PORTB
#define DIO3_PWM     NULL
#define DIO3_DDR     DDRB

#define DIO4_PIN     PINB7
#define DIO4_RPORT   PINB
#define DIO4_WPORT   PORTB
// teensy card says RTS OC1C OC0A PB7 PWM / ATMega32U4 datasheet says PCINT7/OC0A/OC1C/~RTS PB7
// Timer 1 would be a problem drf 2012-01-09
// Timer 0 might be used by serial.c 
#define DIO4_PWM     &OCR0A
#define DIO4_DDR     DDRB

#define DIO5_PIN     PIND0
#define DIO5_RPORT   PIND
#define DIO5_WPORT   PORTD
#define DIO5_PWM     &OCR0B
//#define DIO5_PWM     NULL
#define DIO5_DDR     DDRD

#define DIO6_PIN     PIND1
#define DIO6_RPORT   PIND
#define DIO6_WPORT   PORTD
#define DIO6_PWM     NULL
#define DIO6_DDR     DDRD

#define DIO7_PIN     PIND2
#define DIO7_RPORT   PIND
#define DIO7_WPORT   PORTD
#define DIO7_PWM     NULL
#define DIO7_DDR     DDRD

#define DIO8_PIN     PIND3
#define DIO8_RPORT   PIND
#define DIO8_WPORT   PORTD
#define DIO8_PWM     NULL
#define DIO8_DDR     DDRD

#define DIO9_PIN     PINC6
#define DIO9_RPORT   PINC
#define DIO9_WPORT   PORTC
#define DIO9_PWM     &OCR3AL  // inverse of DIO10_PWM
#define DIO9_DDR     DDRC

#define DIO10_PIN    PINC7
#define DIO10_RPORT  PINC
#define DIO10_WPORT  PORTC
// pin 10/c7 is on a 8/10 bit timer 4 and share the register with inverses
#define DIO10_PWM    &OCR4A
#define DIO10_DDR    DDRC

#define DIO11_PIN    PIND6
#define DIO11_RPORT  PIND
#define DIO11_WPORT  PORTD
#define DIO11_PWM    NULL  // inverse of DIO12_PWM
#define DIO11_DDR    DDRD

#define DIO12_PIN    PIND7
#define DIO12_RPORT  PIND
#define DIO12_WPORT  PORTD
#define DIO12_PWM    &OCR4D
#define DIO12_DDR    DDRD

#define DIO13_PIN    PINB4
#define DIO13_RPORT  PINB
#define DIO13_WPORT  PORTB
#define DIO13_PWM    NULL
#define DIO13_DDR    DDRB

#define DIO14_PIN    PINB5
#define DIO14_RPORT  PINB
#define DIO14_WPORT  PORTB
#define DIO14_PWM    NULL  // inverse of DIO15_PWM
#define DIO14_DDR    DDRB

#define DIO15_PIN    PINB6
#define DIO15_RPORT  PINB
#define DIO15_WPORT  PORTB
#define DIO15_PWM    &OCR4B
#define DIO15_DDR    DDRB

#define DIO16_PIN    PINF7
#define DIO16_RPORT  PINF
#define DIO16_WPORT  PORTF
#define DIO16_PWM    NULL
#define DIO16_DDR    DDRF

#define DIO17_PIN    PINF6
#define DIO17_RPORT  PINF
#define DIO17_WPORT  PORTF
#define DIO17_PWM    NULL
#define DIO17_DDR    DDRF

#define DIO18_PIN    PINF5
#define DIO18_RPORT  PINF
#define DIO18_WPORT  PORTF
#define DIO18_PWM    NULL
#define DIO18_DDR    DDRF

#define DIO19_PIN    PINF4
#define DIO19_RPORT  PINF
#define DIO19_WPORT  PORTF
#define DIO19_PWM    NULL
#define DIO19_DDR    DDRF

#define DIO20_PIN    PINF1
#define DIO20_RPORT  PINF
#define DIO20_WPORT  PORTF
#define DIO20_PWM    NULL
#define DIO20_DDR    DDRF

#define DIO21_PIN    PINF0
#define DIO21_RPORT  PINF
#define DIO21_WPORT  PORTF
#define DIO21_PWM    NULL
#define DIO21_DDR    DDRF

#define DIO22_PIN    PIND4
#define DIO22_RPORT  PIND
#define DIO22_WPORT  PORTD
#define DIO22_PWM    NULL
#define DIO22_DDR    DDRD

#define DIO23_PIN    PIND5
#define DIO23_RPORT  PIND
#define DIO23_WPORT  PORTD
#define DIO23_PWM    NULL
#define DIO23_DDR    DDRD

#define DIO24_PIN    PINE6
#define DIO24_RPORT  PINE
#define DIO24_WPORT  PORTE
#define DIO24_PWM    NULL
#define DIO24_DDR    DDRE

// ADC converters

#define AIO0_PIN     PINF0
#define AIO0_RPORT   PINF
#define AIO0_WPORT   PORTF
#define AIO0_PWM     NULL
#define AIO0_DDR     DDRF
#define AIO0_ADC     0

#define AIO1_PIN     PINF1
#define AIO1_RPORT   PINF
#define AIO1_WPORT   PORTF
#define AIO1_PWM     NULL
#define AIO1_DDR     DDRF
#define AIO1_ADC     1

#define AIO2_PIN     PINF4
#define AIO2_RPORT   PINF
#define AIO2_WPORT   PORTF
#define AIO2_PWM     NULL
#define AIO2_DDR     DDRF
#define AIO2_ADC     2

#define AIO3_PIN     PINF5
#define AIO3_RPORT   PINF
#define AIO3_WPORT   PORTF
#define AIO3_PWM     NULL
#define AIO3_DDR     DDRF
#define AIO3_ADC     3

#define AIO4_PIN     PINF6
#define AIO4_RPORT   PINF
#define AIO4_WPORT   PORTF
#define AIO4_PWM     NULL
#define AIO4_DDR     DDRF
#define AIO4_ADC     4

#define AIO5_PIN     PINF7
#define AIO5_RPORT   PINF
#define AIO5_WPORT   PORTF
#define AIO5_PWM     NULL
#define AIO5_DDR     DDRF
#define AIO5_ADC     5

#define AIO6_PIN     PINB6
#define AIO6_RPORT   PINB
#define AIO6_WPORT   PORTB
#define AIO6_PWM     NULL
#define AIO6_DDR     DDRB
#define AIO6_ADC     6

#define AIO7_PIN     PINB5
#define AIO7_RPORT   PINB
#define AIO7_WPORT   PORTB
#define AIO7_PWM     NULL
#define AIO7_DDR     DDRB
#define AIO7_ADC     7

#define AIO8_PIN     PINB4
#define AIO8_RPORT   PINB
#define AIO8_WPORT   PORTB
#define AIO8_PWM     NULL
#define AIO8_DDR     DDRB
#define AIO8_ADC     8

#define AIO9_PIN     PIND7
#define AIO9_RPORT   PIND
#define AIO9_WPORT   PORTD
#define AIO9_PWM     NULL
#define AIO9_DDR     DDRD
#define AIO9_ADC     9

#define AIO10_PIN    PIND6
#define AIO10_RPORT  PIND
#define AIO10_WPORT  PORTD
#define AIO10_PWM    NULL
#define AIO10_DDR    DDRD
#define AIO10_ADC    10

#define AIO11_PIN    PIND4
#define AIO11_RPORT  PIND
#define AIO11_WPORT  PORTD
#define AIO11_PWM    NULL
#define AIO11_DDR    DDRD
#define AIO11_ADC    11


// The PWM 

// Port A isn't defined

// Port B is fully available
#undef PB0
#define PB0_PIN      PINB0
#define PB0_RPORT    PINB
#define PB0_WPORT    PORTB
#define PB0_PWM      NULL
#define PB0_DDR      DDRB
#undef PB1
#define PB1_PIN      PINB1
#define PB1_RPORT    PINB
#define PB1_WPORT    PORTB
#define PB1_PWM      NULL
#define PB1_DDR      DDRB
#undef PB2
#define PB2_PIN      PINB2
#define PB2_RPORT    PINB
#define PB2_WPORT    PORTB
#define PB2_PWM      NULL
#define PB2_DDR      DDRB
#undef PB3
#define PB3_PIN      PINB3
#define PB3_RPORT    PINB
#define PB3_WPORT    PORTB
#define PB3_PWM      NULL
#define PB3_DDR      DDRB
#undef PB4
#define PB4_PIN      PINB4
#define PB4_RPORT    PINB
#define PB4_WPORT    PORTB
#define PB4_PWM      NULL
#define PB4_DDR      DDRB
#undef PB5
#define PB5_PIN      PINB5
#define PB5_RPORT    PINB
#define PB5_WPORT    PORTB
#define PB5_PWM      NULL
#define PB5_DDR      DDRB
#undef PB6
#define PB6_PIN      PINB6
#define PB6_RPORT    PINB
#define PB6_WPORT    PORTB
#define PB6_PWM      NULL
#define PB6_DDR      DDRB
#undef PB7
#define PB7_PIN      PINB7
#define PB7_RPORT    PINB
#define PB7_WPORT    PORTB
#define PB7_PWM      NULL
#define PB7_DDR      DDRB


// Port C has only bits 6&7 available on pins
#undef PC6
#define PC6_PIN      PINC6
#define PC6_RPORT    PINC
#define PC6_WPORT    PORTC
#define PC6_PWM      NULL
#define PC6_DDR      DDRC
#undef PC7
#define PC7_PIN      PINC7
#define PC7_RPORT    PINC
#define PC7_WPORT    PORTC
#define PC7_PWM      NULL
#define PC7_DDR      DDRC


//Port D is fully available
#undef PD0
#define PD0_PIN      PIND0
#define PD0_RPORT    PIND
#define PD0_WPORT    PORTD
#define PD0_PWM      NULL
#define PD0_DDR      DDRD
#undef PD1
#define PD1_PIN      PIND1
#define PD1_RPORT    PIND
#define PD1_WPORT    PORTD
#define PD1_PWM      NULL
#define PD1_DDR      DDRD
#undef PD2
#define PD2_PIN      PIND2
#define PD2_RPORT    PIND
#define PD2_WPORT    PORTD
#define PD2_PWM      NULL
#define PD2_DDR      DDRD
#undef PD3
#define PD3_PIN      PIND3
#define PD3_RPORT    PIND
#define PD3_WPORT    PORTD
#define PD3_PWM      NULL
#define PD3_DDR      DDRD
#undef PD4
#define PD4_PIN      PIND4
#define PD4_RPORT    PIND
#define PD4_WPORT    PORTD
#define PD4_PWM      NULL
#define PD4_DDR      DDRD
#undef PD5
#define PD5_PIN      PIND5
#define PD5_RPORT    PIND
#define PD5_WPORT    PORTD
#define PD5_PWM      NULL
#define PD5_DDR      DDRD
#undef PD6
#define PD6_PIN      PIND6
#define PD6_RPORT    PIND
#define PD6_WPORT    PORTD
#define PD6_PWM      NULL
#define PD6_DDR      DDRD
#undef PD7
#define PD7_PIN      PIND7
#define PD7_RPORT    PIND
#define PD7_WPORT    PORTD
#define PD7_PWM      NULL
#define PD7_DDR      DDRD

// Port E has only 2&6 available, does reset/bootload & be the ADC positive signal

#undef PE2
#define PE2_PIN      PINE2
#define PE2_RPORT    PINE
#define PE2_WPORT    PORTE
#define PE2_PWM      NULL
#define PE2_DDR      DDRE
#undef PE6
#define PE6_PIN      PINE6
#define PE6_RPORT    PINE
#define PE6_WPORT    PORTE
#define PE6_PWM      NULL
#define PE6_DDR      DDRE


//port F is missing bits 2 & 3 and does ADC and can do JTAG
#undef PF0
#define PF0_PIN      PINF0
#define PF0_RPORT    PINF
#define PF0_WPORT    PORTF
#define PF0_PWM      NULL
#define PF0_DDR      DDRF
#undef PF1
#define PF1_PIN      PINF1
#define PF1_RPORT    PINF
#define PF1_WPORT    PORTF
#define PF1_PWM      NULL
#define PF1_DDR      DDRF

#undef PF4
#define PF4_PIN      PINF4
#define PF4_RPORT    PINF
#define PF4_WPORT    PORTF
#define PF4_PWM      NULL
#define PF4_DDR      DDRF
#undef PF5
#define PF5_PIN      PINF5
#define PF5_RPORT    PINF
#define PF5_WPORT    PORTF
#define PF5_PWM      NULL
#define PF5_DDR      DDRF
#undef PF6
#define PF6_PIN      PINF6
#define PF6_RPORT    PINF
#define PF6_WPORT    PORTF
#define PF6_PWM      NULL
#define PF6_DDR      DDRF
#undef PF7
#define PF7_PIN      PINF7
#define PF7_RPORT    PINF
#define PF7_WPORT    PORTF
#define PF7_PWM      NULL
#define PF7_DDR      DDRF
