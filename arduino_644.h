// UART
#define  RXD         DIO8
#define  TXD         DIO9
#define  RXD0        DIO8
#define  TXD0        DIO9

#define  RXD1        DIO10
#define  TXD1        DIO11

// SPI
#define  SCK         DIO7
#define  MISO        DIO6
#define  MOSI        DIO5
// TODO: This depends on the board, so this definition is misplaced here,
//       should be more appropriately named and go to config.h. It's used in
//       temp.c and simulator.h and defines the Chip Select pin for an eventual
//       MAX6675 temperature sensor.
#define  SS          DIO4

// TWI (I2C)
#define  SCL         DIO16
#define  SDA         DIO17

// timers and PWM
#define  OC0A        DIO3
#define  OC0B        DIO4
#define  OC1A        DIO13
#define  OC1B        DIO12
#define  OC2A        DIO15
#define  OC2B        DIO14


// digital pins
#define DIO0_PIN    PINB0
#define DIO0_RPORT  PINB
#define DIO0_WPORT  PORTB
#define DIO0_DDR    DDRB
#define DIO0_PWM    NULL

#define DIO1_PIN    PINB1
#define DIO1_RPORT  PINB
#define DIO1_WPORT  PORTB
#define DIO1_DDR    DDRB
#define DIO1_PWM    NULL

#define DIO2_PIN    PINB2
#define DIO2_RPORT  PINB
#define DIO2_WPORT  PORTB
#define DIO2_DDR    DDRB
#define DIO2_PWM    NULL

#define DIO3_PIN    PINB3
#define DIO3_RPORT  PINB
#define DIO3_WPORT  PORTB
#define DIO3_DDR    DDRB
#define DIO3_PWM    &OCR0A

#define DIO4_PIN    PINB4
#define DIO4_RPORT  PINB
#define DIO4_WPORT  PORTB
#define DIO4_DDR    DDRB
#define DIO4_PWM    &OCR0B

#define DIO5_PIN    PINB5
#define DIO5_RPORT  PINB
#define DIO5_WPORT  PORTB
#define DIO5_DDR    DDRB
#define DIO5_PWM    NULL

#define DIO6_PIN    PINB6
#define DIO6_RPORT  PINB
#define DIO6_WPORT  PORTB
#define DIO6_DDR    DDRB
#define DIO6_PWM    NULL

#define DIO7_PIN    PINB7
#define DIO7_RPORT  PINB
#define DIO7_WPORT  PORTB
#define DIO7_DDR    DDRB
#define DIO7_PWM    NULL

#define DIO8_PIN    PIND0
#define DIO8_RPORT  PIND
#define DIO8_WPORT  PORTD
#define DIO8_DDR    DDRD
#define DIO8_PWM    NULL

#define DIO9_PIN    PIND1
#define DIO9_RPORT  PIND
#define DIO9_WPORT  PORTD
#define DIO9_DDR    DDRD
#define DIO9_PWM    NULL

#define DIO10_PIN    PIND2
#define DIO10_RPORT  PIND
#define DIO10_WPORT  PORTD
#define DIO10_DDR    DDRD
#define DIO10_PWM    NULL

#define DIO11_PIN    PIND3
#define DIO11_RPORT  PIND
#define DIO11_WPORT  PORTD
#define DIO11_DDR    DDRD
#define DIO11_PWM    NULL

#define DIO12_PIN    PIND4
#define DIO12_RPORT  PIND
#define DIO12_WPORT  PORTD
#define DIO12_DDR    DDRD
#define DIO12_PWM    NULL

#define DIO13_PIN    PIND5
#define DIO13_RPORT  PIND
#define DIO13_WPORT  PORTD
#define DIO13_DDR    DDRD
#define DIO13_PWM    NULL

#define DIO14_PIN    PIND6
#define DIO14_RPORT  PIND
#define DIO14_WPORT  PORTD
#define DIO14_DDR    DDRD
#define DIO14_PWM    &OCR2B

#define DIO15_PIN    PIND7
#define DIO15_RPORT  PIND
#define DIO15_WPORT  PORTD
#define DIO15_DDR    DDRD
#define DIO15_PWM    &OCR2A

#define DIO16_PIN    PINC0
#define DIO16_RPORT  PINC
#define DIO16_WPORT  PORTC
#define DIO16_DDR    DDRC
#define DIO16_PWM    NULL

#define DIO17_PIN    PINC1
#define DIO17_RPORT  PINC
#define DIO17_WPORT  PORTC
#define DIO17_DDR    DDRC
#define DIO17_PWM    NULL

#define DIO18_PIN    PINC2
#define DIO18_RPORT  PINC
#define DIO18_WPORT  PORTC
#define DIO18_DDR    DDRC
#define DIO18_PWM    NULL

#define DIO19_PIN    PINC3
#define DIO19_RPORT  PINC
#define DIO19_WPORT  PORTC
#define DIO19_DDR    DDRC
#define DIO19_PWM    NULL

#define DIO20_PIN    PINC4
#define DIO20_RPORT  PINC
#define DIO20_WPORT  PORTC
#define DIO20_DDR    DDRC
#define DIO20_PWM    NULL

#define DIO21_PIN    PINC5
#define DIO21_RPORT  PINC
#define DIO21_WPORT  PORTC
#define DIO21_DDR    DDRC
#define DIO21_PWM    NULL

#define DIO22_PIN    PINC6
#define DIO22_RPORT  PINC
#define DIO22_WPORT  PORTC
#define DIO22_DDR    DDRC
#define DIO22_PWM    NULL

#define DIO23_PIN    PINC7
#define DIO23_RPORT  PINC
#define DIO23_WPORT  PORTC
#define DIO23_DDR    DDRC
#define DIO23_PWM    NULL

/**
  DIO24 ... DIO31 are duplicate names for AIO0 ... AIO7 in opposite order, so
  DIO24 == AIO7, DIO25 == AIO6, DIO26 == AIO5, DIO27 == AIO4,
  DIO28 == AIO3, DIO29 == AIO2, DIO30 == AIO1 and DIO31 == AIO0
*/
#define DIO24_PIN    PINA7
#define DIO24_RPORT  PINA
#define DIO24_WPORT  PORTA
#define DIO24_DDR    DDRA
#define DIO24_PWM    NULL

#define DIO25_PIN    PINA6
#define DIO25_RPORT  PINA
#define DIO25_WPORT  PORTA
#define DIO25_DDR    DDRA
#define DIO25_PWM    NULL

#define DIO26_PIN    PINA5
#define DIO26_RPORT  PINA
#define DIO26_WPORT  PORTA
#define DIO26_DDR    DDRA
#define DIO26_PWM    NULL

#define DIO27_PIN    PINA4
#define DIO27_RPORT  PINA
#define DIO27_WPORT  PORTA
#define DIO27_DDR    DDRA
#define DIO27_PWM    NULL

#define DIO28_PIN    PINA3
#define DIO28_RPORT  PINA
#define DIO28_WPORT  PORTA
#define DIO28_DDR    DDRA
#define DIO28_PWM    NULL

#define DIO29_PIN    PINA2
#define DIO29_RPORT  PINA
#define DIO29_WPORT  PORTA
#define DIO29_DDR    DDRA
#define DIO29_PWM    NULL

#define DIO30_PIN    PINA1
#define DIO30_RPORT  PINA
#define DIO30_WPORT  PORTA
#define DIO30_DDR    DDRA
#define DIO30_PWM    NULL

#define DIO31_PIN    PINA0
#define DIO31_RPORT  PINA
#define DIO31_WPORT  PORTA
#define DIO31_DDR    DDRA
#define DIO31_PWM    NULL


// analog pins
#define AIO0_PIN    PINA0
#define AIO0_RPORT  PINA
#define AIO0_WPORT  PORTA
#define AIO0_DDR    DDRA
#define AIO0_PWM    NULL
#define AIO0_ADC    0

#define AIO1_PIN    PINA1
#define AIO1_RPORT  PINA
#define AIO1_WPORT  PORTA
#define AIO1_DDR    DDRA
#define AIO1_PWM    NULL
#define AIO1_ADC    1

#define AIO2_PIN    PINA2
#define AIO2_RPORT  PINA
#define AIO2_WPORT  PORTA
#define AIO2_DDR    DDRA
#define AIO2_PWM    NULL
#define AIO2_ADC    2

#define AIO3_PIN    PINA3
#define AIO3_RPORT  PINA
#define AIO3_WPORT  PORTA
#define AIO3_DDR    DDRA
#define AIO3_PWM    NULL
#define AIO3_ADC    3

#define AIO4_PIN    PINA4
#define AIO4_RPORT  PINA
#define AIO4_WPORT  PORTA
#define AIO4_DDR    DDRA
#define AIO4_PWM    NULL
#define AIO4_ADC    4

#define AIO5_PIN    PINA5
#define AIO5_RPORT  PINA
#define AIO5_WPORT  PORTA
#define AIO5_DDR    DDRA
#define AIO5_PWM    NULL
#define AIO5_ADC    5

#define AIO6_PIN    PINA6
#define AIO6_RPORT  PINA
#define AIO6_WPORT  PORTA
#define AIO6_DDR    DDRA
#define AIO6_PWM    NULL
#define AIO6_ADC    6

#define AIO7_PIN    PINA7
#define AIO7_RPORT  PINA
#define AIO7_WPORT  PORTA
#define AIO7_DDR    DDRA
#define AIO7_PWM    NULL
#define AIO7_ADC    7



#undef PA0
#define PA0_PIN      PINA0
#define PA0_RPORT    PINA
#define PA0_WPORT    PORTA
#define PA0_DDR      DDRA
#define PA0_PWM      NULL

#undef PA1
#define PA1_PIN      PINA1
#define PA1_RPORT    PINA
#define PA1_WPORT    PORTA
#define PA1_DDR      DDRA
#define PA1_PWM      NULL

#undef PA2
#define PA2_PIN      PINA2
#define PA2_RPORT    PINA
#define PA2_WPORT    PORTA
#define PA2_DDR      DDRA
#define PA2_PWM      NULL

#undef PA3
#define PA3_PIN      PINA3
#define PA3_RPORT    PINA
#define PA3_WPORT    PORTA
#define PA3_DDR      DDRA
#define PA3_PWM      NULL

#undef PA4
#define PA4_PIN      PINA4
#define PA4_RPORT    PINA
#define PA4_WPORT    PORTA
#define PA4_DDR      DDRA
#define PA4_PWM      NULL

#undef PA5
#define PA5_PIN      PINA5
#define PA5_RPORT    PINA
#define PA5_WPORT    PORTA
#define PA5_DDR      DDRA
#define PA5_PWM      NULL

#undef PA6
#define PA6_PIN      PINA6
#define PA6_RPORT    PINA
#define PA6_WPORT    PORTA
#define PA6_DDR      DDRA
#define PA6_PWM      NULL

#undef PA7
#define PA7_PIN      PINA7
#define PA7_RPORT    PINA
#define PA7_WPORT    PORTA
#define PA7_DDR      DDRA
#define PA7_PWM      NULL


#undef PB0
#define PB0_PIN      PINB0
#define PB0_RPORT    PINB
#define PB0_WPORT    PORTB
#define PB0_DDR      DDRB
#define PB0_PWM      NULL

#undef PB1
#define PB1_PIN      PINB1
#define PB1_RPORT    PINB
#define PB1_WPORT    PORTB
#define PB1_DDR      DDRB
#define PB1_PWM      NULL

#undef PB2
#define PB2_PIN      PINB2
#define PB2_RPORT    PINB
#define PB2_WPORT    PORTB
#define PB2_DDR      DDRB
#define PB2_PWM      NULL

#undef PB3
#define PB3_PIN      PINB3
#define PB3_RPORT    PINB
#define PB3_WPORT    PORTB
#define PB3_DDR      DDRB
#define PB3_PWM      &OCR0A

#undef PB4
#define PB4_PIN      PINB4
#define PB4_RPORT    PINB
#define PB4_WPORT    PORTB
#define PB4_DDR      DDRB
#define PB4_PWM      &OCR0B

#undef PB5
#define PB5_PIN      PINB5
#define PB5_RPORT    PINB
#define PB5_WPORT    PORTB
#define PB5_DDR      DDRB
#define PB5_PWM      NULL

#undef PB6
#define PB6_PIN      PINB6
#define PB6_RPORT    PINB
#define PB6_WPORT    PORTB
#define PB6_DDR      DDRB
#define PB6_PWM      NULL

#undef PB7
#define PB7_PIN      PINB7
#define PB7_RPORT    PINB
#define PB7_WPORT    PORTB
#define PB7_DDR      DDRB
#define PB7_PWM      NULL


#undef PC0
#define PC0_PIN      PINC0
#define PC0_RPORT    PINC
#define PC0_WPORT    PORTC
#define PC0_DDR      DDRC
#define PC0_PWM      NULL

#undef PC1
#define PC1_PIN      PINC1
#define PC1_RPORT    PINC
#define PC1_WPORT    PORTC
#define PC1_DDR      DDRC
#define PC1_PWM      NULL

#undef PC2
#define PC2_PIN      PINC2
#define PC2_RPORT    PINC
#define PC2_WPORT    PORTC
#define PC2_DDR      DDRC
#define PC2_PWM      NULL

#undef PC3
#define PC3_PIN      PINC3
#define PC3_RPORT    PINC
#define PC3_WPORT    PORTC
#define PC3_DDR      DDRC
#define PC3_PWM      NULL

#undef PC4
#define PC4_PIN      PINC4
#define PC4_RPORT    PINC
#define PC4_WPORT    PORTC
#define PC4_DDR      DDRC
#define PC4_PWM      NULL

#undef PC5
#define PC5_PIN      PINC5
#define PC5_RPORT    PINC
#define PC5_WPORT    PORTC
#define PC5_DDR      DDRC
#define PC5_PWM      NULL

#undef PC6
#define PC6_PIN      PINC6
#define PC6_RPORT    PINC
#define PC6_WPORT    PORTC
#define PC6_DDR      DDRC
#define PC6_PWM      NULL

#undef PC7
#define PC7_PIN      PINC7
#define PC7_RPORT    PINC
#define PC7_WPORT    PORTC
#define PC7_DDR      DDRC
#define PC7_PWM      NULL


#undef PD0
#define PD0_PIN      PIND0
#define PD0_RPORT    PIND
#define PD0_WPORT    PORTD
#define PD0_DDR      DDRD
#define PD0_PWM      NULL

#undef PD1
#define PD1_PIN      PIND1
#define PD1_RPORT    PIND
#define PD1_WPORT    PORTD
#define PD1_DDR      DDRD
#define PD1_PWM      NULL

#undef PD2
#define PD2_PIN      PIND2
#define PD2_RPORT    PIND
#define PD2_WPORT    PORTD
#define PD2_DDR      DDRD
#define PD2_PWM      NULL

#undef PD3
#define PD3_PIN      PIND3
#define PD3_RPORT    PIND
#define PD3_WPORT    PORTD
#define PD3_DDR      DDRD
#define PD3_PWM      NULL

#undef PD4
#define PD4_PIN      PIND4
#define PD4_RPORT    PIND
#define PD4_WPORT    PORTD
#define PD4_DDR      DDRD
#define PD4_PWM      NULL

#undef PD5
#define PD5_PIN      PIND5
#define PD5_RPORT    PIND
#define PD5_WPORT    PORTD
#define PD5_DDR      DDRD
#define PD5_PWM      NULL

#undef PD6
#define PD6_PIN      PIND6
#define PD6_RPORT    PIND
#define PD6_WPORT    PORTD
#define PD6_DDR      DDRD
#define PD6_PWM      &OCR2B

#undef PD7
#define PD7_PIN      PIND7
#define PD7_RPORT    PIND
#define PD7_WPORT    PORTD
#define PD7_DDR      DDRD
#define PD7_PWM      &OCR2A
