#define NO_PWM_PIN      (uint8_t *)0
#define NO_TCCR_PIN     *(uint8_t *)0

// UART
#define RXD         DIO0
#define TXD         DIO1

// SPI
#define SCK         DIO13
#define MISO        DIO12
#define MOSI        DIO11
#define SS          DIO10

// TWI (I2C)
#define SCL         AIO5
#define SDA         AIO4

// timers and PWM
#define OC0A        DIO6
#define OC0B        DIO5
#define OC1A        DIO9
#define OC1B        DIO10
#define OC2A        DIO11
#define OC2B        DIO3


// digital pins
#define DIO0_PIN    PIND0
#define DIO0_RPORT  PIND
#define DIO0_WPORT  PORTD
#define DIO0_DDR    DDRD
#define DIO0_PWM     NO_PWM_PIN
#define DIO0_TCCR    NO_TCCR_PIN
#define DIO0_COM    0

#define DIO1_PIN    PIND1
#define DIO1_RPORT  PIND
#define DIO1_WPORT  PORTD
#define DIO1_DDR    DDRD
#define DIO1_PWM     NO_PWM_PIN
#define DIO1_TCCR    NO_TCCR_PIN
#define DIO1_COM    0

#define DIO2_PIN    PIND2
#define DIO2_RPORT  PIND
#define DIO2_WPORT  PORTD
#define DIO2_DDR    DDRD
#define DIO2_PWM     NO_PWM_PIN
#define DIO2_TCCR    NO_TCCR_PIN
#define DIO2_COM    0

#define DIO3_PIN    PIND3
#define DIO3_RPORT  PIND
#define DIO3_WPORT  PORTD
#define DIO3_DDR    DDRD
#define DIO3_PWM    &OCR2B
#define DIO3_TCCR   TCCR2A
#define DIO3_COM    COM2B1

#define DIO4_PIN    PIND4
#define DIO4_RPORT  PIND
#define DIO4_WPORT  PORTD
#define DIO4_DDR    DDRD
#define DIO4_PWM     NO_PWM_PIN
#define DIO4_TCCR    NO_TCCR_PIN
#define DIO4_COM    0

#define DIO5_PIN    PIND5
#define DIO5_RPORT  PIND
#define DIO5_WPORT  PORTD
#define DIO5_DDR    DDRD
#define DIO5_PWM    &OCR0B
#define DIO5_TCCR   TCCR0A
#define DIO5_COM    COM0B1

#define DIO6_PIN    PIND6
#define DIO6_RPORT  PIND
#define DIO6_WPORT  PORTD
#define DIO6_DDR    DDRD
#define DIO6_PWM    &OCR0A
#define DIO6_TCCR   TCCR0A
#define DIO6_COM    COM0A1

#define DIO7_PIN    PIND7
#define DIO7_RPORT  PIND
#define DIO7_WPORT  PORTD
#define DIO7_DDR    DDRD
#define DIO7_PWM     NO_PWM_PIN
#define DIO7_TCCR    NO_TCCR_PIN
#define DIO7_COM    0

#define DIO8_PIN    PINB0
#define DIO8_RPORT  PINB
#define DIO8_WPORT  PORTB
#define DIO8_DDR    DDRB
#define DIO8_PWM     NO_PWM_PIN
#define DIO8_TCCR    NO_TCCR_PIN
#define DIO8_COM    0

#define DIO9_PIN    PINB1
#define DIO9_RPORT  PINB
#define DIO9_WPORT  PORTB
#define DIO9_DDR    DDRB
#define DIO9_PWM     NO_PWM_PIN
#define DIO9_TCCR    NO_TCCR_PIN
#define DIO9_COM    0

#define DIO10_PIN   PINB2
#define DIO10_RPORT PINB
#define DIO10_WPORT PORTB
#define DIO10_DDR   DDRB
#define DIO10_PWM     NO_PWM_PIN
#define DIO10_TCCR    NO_TCCR_PIN
#define DIO10_COM   0

#define DIO11_PIN   PINB3
#define DIO11_RPORT PINB
#define DIO11_WPORT PORTB
#define DIO11_DDR   DDRB
#define DIO11_PWM   &OCR2A
#define DIO11_TCCR   TCCR2A
#define DIO11_COM    COM2A1

#define DIO12_PIN   PINB4
#define DIO12_RPORT PINB
#define DIO12_WPORT PORTB
#define DIO12_DDR   DDRB
#define DIO12_PWM     NO_PWM_PIN
#define DIO12_TCCR    NO_TCCR_PIN
#define DIO12_COM   0

#define DIO13_PIN   PINB5
#define DIO13_RPORT PINB
#define DIO13_WPORT PORTB
#define DIO13_DDR   DDRB
#define DIO13_PWM     NO_PWM_PIN
#define DIO13_TCCR    NO_TCCR_PIN
#define DIO13_COM   0

/**
  DIO14 ... DIO21 are added for compatibility with other
  firmwares and duplicate names for AIO0 ... AIO7,
  so DIO14 == AIO0, DIO15 == AIO1, DIO16 == AIO2, ...
*/
#define DIO14_PIN   PINC0
#define DIO14_RPORT PINC
#define DIO14_WPORT PORTC
#define DIO14_DDR   DDRC
#define DIO14_PWM     NO_PWM_PIN
#define DIO14_TCCR    NO_TCCR_PIN
#define DIO14_COM   0

#define DIO15_PIN   PINC1
#define DIO15_RPORT PINC
#define DIO15_WPORT PORTC
#define DIO15_DDR   DDRC
#define DIO15_PWM     NO_PWM_PIN
#define DIO15_TCCR    NO_TCCR_PIN
#define DIO15_COM   0

#define DIO16_PIN   PINC2
#define DIO16_RPORT PINC
#define DIO16_WPORT PORTC
#define DIO16_DDR   DDRC
#define DIO16_PWM     NO_PWM_PIN
#define DIO16_TCCR    NO_TCCR_PIN
#define DIO16_COM   0

#define DIO17_PIN   PINC3
#define DIO17_RPORT PINC
#define DIO17_WPORT PORTC
#define DIO17_DDR   DDRC
#define DIO17_PWM     NO_PWM_PIN
#define DIO17_TCCR    NO_TCCR_PIN
#define DIO17_COM   0

#define DIO18_PIN   PINC4
#define DIO18_RPORT PINC
#define DIO18_WPORT PORTC
#define DIO18_DDR   DDRC
#define DIO18_PWM     NO_PWM_PIN
#define DIO18_TCCR    NO_TCCR_PIN
#define DIO18_COM   0

#define DIO19_PIN   PINC5
#define DIO19_RPORT PINC
#define DIO19_WPORT PORTC
#define DIO19_DDR   DDRC
#define DIO19_PWM     NO_PWM_PIN
#define DIO19_TCCR    NO_TCCR_PIN
#define DIO19_COM   0

#define DIO20_PIN   PINC6
#define DIO20_RPORT PINC
#define DIO20_WPORT PORTC
#define DIO20_DDR   DDRC
#define DIO20_PWM     NO_PWM_PIN
#define DIO20_TCCR    NO_TCCR_PIN
#define DIO20_COM   0

#define DIO21_PIN   PINC7
#define DIO21_RPORT PINC
#define DIO21_WPORT PORTC
#define DIO21_DDR   DDRC
#define DIO21_PWM     NO_PWM_PIN
#define DIO21_TCCR    NO_TCCR_PIN
#define DIO21_COM   0


// analog pins
#define AIO0_PIN    PINC0
#define AIO0_RPORT  PINC
#define AIO0_WPORT  PORTC
#define AIO0_DDR    DDRC
#define AIO0_PWM     NO_PWM_PIN
#define AIO0_TCCR    NO_TCCR_PIN
#define AIO0_COM    0
#define AIO0_ADC    0

#define AIO1_PIN    PINC1
#define AIO1_RPORT  PINC
#define AIO1_WPORT  PORTC
#define AIO1_DDR    DDRC
#define AIO1_PWM     NO_PWM_PIN
#define AIO1_TCCR    NO_TCCR_PIN
#define AIO1_COM    0
#define AIO1_ADC    1

#define AIO2_PIN    PINC2
#define AIO2_RPORT  PINC
#define AIO2_WPORT  PORTC
#define AIO2_DDR    DDRC
#define AIO2_PWM     NO_PWM_PIN
#define AIO2_TCCR    NO_TCCR_PIN
#define AIO2_COM    0
#define AIO2_ADC    2

#define AIO3_PIN    PINC3
#define AIO3_RPORT  PINC
#define AIO3_WPORT  PORTC
#define AIO3_DDR    DDRC
#define AIO3_PWM     NO_PWM_PIN
#define AIO3_TCCR    NO_TCCR_PIN
#define AIO3_COM    0
#define AIO3_ADC    3

#define AIO4_PIN    PINC4
#define AIO4_RPORT  PINC
#define AIO4_WPORT  PORTC
#define AIO4_DDR    DDRC
#define AIO4_PWM     NO_PWM_PIN
#define AIO4_TCCR    NO_TCCR_PIN
#define AIO4_COM    0
#define AIO4_ADC    4

#define AIO5_PIN    PINC5
#define AIO5_RPORT  PINC
#define AIO5_WPORT  PORTC
#define AIO5_DDR    DDRC
#define AIO5_PWM     NO_PWM_PIN
#define AIO5_TCCR    NO_TCCR_PIN
#define AIO5_COM    0
#define AIO5_ADC    5

#define AIO6_PIN    PINC6
#define AIO6_RPORT  PINC
#define AIO6_WPORT  PORTC
#define AIO6_DDR    DDRC
#define AIO6_PWM     NO_PWM_PIN
#define AIO6_TCCR    NO_TCCR_PIN
#define AIO6_COM    0
#define AIO6_ADC    6

#define AIO7_PIN    PINC7
#define AIO7_RPORT  PINC
#define AIO7_WPORT  PORTC
#define AIO7_DDR    DDRC
#define AIO7_PWM     NO_PWM_PIN
#define AIO7_TCCR    NO_TCCR_PIN
#define AIO7_COM    0
#define AIO7_ADC    7



#undef PB0
#define PB0_PIN     PINB0
#define PB0_RPORT   PINB
#define PB0_WPORT   PORTB
#define PB0_DDR     DDRB
#define PB0_PWM     NO_PWM_PIN
#define PB0_TCCR    NO_TCCR_PIN
#define PB0_COM   0

#undef PB1
#define PB1_PIN     PINB1
#define PB1_RPORT   PINB
#define PB1_WPORT   PORTB
#define PB1_DDR     DDRB
#define PB1_PWM     NO_PWM_PIN
#define PB1_TCCR    NO_TCCR_PIN
#define PB1_COM   0

#undef PB2
#define PB2_PIN     PINB2
#define PB2_RPORT   PINB
#define PB2_WPORT   PORTB
#define PB2_DDR     DDRB
#define PB2_PWM     NO_PWM_PIN
#define PB2_TCCR    NO_TCCR_PIN
#define PB2_COM   0

#undef PB3
#define PB3_PIN     PINB3
#define PB3_RPORT   PINB
#define PB3_WPORT   PORTB
#define PB3_DDR     DDRB
#define PB3_PWM     &OCR2A
#define PB3_TCCR   TCCR2A
#define PB3_COM    COM2A1

#undef PB4
#define PB4_PIN     PINB4
#define PB4_RPORT   PINB
#define PB4_WPORT   PORTB
#define PB4_DDR     DDRB
#define PB4_PWM     NO_PWM_PIN
#define PB4_TCCR    NO_TCCR_PIN
#define PB4_COM   0

#undef PB5
#define PB5_PIN     PINB5
#define PB5_RPORT   PINB
#define PB5_WPORT   PORTB
#define PB5_DDR     DDRB
#define PB5_PWM     NO_PWM_PIN
#define PB5_TCCR    NO_TCCR_PIN
#define PB5_COM   0

#undef PB6
#define PB6_PIN     PINB6
#define PB6_RPORT   PINB
#define PB6_WPORT   PORTB
#define PB6_DDR     DDRB
#define PB6_PWM     NO_PWM_PIN
#define PB6_TCCR    NO_TCCR_PIN
#define PB6_COM   0

#undef PB7
#define PB7_PIN     PINB7
#define PB7_RPORT   PINB
#define PB7_WPORT   PORTB
#define PB7_DDR     DDRB
#define PB7_PWM     NO_PWM_PIN
#define PB7_TCCR    NO_TCCR_PIN
#define PB7_COM   0


#undef PC0
#define PC0_PIN     PINC0
#define PC0_RPORT   PINC
#define PC0_WPORT   PORTC
#define PC0_DDR     DDRC
#define PC0_PWM     NO_PWM_PIN
#define PC0_TCCR    NO_TCCR_PIN
#define PC0_COM   0

#undef PC1
#define PC1_PIN     PINC1
#define PC1_RPORT   PINC
#define PC1_WPORT   PORTC
#define PC1_DDR     DDRC
#define PC1_PWM     NO_PWM_PIN
#define PC1_TCCR    NO_TCCR_PIN
#define PC1_COM   0

#undef PC2
#define PC2_PIN     PINC2
#define PC2_RPORT   PINC
#define PC2_WPORT   PORTC
#define PC2_DDR     DDRC
#define PC2_PWM     NO_PWM_PIN
#define PC2_TCCR    NO_TCCR_PIN
#define PC2_COM   0

#undef PC3
#define PC3_PIN     PINC3
#define PC3_RPORT   PINC
#define PC3_WPORT   PORTC
#define PC3_DDR     DDRC
#define PC3_PWM     NO_PWM_PIN
#define PC3_TCCR    NO_TCCR_PIN
#define PC3_COM   0

#undef PC4
#define PC4_PIN     PINC4
#define PC4_RPORT   PINC
#define PC4_WPORT   PORTC
#define PC4_DDR     DDRC
#define PC4_PWM     NO_PWM_PIN
#define PC4_TCCR    NO_TCCR_PIN
#define PC4_COM   0

#undef PC5
#define PC5_PIN     PINC5
#define PC5_RPORT   PINC
#define PC5_WPORT   PORTC
#define PC5_DDR     DDRC
#define PC5_PWM     NO_PWM_PIN
#define PC5_TCCR    NO_TCCR_PIN
#define PC5_COM   0

#undef PC6
#define PC6_PIN     PINC6
#define PC6_RPORT   PINC
#define PC6_WPORT   PORTC
#define PC6_DDR     DDRC
#define PC6_PWM     NO_PWM_PIN
#define PC6_TCCR    NO_TCCR_PIN
#define PC6_COM   0

#undef PC7
#define PC7_PIN     PINC7
#define PC7_RPORT   PINC
#define PC7_WPORT   PORTC
#define PC7_DDR     DDRC
#define PC7_PWM     NO_PWM_PIN
#define PC7_TCCR    NO_TCCR_PIN
#define PC7_COM   0


#undef PD0
#define PD0_PIN     PIND0
#define PD0_RPORT   PIND
#define PD0_WPORT   PORTD
#define PD0_DDR     DDRD
#define PD0_PWM     NO_PWM_PIN
#define PD0_TCCR    NO_TCCR_PIN
#define PD0_COM   0

#undef PD1
#define PD1_PIN     PIND1
#define PD1_RPORT   PIND
#define PD1_WPORT   PORTD
#define PD1_DDR     DDRD
#define PD1_PWM     NO_PWM_PIN
#define PD1_TCCR    NO_TCCR_PIN
#define PD1_COM   0

#undef PD2
#define PD2_PIN     PIND2
#define PD2_RPORT   PIND
#define PD2_WPORT   PORTD
#define PD2_DDR     DDRD
#define PD2_PWM     NO_PWM_PIN
#define PD2_TCCR    NO_TCCR_PIN
#define PD2_COM   0

#undef PD3
#define PD3_PIN     PIND3
#define PD3_RPORT   PIND
#define PD3_WPORT   PORTD
#define PD3_DDR     DDRD
#define PD3_PWM     &OCR2B
#define PD3_TCCR   TCCR2A
#define PD3_COM    COM2B1

#undef PD4
#define PD4_PIN     PIND4
#define PD4_RPORT   PIND
#define PD4_WPORT   PORTD
#define PD4_DDR     DDRD
#define PD4_PWM     NO_PWM_PIN
#define PD4_TCCR    NO_TCCR_PIN
#define PD4_COM   0

#undef PD5
#define PD5_PIN     PIND5
#define PD5_RPORT   PIND
#define PD5_WPORT   PORTD
#define PD5_DDR     DDRD
#define PD5_PWM     &OCR0B
#define PD5_TCCR   TCCR0A
#define PD5_COM    COM0B1

#undef PD6
#define PD6_PIN     PIND6
#define PD6_RPORT   PIND
#define PD6_WPORT   PORTD
#define PD6_DDR     DDRD
#define PD6_PWM     &OCR0A
#define PD6_TCCR   TCCR0A
#define PD6_COM    COM0A1

#undef PD7
#define PD7_PIN     PIND7
#define PD7_RPORT   PIND
#define PD7_WPORT   PORTD
#define PD7_DDR     DDRD
#define PD7_PWM     NO_PWM_PIN
#define PD7_TCCR    NO_TCCR_PIN
#define PD7_COM   0
