/* 
      These pin definitions match the AT90USB1286 as used in the 
      Teensy++ 2.0 carrier per http://www.pjrc.com/teensy/pinout.html

      See Sprinter's https://github.com/kliment/Sprinter/blob/master/Sprinter/fastio.h or 
      Marlin's lincomatic fork https://github.com/lincomatic/Marlin/blob/Marlin_v1/Marlin/fastio.h 

 */
#define NO_PWM_PIN      (uint8_t *)0
#define NO_TCCR_PIN     *(uint8_t *)0

// SPI
#define SCK            DIO9
#define MISO           DIO11
#define MOSI           DIO10
#define SS             DIO8


/*
pins
*/
#define DIO0_PIN       PIND0
#define DIO0_RPORT     PIND
#define DIO0_WPORT     PORTD
#define DIO0_PWM       &OCR0B
#define DIO0_TCCR   TCCR0A
#define DIO0_COM    COM0B1
#define DIO0_DDR       DDRD

#define DIO1_PIN       PIND1
#define DIO1_RPORT     PIND
#define DIO1_WPORT     PORTD
#define DIO1_PWM       &OCR2B
#define DIO1_TCCR   TCCR2A
#define DIO1_COM    COM2B1
#define DIO1_DDR       DDRD

#define DIO2_PIN       PIND2
#define DIO2_RPORT     PIND
#define DIO2_WPORT     PORTD
#define DIO2_PWM     NO_PWM_PIN
#define DIO2_TCCR    NO_TCCR_PIN
#define DIO2_COM    0
#define DIO2_DDR       DDRD

#define DIO3_PIN       PIND3
#define DIO3_RPORT     PIND
#define DIO3_WPORT     PORTD
#define DIO3_PWM     NO_PWM_PIN
#define DIO3_TCCR    NO_TCCR_PIN
#define DIO3_COM    0
#define DIO3_DDR       DDRD

#define DIO4_PIN       PIND4
#define DIO4_RPORT     PIND
#define DIO4_WPORT     PORTD
#define DIO4_PWM     NO_PWM_PIN
#define DIO4_TCCR    NO_TCCR_PIN
#define DIO4_COM    0
#define DIO4_DDR       DDRD

#define DIO5_PIN       PIND5
#define DIO5_RPORT     PIND
#define DIO5_WPORT     PORTD
#define DIO5_PWM     NO_PWM_PIN
#define DIO5_TCCR    NO_TCCR_PIN
#define DIO5_COM    0
#define DIO5_DDR       DDRD

#define DIO6_PIN       PIND6
#define DIO6_RPORT     PIND
#define DIO6_WPORT     PORTD
#define DIO6_PWM     NO_PWM_PIN
#define DIO6_TCCR    NO_TCCR_PIN
#define DIO6_COM    0
#define DIO6_DDR       DDRD

#define DIO7_PIN       PIND7
#define DIO7_RPORT     PIND
#define DIO7_WPORT     PORTD
#define DIO7_PWM     NO_PWM_PIN
#define DIO7_TCCR    NO_TCCR_PIN
#define DIO7_COM    0
#define DIO7_DDR       DDRD

#define DIO8_PIN       PINE0
#define DIO8_RPORT     PINE
#define DIO8_WPORT     PORTE
#define DIO8_PWM     NO_PWM_PIN
#define DIO8_TCCR    NO_TCCR_PIN
#define DIO8_COM    0
#define DIO8_DDR       DDRE

#define DIO9_PIN       PINE1
#define DIO9_RPORT     PINE
#define DIO9_WPORT     PORTE
#define DIO9_PWM     NO_PWM_PIN
#define DIO9_TCCR    NO_TCCR_PIN
#define DIO9_COM    0
#define DIO9_DDR       DDRE

#define DIO10_PIN      PINC0
#define DIO10_RPORT    PINC
#define DIO10_WPORT    PORTC
#define DIO10_PWM     NO_PWM_PIN
#define DIO10_TCCR    NO_TCCR_PIN
#define DIO10_COM   0
#define DIO10_DDR      DDRC

#define DIO11_PIN      PINC1
#define DIO11_RPORT    PINC
#define DIO11_WPORT    PORTC
#define DIO11_PWM     NO_PWM_PIN
#define DIO11_TCCR    NO_TCCR_PIN
#define DIO11_COM   0
#define DIO11_DDR      DDRC

#define DIO12_PIN      PINC2
#define DIO12_RPORT    PINC
#define DIO12_WPORT    PORTC
#define DIO12_PWM     NO_PWM_PIN
#define DIO12_TCCR    NO_TCCR_PIN
#define DIO12_COM   0
#define DIO12_DDR      DDRC

#define DIO13_PIN      PINC3
#define DIO13_RPORT    PINC
#define DIO13_WPORT    PORTC
#define DIO13_PWM     NO_PWM_PIN
#define DIO13_TCCR    NO_TCCR_PIN
#define DIO13_COM   0
#define DIO13_DDR      DDRC

#define DIO14_PIN      PINC4
#define DIO14_RPORT    PINC
#define DIO14_WPORT    PORTC
#define DIO14_PWM      &OCR3CL
#define DIO14_TCCR   TCCR3A
#define DIO14_COM    COM3C1
#define DIO14_DDR      DDRC

#define DIO15_PIN      PINC5
#define DIO15_RPORT    PINC
#define DIO15_WPORT    PORTC
#define DIO15_PWM      &OCR3BL
#define DIO15_TCCR   TCCR3A
#define DIO15_COM    COM3B1
#define DIO15_DDR      DDRC

#define DIO16_PIN      PINC6
#define DIO16_RPORT    PINC
#define DIO16_WPORT    PORTC
#define DIO16_PWM      &OCR3AL
#define DIO16_TCCR   TCCR3A
#define DIO16_COM    COM3A1
#define DIO16_DDR      DDRC

#define DIO17_PIN      PINC7
#define DIO17_RPORT    PINC
#define DIO17_WPORT    PORTC
#define DIO17_PWM     NO_PWM_PIN
#define DIO17_TCCR    NO_TCCR_PIN
#define DIO17_COM   0
#define DIO17_DDR      DDRC

#define DIO18_PIN      PINE6
#define DIO18_RPORT    PINE
#define DIO18_WPORT    PORTE
#define DIO18_PWM     NO_PWM_PIN
#define DIO18_TCCR    NO_TCCR_PIN
#define DIO18_COM   0
#define DIO18_DDR      DDRE

#define DIO19_PIN      PINE7
#define DIO19_RPORT    PINE
#define DIO19_WPORT    PORTE
#define DIO19_PWM     NO_PWM_PIN
#define DIO19_TCCR    NO_TCCR_PIN
#define DIO19_COM   0
#define DIO19_DDR      DDRE

#define DIO20_PIN      PINB0
#define DIO20_RPORT    PINB
#define DIO20_WPORT    PORTB
#define DIO20_PWM     NO_PWM_PIN
#define DIO20_TCCR    NO_TCCR_PIN
#define DIO20_COM   0
#define DIO20_DDR      DDRB

#define DIO21_PIN      PINB1
#define DIO21_RPORT    PINB
#define DIO21_WPORT    PORTB
#define DIO21_PWM     NO_PWM_PIN
#define DIO21_TCCR    NO_TCCR_PIN
#define DIO21_COM   0
#define DIO21_DDR      DDRB

#define DIO22_PIN      PINB2
#define DIO22_RPORT    PINB
#define DIO22_WPORT    PORTB
#define DIO22_PWM     NO_PWM_PIN
#define DIO22_TCCR    NO_TCCR_PIN
#define DIO22_COM   0
#define DIO22_DDR      DDRB

#define DIO23_PIN      PINB3
#define DIO23_RPORT    PINB
#define DIO23_WPORT    PORTB
#define DIO23_PWM     NO_PWM_PIN
#define DIO23_TCCR    NO_TCCR_PIN
#define DIO23_COM   0
#define DIO23_DDR      DDRB

#define DIO24_PIN      PINB4
#define DIO24_RPORT    PINB
#define DIO24_WPORT    PORTB
#define DIO24_PWM      &OCR2A
#define DIO24_TCCR   TCCR2A
#define DIO24_COM    COM2A1
#define DIO24_DDR      DDRB

#define DIO25_PIN      PINB5
#define DIO25_RPORT    PINB
#define DIO25_WPORT    PORTB
#define DIO25_PWM     NO_PWM_PIN /* &OCR1A would interfere with timer.c */
#define DIO25_TCCR    NO_TCCR_PIN
#define DIO25_COM   0
#define DIO25_DDR      DDRB

#define DIO26_PIN      PINB6
#define DIO26_RPORT    PINB
#define DIO26_WPORT    PORTB
#define DIO26_PWM     NO_PWM_PIN /* &OCR1B would interfere with timer.c */
#define DIO26_TCCR    NO_TCCR_PIN
#define DIO26_COM   0
#define DIO26_DDR      DDRB

#define DIO27_PIN      PINB7
#define DIO27_RPORT    PINB
#define DIO27_WPORT    PORTB
#define DIO27_PWM      &OCR0A
#define DIO27_TCCR   TCCR0A
#define DIO27_COM    COM0A1
#define DIO27_DDR      DDRB

#define DIO28_PIN      PINA0
#define DIO28_RPORT    PINA
#define DIO28_WPORT    PORTA
#define DIO28_PWM     NO_PWM_PIN
#define DIO28_TCCR    NO_TCCR_PIN
#define DIO28_COM   0
#define DIO28_DDR      DDRA

#define DIO29_PIN      PINA1
#define DIO29_RPORT    PINA
#define DIO29_WPORT    PORTA
#define DIO29_PWM     NO_PWM_PIN
#define DIO29_TCCR    NO_TCCR_PIN
#define DIO29_COM   0
#define DIO29_DDR      DDRA

#define DIO30_PIN      PINA2
#define DIO30_RPORT    PINA
#define DIO30_WPORT    PORTA
#define DIO30_PWM     NO_PWM_PIN
#define DIO30_TCCR    NO_TCCR_PIN
#define DIO30_COM   0
#define DIO30_DDR      DDRA

#define DIO31_PIN      PINA3
#define DIO31_RPORT    PINA
#define DIO31_WPORT    PORTA
#define DIO31_PWM     NO_PWM_PIN
#define DIO31_TCCR    NO_TCCR_PIN
#define DIO31_COM   0
#define DIO31_DDR      DDRA

#define DIO32_PIN      PINA4
#define DIO32_RPORT    PINA
#define DIO32_WPORT    PORTA
#define DIO32_PWM     NO_PWM_PIN
#define DIO32_TCCR    NO_TCCR_PIN
#define DIO32_COM   0
#define DIO32_DDR      DDRA

#define DIO33_PIN      PINA5
#define DIO33_RPORT    PINA
#define DIO33_WPORT    PORTA
#define DIO33_PWM     NO_PWM_PIN
#define DIO33_TCCR    NO_TCCR_PIN
#define DIO33_COM   0
#define DIO33_DDR      DDRA

#define DIO34_PIN      PINA6
#define DIO34_RPORT    PINA
#define DIO34_WPORT    PORTA
#define DIO34_PWM     NO_PWM_PIN
#define DIO34_TCCR    NO_TCCR_PIN
#define DIO34_COM   0
#define DIO34_DDR      DDRA

#define DIO35_PIN      PINA7
#define DIO35_RPORT    PINA
#define DIO35_WPORT    PORTA
#define DIO35_PWM     NO_PWM_PIN
#define DIO35_TCCR    NO_TCCR_PIN
#define DIO35_COM   0
#define DIO35_DDR      DDRA

#define DIO36_PIN      PINE4
#define DIO36_RPORT    PINE
#define DIO36_WPORT    PORTE
#define DIO36_PWM     NO_PWM_PIN
#define DIO36_TCCR    NO_TCCR_PIN
#define DIO36_COM   0
#define DIO36_DDR      DDRE

#define DIO37_PIN      PINE5
#define DIO37_RPORT    PINE
#define DIO37_WPORT    PORTE
#define DIO37_PWM     NO_PWM_PIN
#define DIO37_TCCR    NO_TCCR_PIN
#define DIO37_COM   0
#define DIO37_DDR      DDRE

#define DIO38_PIN      PINF0
#define DIO38_RPORT    PINF
#define DIO38_WPORT    PORTF
#define DIO38_PWM     NO_PWM_PIN
#define DIO38_TCCR    NO_TCCR_PIN
#define DIO38_COM   0
#define DIO38_DDR      DDRF

#define DIO39_PIN      PINF1
#define DIO39_RPORT    PINF
#define DIO39_WPORT    PORTF
#define DIO39_PWM     NO_PWM_PIN
#define DIO39_TCCR    NO_TCCR_PIN
#define DIO39_COM   0
#define DIO39_DDR      DDRF

#define AIO0_PIN       PINF0
#define AIO0_RPORT     PINF
#define AIO0_WPORT     PORTF
#define AIO0_PWM     NO_PWM_PIN
#define AIO0_TCCR    NO_TCCR_PIN
#define AIO0_COM    0
#define AIO0_DDR       DDRF
#define AIO0_ADC       0

#define AIO1_PIN       PINF1
#define AIO1_RPORT     PINF
#define AIO1_WPORT     PORTF
#define AIO1_PWM     NO_PWM_PIN
#define AIO1_TCCR    NO_TCCR_PIN
#define AIO1_COM    0
#define AIO1_DDR       DDRF
#define AIO1_ADC       1

#define AIO2_PIN       PINF2
#define AIO2_RPORT     PINF
#define AIO2_WPORT     PORTF
#define AIO2_PWM     NO_PWM_PIN
#define AIO2_TCCR    NO_TCCR_PIN
#define AIO2_COM    0
#define AIO2_DDR       DDRF
#define AIO2_ADC       2

#define AIO3_PIN       PINF3
#define AIO3_RPORT     PINF
#define AIO3_WPORT     PORTF
#define AIO3_PWM     NO_PWM_PIN
#define AIO3_TCCR    NO_TCCR_PIN
#define AIO3_COM    0
#define AIO3_DDR       DDRF
#define AIO3_ADC       3

#define AIO4_PIN       PINF4
#define AIO4_RPORT     PINF
#define AIO4_WPORT     PORTF
#define AIO4_PWM     NO_PWM_PIN
#define AIO4_TCCR    NO_TCCR_PIN
#define AIO4_COM    0
#define AIO4_DDR       DDRF
#define AIO4_ADC       4

#define AIO5_PIN       PINF5
#define AIO5_RPORT     PINF
#define AIO5_WPORT     PORTF
#define AIO5_PWM     NO_PWM_PIN
#define AIO5_TCCR    NO_TCCR_PIN
#define AIO5_COM    0
#define AIO5_DDR       DDRF
#define AIO5_ADC       5

#define AIO6_PIN       PINF6
#define AIO6_RPORT     PINF
#define AIO6_WPORT     PORTF
#define AIO6_PWM     NO_PWM_PIN
#define AIO6_TCCR    NO_TCCR_PIN
#define AIO6_COM    0
#define AIO6_DDR       DDRF
#define AIO6_ADC       6

#define AIO7_PIN       PINF7
#define AIO7_RPORT     PINF
#define AIO7_WPORT     PORTF
#define AIO7_PWM     NO_PWM_PIN
#define AIO7_TCCR    NO_TCCR_PIN
#define AIO7_COM    0
#define AIO7_DDR       DDRF
#define AIO7_ADC       7

#define DIO40_PIN      PINF2
#define DIO40_RPORT    PINF
#define DIO40_WPORT    PORTF
#define DIO40_PWM     NO_PWM_PIN
#define DIO40_TCCR    NO_TCCR_PIN
#define DIO40_COM   0
#define DIO40_DDR      DDRF

#define DIO41_PIN      PINF3
#define DIO41_RPORT    PINF
#define DIO41_WPORT    PORTF
#define DIO41_PWM     NO_PWM_PIN
#define DIO41_TCCR    NO_TCCR_PIN
#define DIO41_COM   0
#define DIO41_DDR      DDRF

#define DIO42_PIN      PINF4
#define DIO42_RPORT    PINF
#define DIO42_WPORT    PORTF
#define DIO42_PWM     NO_PWM_PIN
#define DIO42_TCCR    NO_TCCR_PIN
#define DIO42_COM   0
#define DIO42_DDR      DDRF

#define DIO43_PIN      PINF5
#define DIO43_RPORT    PINF
#define DIO43_WPORT    PORTF
#define DIO43_PWM     NO_PWM_PIN
#define DIO43_TCCR    NO_TCCR_PIN
#define DIO43_COM   0
#define DIO43_DDR      DDRF

#define DIO44_PIN      PINF6
#define DIO44_RPORT    PINF
#define DIO44_WPORT    PORTF
#define DIO44_PWM     NO_PWM_PIN
#define DIO44_TCCR    NO_TCCR_PIN
#define DIO44_COM   0
#define DIO44_DDR      DDRF

#define DIO45_PIN      PINF7
#define DIO45_RPORT    PINF
#define DIO45_WPORT    PORTF
#define DIO45_PWM     NO_PWM_PIN
#define DIO45_TCCR    NO_TCCR_PIN
#define DIO45_COM   0
#define DIO45_DDR      DDRF


#undef PA0
#define PA0_PIN        PINA0
#define PA0_RPORT      PINA
#define PA0_WPORT      PORTA
#define PA0_PWM     NO_PWM_PIN
#define PA0_TCCR    NO_TCCR_PIN
#define PA0_COM   0
#define PA0_DDR        DDRA
#undef PA1
#define PA1_PIN        PINA1
#define PA1_RPORT      PINA
#define PA1_WPORT      PORTA
#define PA1_PWM     NO_PWM_PIN
#define PA1_TCCR    NO_TCCR_PIN
#define PA1_COM   0
#define PA1_DDR        DDRA
#undef PA2
#define PA2_PIN        PINA2
#define PA2_RPORT      PINA
#define PA2_WPORT      PORTA
#define PA2_PWM     NO_PWM_PIN
#define PA2_TCCR    NO_TCCR_PIN
#define PA2_COM   0
#define PA2_DDR        DDRA
#undef PA3
#define PA3_PIN        PINA3
#define PA3_RPORT      PINA
#define PA3_WPORT      PORTA
#define PA3_PWM     NO_PWM_PIN
#define PA3_TCCR    NO_TCCR_PIN
#define PA3_COM   0
#define PA3_DDR        DDRA
#undef PA4
#define PA4_PIN        PINA4
#define PA4_RPORT      PINA
#define PA4_WPORT      PORTA
#define PA4_PWM     NO_PWM_PIN
#define PA4_TCCR    NO_TCCR_PIN
#define PA4_COM   0
#define PA4_DDR        DDRA
#undef PA5
#define PA5_PIN        PINA5
#define PA5_RPORT      PINA
#define PA5_WPORT      PORTA
#define PA5_PWM     NO_PWM_PIN
#define PA5_TCCR    NO_TCCR_PIN
#define PA5_COM   0
#define PA5_DDR        DDRA
#undef PA6
#define PA6_PIN        PINA6
#define PA6_RPORT      PINA
#define PA6_WPORT      PORTA
#define PA6_PWM     NO_PWM_PIN
#define PA6_TCCR    NO_TCCR_PIN
#define PA6_COM   0
#define PA6_DDR        DDRA
#undef PA7
#define PA7_PIN        PINA7
#define PA7_RPORT      PINA
#define PA7_WPORT      PORTA
#define PA7_PWM     NO_PWM_PIN
#define PA7_TCCR    NO_TCCR_PIN
#define PA7_COM   0
#define PA7_DDR        DDRA

#undef PB0
#define PB0_PIN        PINB0
#define PB0_RPORT      PINB
#define PB0_WPORT      PORTB
#define PB0_PWM     NO_PWM_PIN
#define PB0_TCCR    NO_TCCR_PIN
#define PB0_COM   0
#define PB0_DDR        DDRB
#undef PB1
#define PB1_PIN        PINB1
#define PB1_RPORT      PINB
#define PB1_WPORT      PORTB
#define PB1_PWM     NO_PWM_PIN
#define PB1_TCCR    NO_TCCR_PIN
#define PB1_COM   0
#define PB1_DDR        DDRB
#undef PB2
#define PB2_PIN        PINB2
#define PB2_RPORT      PINB
#define PB2_WPORT      PORTB
#define PB2_PWM     NO_PWM_PIN
#define PB2_TCCR    NO_TCCR_PIN
#define PB2_COM   0
#define PB2_DDR        DDRB
#undef PB3
#define PB3_PIN        PINB3
#define PB3_RPORT      PINB
#define PB3_WPORT      PORTB
#define PB3_PWM     NO_PWM_PIN
#define PB3_TCCR    NO_TCCR_PIN
#define PB3_COM   0
#define PB3_DDR        DDRB
#undef PB4
#define PB4_PIN        PINB4
#define PB4_RPORT      PINB
#define PB4_WPORT      PORTB
#define PB4_PWM     NO_PWM_PIN
#define PB4_TCCR    NO_TCCR_PIN
#define PB4_COM   0
#define PB4_DDR        DDRB
#undef PB5
#define PB5_PIN        PINB5
#define PB5_RPORT      PINB
#define PB5_WPORT      PORTB
#define PB5_PWM     NO_PWM_PIN
#define PB5_TCCR    NO_TCCR_PIN
#define PB5_COM   0
#define PB5_DDR        DDRB
#undef PB6
#define PB6_PIN        PINB6
#define PB6_RPORT      PINB
#define PB6_WPORT      PORTB
#define PB6_PWM     NO_PWM_PIN
#define PB6_TCCR    NO_TCCR_PIN
#define PB6_COM   0
#define PB6_DDR        DDRB
#undef PB7
#define PB7_PIN        PINB7
#define PB7_RPORT      PINB
#define PB7_WPORT      PORTB
#define PB7_PWM     NO_PWM_PIN
#define PB7_TCCR    NO_TCCR_PIN
#define PB7_COM   0
#define PB7_DDR        DDRB

#undef PC0
#define PC0_PIN        PINC0
#define PC0_RPORT      PINC
#define PC0_WPORT      PORTC
#define PC0_PWM     NO_PWM_PIN
#define PC0_TCCR    NO_TCCR_PIN
#define PC0_COM   0
#define PC0_DDR        DDRC
#undef PC1
#define PC1_PIN        PINC1
#define PC1_RPORT      PINC
#define PC1_WPORT      PORTC
#define PC1_PWM     NO_PWM_PIN
#define PC1_TCCR    NO_TCCR_PIN
#define PC1_COM   0
#define PC1_DDR        DDRC
#undef PC2
#define PC2_PIN        PINC2
#define PC2_RPORT      PINC
#define PC2_WPORT      PORTC
#define PC2_PWM     NO_PWM_PIN
#define PC2_TCCR    NO_TCCR_PIN
#define PC2_COM   0
#define PC2_DDR        DDRC
#undef PC3
#define PC3_PIN        PINC3
#define PC3_RPORT      PINC
#define PC3_WPORT      PORTC
#define PC3_PWM     NO_PWM_PIN
#define PC3_TCCR    NO_TCCR_PIN
#define PC3_COM   0
#define PC3_DDR        DDRC
#undef PC4
#define PC4_PIN        PINC4
#define PC4_RPORT      PINC
#define PC4_WPORT      PORTC
#define PC4_PWM     NO_PWM_PIN
#define PC4_TCCR    NO_TCCR_PIN
#define PC4_COM   0
#define PC4_DDR        DDRC
#undef PC5
#define PC5_PIN        PINC5
#define PC5_RPORT      PINC
#define PC5_WPORT      PORTC
#define PC5_PWM     NO_PWM_PIN
#define PC5_TCCR    NO_TCCR_PIN
#define PC5_COM   0
#define PC5_DDR        DDRC
#undef PC6
#define PC6_PIN        PINC6
#define PC6_RPORT      PINC
#define PC6_WPORT      PORTC
#define PC6_PWM     NO_PWM_PIN
#define PC6_TCCR    NO_TCCR_PIN
#define PC6_COM   0
#define PC6_DDR        DDRC
#undef PC7
#define PC7_PIN        PINC7
#define PC7_RPORT      PINC
#define PC7_WPORT      PORTC
#define PC7_PWM     NO_PWM_PIN
#define PC7_TCCR    NO_TCCR_PIN
#define PC7_COM   0
#define PC7_DDR        DDRC

#undef PD0
#define PD0_PIN        PIND0
#define PD0_RPORT      PIND
#define PD0_WPORT      PORTD
#define PD0_PWM     NO_PWM_PIN
#define PD0_TCCR    NO_TCCR_PIN
#define PD0_COM   0
#define PD0_DDR        DDRD
#undef PD1
#define PD1_PIN        PIND1
#define PD1_RPORT      PIND
#define PD1_WPORT      PORTD
#define PD1_PWM     NO_PWM_PIN
#define PD1_TCCR    NO_TCCR_PIN
#define PD1_COM   0
#define PD1_DDR        DDRD
#undef PD2
#define PD2_PIN        PIND2
#define PD2_RPORT      PIND
#define PD2_WPORT      PORTD
#define PD2_PWM     NO_PWM_PIN
#define PD2_TCCR    NO_TCCR_PIN
#define PD2_COM   0
#define PD2_DDR        DDRD
#undef PD3
#define PD3_PIN        PIND3
#define PD3_RPORT      PIND
#define PD3_WPORT      PORTD
#define PD3_PWM     NO_PWM_PIN
#define PD3_TCCR    NO_TCCR_PIN
#define PD3_COM   0
#define PD3_DDR        DDRD
#undef PD4
#define PD4_PIN        PIND4
#define PD4_RPORT      PIND
#define PD4_WPORT      PORTD
#define PD4_PWM     NO_PWM_PIN
#define PD4_TCCR    NO_TCCR_PIN
#define PD4_COM   0
#define PD4_DDR        DDRD
#undef PD5
#define PD5_PIN        PIND5
#define PD5_RPORT      PIND
#define PD5_WPORT      PORTD
#define PD5_PWM     NO_PWM_PIN
#define PD5_TCCR    NO_TCCR_PIN
#define PD5_COM   0
#define PD5_DDR        DDRD
#undef PD6
#define PD6_PIN        PIND6
#define PD6_RPORT      PIND
#define PD6_WPORT      PORTD
#define PD6_PWM     NO_PWM_PIN
#define PD6_TCCR    NO_TCCR_PIN
#define PD6_COM   0
#define PD6_DDR        DDRD
#undef PD7
#define PD7_PIN        PIND7
#define PD7_RPORT      PIND
#define PD7_WPORT      PORTD
#define PD7_PWM     NO_PWM_PIN
#define PD7_TCCR    NO_TCCR_PIN
#define PD7_COM   0
#define PD7_DDR        DDRD

#undef PE0
#define PE0_PIN        PINE0
#define PE0_RPORT      PINE
#define PE0_WPORT      PORTE
#define PE0_PWM     NO_PWM_PIN
#define PE0_TCCR    NO_TCCR_PIN
#define PE0_COM   0
#define PE0_DDR        DDRE
#undef PE1
#define PE1_PIN        PINE1
#define PE1_RPORT      PINE
#define PE1_WPORT      PORTE
#define PE1_PWM     NO_PWM_PIN
#define PE1_TCCR    NO_TCCR_PIN
#define PE1_COM   0
#define PE1_DDR        DDRE
#undef PE2
#define PE2_PIN        PINE2
#define PE2_RPORT      PINE
#define PE2_WPORT      PORTE
#define PE2_PWM     NO_PWM_PIN
#define PE2_TCCR    NO_TCCR_PIN
#define PE2_COM   0
#define PE2_DDR        DDRE
#undef PE3
#define PE3_PIN        PINE3
#define PE3_RPORT      PINE
#define PE3_WPORT      PORTE
#define PE3_PWM     NO_PWM_PIN
#define PE3_TCCR    NO_TCCR_PIN
#define PE3_COM   0
#define PE3_DDR        DDRE
#undef PE4
#define PE4_PIN        PINE4
#define PE4_RPORT      PINE
#define PE4_WPORT      PORTE
#define PE4_PWM     NO_PWM_PIN
#define PE4_TCCR    NO_TCCR_PIN
#define PE4_COM   0
#define PE4_DDR        DDRE
#undef PE5
#define PE5_PIN        PINE5
#define PE5_RPORT      PINE
#define PE5_WPORT      PORTE
#define PE5_PWM     NO_PWM_PIN
#define PE5_TCCR    NO_TCCR_PIN
#define PE5_COM   0
#define PE5_DDR        DDRE
#undef PE6
#define PE6_PIN        PINE6
#define PE6_RPORT      PINE
#define PE6_WPORT      PORTE
#define PE6_PWM     NO_PWM_PIN
#define PE6_TCCR    NO_TCCR_PIN
#define PE6_COM   0
#define PE6_DDR        DDRE
#undef PE7
#define PE7_PIN        PINE7
#define PE7_RPORT      PINE
#define PE7_WPORT      PORTE
#define PE7_PWM     NO_PWM_PIN
#define PE7_TCCR    NO_TCCR_PIN
#define PE7_COM   0
#define PE7_DDR        DDRE

#undef PF0
#define PF0_PIN        PINF0
#define PF0_RPORT      PINF
#define PF0_WPORT      PORTF
#define PF0_PWM     NO_PWM_PIN
#define PF0_TCCR    NO_TCCR_PIN
#define PF0_COM   0
#define PF0_DDR        DDRF
#undef PF1
#define PF1_PIN        PINF1
#define PF1_RPORT      PINF
#define PF1_WPORT      PORTF
#define PF1_PWM     NO_PWM_PIN
#define PF1_TCCR    NO_TCCR_PIN
#define PF1_COM   0
#define PF1_DDR        DDRF
#undef PF2
#define PF2_PIN        PINF2
#define PF2_RPORT      PINF
#define PF2_WPORT      PORTF
#define PF2_PWM     NO_PWM_PIN
#define PF2_TCCR    NO_TCCR_PIN
#define PF2_COM   0
#define PF2_DDR        DDRF
#undef PF3
#define PF3_PIN        PINF3
#define PF3_RPORT      PINF
#define PF3_WPORT      PORTF
#define PF3_PWM     NO_PWM_PIN
#define PF3_TCCR    NO_TCCR_PIN
#define PF3_COM   0
#define PF3_DDR        DDRF
#undef PF4
#define PF4_PIN        PINF4
#define PF4_RPORT      PINF
#define PF4_WPORT      PORTF
#define PF4_PWM     NO_PWM_PIN
#define PF4_TCCR    NO_TCCR_PIN
#define PF4_COM   0
#define PF4_DDR        DDRF
#undef PF5
#define PF5_PIN        PINF5
#define PF5_RPORT      PINF
#define PF5_WPORT      PORTF
#define PF5_PWM     NO_PWM_PIN
#define PF5_TCCR    NO_TCCR_PIN
#define PF5_COM   0
#define PF5_DDR        DDRF
#undef PF6
#define PF6_PIN        PINF6
#define PF6_RPORT      PINF
#define PF6_WPORT      PORTF
#define PF6_PWM     NO_PWM_PIN
#define PF6_TCCR    NO_TCCR_PIN
#define PF6_COM   0
#define PF6_DDR        DDRF
#undef PF7
#define PF7_PIN        PINF7
#define PF7_RPORT      PINF
#define PF7_WPORT      PORTF
#define PF7_PWM     NO_PWM_PIN
#define PF7_TCCR    NO_TCCR_PIN
#define PF7_COM   0
#define PF7_DDR        DDRF
