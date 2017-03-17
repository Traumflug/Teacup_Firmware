#define NO_PWM_PIN      (uint8_t *)0
#define NO_TCCR_PIN     *(uint8_t *)0

// UART
#define	RXD					DIO0
#define	TXD					DIO1

// SPI
#define	SCK					DIO52
#define	MISO				DIO50
#define	MOSI				DIO51
#define	SS					DIO53

// TWI (I2C)
#define	SCL					DIO21
#define	SDA					DIO20

// timers and PWM
#define	OC0A				DIO13
#define	OC0B				DIO4
#define	OC1A				DIO11
#define	OC1B				DIO12
#define	OC2A				DIO10
#define	OC2B				DIO9
#define	OC3A				DIO5
#define	OC3B				DIO2
#define	OC3C				DIO3
#define	OC4A				DIO6
#define	OC4B				DIO7
#define	OC4C				DIO8
#define	OC5A				DIO46
#define	OC5B				DIO45
#define	OC5C				DIO44


// digital pins
#define	DIO0_PIN		PINE0
#define	DIO0_RPORT	PINE
#define	DIO0_WPORT	PORTE
#define	DIO0_DDR		DDRE
#define DIO0_PWM     NO_PWM_PIN
#define DIO0_TCCR    NO_TCCR_PIN
#define DIO0_COM		0

#define	DIO1_PIN		PINE1
#define	DIO1_RPORT	PINE
#define	DIO1_WPORT	PORTE
#define	DIO1_DDR		DDRE
#define DIO1_PWM     NO_PWM_PIN
#define DIO1_TCCR    NO_TCCR_PIN
#define DIO1_COM		0

#define	DIO2_PIN		PINE4
#define	DIO2_RPORT	PINE
#define	DIO2_WPORT	PORTE
#define	DIO2_DDR		DDRE
#define DIO2_PWM		&OCR3BL
#define DIO2_TCCR   TCCR3A
#define DIO2_COM    COM3B1

#define	DIO3_PIN		PINE5
#define	DIO3_RPORT	PINE
#define	DIO3_WPORT	PORTE
#define	DIO3_DDR		DDRE
#define DIO3_PWM		&OCR3CL
#define DIO3_TCCR   TCCR3A
#define DIO3_COM    COM3C1

#define	DIO4_PIN		PING5
#define	DIO4_RPORT	PING
#define	DIO4_WPORT	PORTG
#define	DIO4_DDR		DDRG
#define DIO4_PWM		&OCR0B
#define DIO4_TCCR   TCCR0A
#define DIO4_COM    COM0B1

#define	DIO5_PIN		PINE3
#define	DIO5_RPORT	PINE
#define	DIO5_WPORT	PORTE
#define	DIO5_DDR		DDRE
#define DIO5_PWM		&OCR3AL
#define DIO5_TCCR   TCCR3A
#define DIO5_COM    COM3A1

#define	DIO6_PIN		PINH3
#define	DIO6_RPORT	PINH
#define	DIO6_WPORT	PORTH
#define	DIO6_DDR		DDRH
#define DIO6_PWM		&OCR4AL
#define DIO6_TCCR   TCCR4A
#define DIO6_COM    COM4A1

#define	DIO7_PIN		PINH4
#define	DIO7_RPORT	PINH
#define	DIO7_WPORT	PORTH
#define	DIO7_DDR		DDRH
#define DIO7_PWM		&OCR4BL
#define DIO7_TCCR   TCCR4A
#define DIO7_COM    COM4B1

#define	DIO8_PIN		PINH5
#define	DIO8_RPORT	PINH
#define	DIO8_WPORT	PORTH
#define	DIO8_DDR		DDRH
#define DIO8_PWM		&OCR4CL
#define DIO8_TCCR   TCCR4A
#define DIO8_COM    COM4C1

#define	DIO9_PIN		PINH6
#define	DIO9_RPORT	PINH
#define	DIO9_WPORT	PORTH
#define	DIO9_DDR		DDRH
#define DIO9_PWM		&OCR2B
#define DIO9_TCCR   TCCR2A
#define DIO9_COM    COM2B1

#define	DIO10_PIN		PINB4
#define	DIO10_RPORT	PINB
#define	DIO10_WPORT	PORTB
#define	DIO10_DDR		DDRB
#define DIO10_PWM		&OCR2A
#define DIO10_TCCR   TCCR2A
#define DIO10_COM    COM2A1

#define	DIO11_PIN		PINB5
#define	DIO11_RPORT	PINB
#define	DIO11_WPORT	PORTB
#define	DIO11_DDR		DDRB
#define DIO11_PWM     NO_PWM_PIN
#define DIO11_TCCR    NO_TCCR_PIN
#define DIO11_COM		0

#define	DIO12_PIN		PINB6
#define	DIO12_RPORT	PINB
#define	DIO12_WPORT	PORTB
#define	DIO12_DDR		DDRB
#define DIO12_PWM     NO_PWM_PIN
#define DIO12_TCCR    NO_TCCR_PIN
#define DIO12_COM		0

#define	DIO13_PIN		PINB7
#define	DIO13_RPORT	PINB
#define	DIO13_WPORT	PORTB
#define	DIO13_DDR		DDRB
#define DIO13_PWM		&OCR0A
#define DIO13_TCCR   TCCR0A
#define DIO13_COM    COM0A1

#define	DIO14_PIN		PINJ1
#define	DIO14_RPORT	PINJ
#define	DIO14_WPORT	PORTJ
#define	DIO14_DDR		DDRJ
#define DIO14_PWM     NO_PWM_PIN
#define DIO14_TCCR    NO_TCCR_PIN
#define DIO14_COM		0

#define	DIO15_PIN		PINJ0
#define	DIO15_RPORT	PINJ
#define	DIO15_WPORT	PORTJ
#define	DIO15_DDR		DDRJ
#define DIO15_PWM     NO_PWM_PIN
#define DIO15_TCCR    NO_TCCR_PIN
#define DIO15_COM		0

#define	DIO16_PIN		PINH1
#define	DIO16_RPORT	PINH
#define	DIO16_WPORT	PORTH
#define	DIO16_DDR		DDRH
#define DIO16_PWM     NO_PWM_PIN
#define DIO16_TCCR    NO_TCCR_PIN
#define DIO16_COM		0

#define	DIO17_PIN		PINH0
#define	DIO17_RPORT	PINH
#define	DIO17_WPORT	PORTH
#define	DIO17_DDR		DDRH
#define DIO17_PWM     NO_PWM_PIN
#define DIO17_TCCR    NO_TCCR_PIN
#define DIO17_COM		0

#define	DIO18_PIN		PIND3
#define	DIO18_RPORT	PIND
#define	DIO18_WPORT	PORTD
#define	DIO18_DDR		DDRD
#define DIO18_PWM     NO_PWM_PIN
#define DIO18_TCCR    NO_TCCR_PIN
#define DIO18_COM		0

#define	DIO19_PIN		PIND2
#define	DIO19_RPORT	PIND
#define	DIO19_WPORT	PORTD
#define	DIO19_DDR		DDRD
#define DIO19_PWM     NO_PWM_PIN
#define DIO19_TCCR    NO_TCCR_PIN
#define DIO19_COM		0

#define	DIO20_PIN		PIND1
#define	DIO20_RPORT	PIND
#define	DIO20_WPORT	PORTD
#define	DIO20_DDR		DDRD
#define DIO20_PWM     NO_PWM_PIN
#define DIO20_TCCR    NO_TCCR_PIN
#define DIO20_COM		0

#define	DIO21_PIN		PIND0
#define	DIO21_RPORT	PIND
#define	DIO21_WPORT	PORTD
#define	DIO21_DDR		DDRD
#define DIO21_PWM     NO_PWM_PIN
#define DIO21_TCCR    NO_TCCR_PIN
#define DIO21_COM		0

#define	DIO22_PIN		PINA0
#define	DIO22_RPORT	PINA
#define	DIO22_WPORT	PORTA
#define	DIO22_DDR		DDRA
#define DIO22_PWM     NO_PWM_PIN
#define DIO22_TCCR    NO_TCCR_PIN
#define DIO22_COM		0

#define	DIO23_PIN		PINA1
#define	DIO23_RPORT	PINA
#define	DIO23_WPORT	PORTA
#define	DIO23_DDR		DDRA
#define DIO23_PWM     NO_PWM_PIN
#define DIO23_TCCR    NO_TCCR_PIN
#define DIO23_COM		0

#define	DIO24_PIN		PINA2
#define	DIO24_RPORT	PINA
#define	DIO24_WPORT	PORTA
#define	DIO24_DDR		DDRA
#define DIO24_PWM     NO_PWM_PIN
#define DIO24_TCCR    NO_TCCR_PIN
#define DIO24_COM		0

#define	DIO25_PIN		PINA3
#define	DIO25_RPORT	PINA
#define	DIO25_WPORT	PORTA
#define	DIO25_DDR		DDRA
#define DIO25_PWM     NO_PWM_PIN
#define DIO25_TCCR    NO_TCCR_PIN
#define DIO25_COM		0

#define	DIO26_PIN		PINA4
#define	DIO26_RPORT	PINA
#define	DIO26_WPORT	PORTA
#define	DIO26_DDR		DDRA
#define DIO26_PWM     NO_PWM_PIN
#define DIO26_TCCR    NO_TCCR_PIN
#define DIO26_COM		0

#define	DIO27_PIN		PINA5
#define	DIO27_RPORT	PINA
#define	DIO27_WPORT	PORTA
#define	DIO27_DDR		DDRA
#define DIO27_PWM     NO_PWM_PIN
#define DIO27_TCCR    NO_TCCR_PIN
#define DIO27_COM		0

#define	DIO28_PIN		PINA6
#define	DIO28_RPORT	PINA
#define	DIO28_WPORT	PORTA
#define	DIO28_DDR		DDRA
#define DIO28_PWM     NO_PWM_PIN
#define DIO28_TCCR    NO_TCCR_PIN
#define DIO28_COM		0

#define	DIO29_PIN		PINA7
#define	DIO29_RPORT	PINA
#define	DIO29_WPORT	PORTA
#define	DIO29_DDR		DDRA
#define DIO29_PWM     NO_PWM_PIN
#define DIO29_TCCR    NO_TCCR_PIN
#define DIO29_COM		0

#define	DIO30_PIN		PINC7
#define	DIO30_RPORT	PINC
#define	DIO30_WPORT	PORTC
#define	DIO30_DDR		DDRC
#define DIO30_PWM     NO_PWM_PIN
#define DIO30_TCCR    NO_TCCR_PIN
#define DIO30_COM		0

#define	DIO31_PIN		PINC6
#define	DIO31_RPORT	PINC
#define	DIO31_WPORT	PORTC
#define	DIO31_DDR		DDRC
#define DIO31_PWM     NO_PWM_PIN
#define DIO31_TCCR    NO_TCCR_PIN
#define DIO31_COM		0

#define	DIO32_PIN		PINC5
#define	DIO32_RPORT	PINC
#define	DIO32_WPORT	PORTC
#define	DIO32_DDR		DDRC
#define DIO32_PWM     NO_PWM_PIN
#define DIO32_TCCR    NO_TCCR_PIN
#define DIO32_COM		0

#define	DIO33_PIN		PINC4
#define	DIO33_RPORT	PINC
#define	DIO33_WPORT	PORTC
#define	DIO33_DDR		DDRC
#define DIO33_PWM     NO_PWM_PIN
#define DIO33_TCCR    NO_TCCR_PIN
#define DIO33_COM		0

#define	DIO34_PIN		PINC3
#define	DIO34_RPORT	PINC
#define	DIO34_WPORT	PORTC
#define	DIO34_DDR		DDRC
#define DIO34_PWM     NO_PWM_PIN
#define DIO34_TCCR    NO_TCCR_PIN
#define DIO34_COM		0

#define	DIO35_PIN		PINC2
#define	DIO35_RPORT	PINC
#define	DIO35_WPORT	PORTC
#define	DIO35_DDR		DDRC
#define DIO35_PWM     NO_PWM_PIN
#define DIO35_TCCR    NO_TCCR_PIN
#define DIO35_COM		0

#define	DIO36_PIN		PINC1
#define	DIO36_RPORT	PINC
#define	DIO36_WPORT	PORTC
#define	DIO36_DDR		DDRC
#define DIO36_PWM     NO_PWM_PIN
#define DIO36_TCCR    NO_TCCR_PIN
#define DIO36_COM		0

#define	DIO37_PIN		PINC0
#define	DIO37_RPORT	PINC
#define	DIO37_WPORT	PORTC
#define	DIO37_DDR		DDRC
#define DIO37_PWM     NO_PWM_PIN
#define DIO37_TCCR    NO_TCCR_PIN
#define DIO37_COM		0

#define	DIO38_PIN		PIND7
#define	DIO38_RPORT	PIND
#define	DIO38_WPORT	PORTD
#define	DIO38_DDR		DDRD
#define DIO38_PWM     NO_PWM_PIN
#define DIO38_TCCR    NO_TCCR_PIN
#define DIO38_COM		0

#define	DIO39_PIN		PING2
#define	DIO39_RPORT	PING
#define	DIO39_WPORT	PORTG
#define	DIO39_DDR		DDRG
#define DIO39_PWM     NO_PWM_PIN
#define DIO39_TCCR    NO_TCCR_PIN
#define DIO39_COM		0

#define	DIO40_PIN		PING1
#define	DIO40_RPORT	PING
#define	DIO40_WPORT	PORTG
#define	DIO40_DDR		DDRG
#define DIO40_PWM     NO_PWM_PIN
#define DIO40_TCCR    NO_TCCR_PIN
#define DIO40_COM		0

#define	DIO41_PIN		PING0
#define	DIO41_RPORT	PING
#define	DIO41_WPORT	PORTG
#define	DIO41_DDR		DDRG
#define DIO41_PWM     NO_PWM_PIN
#define DIO41_TCCR    NO_TCCR_PIN
#define DIO41_COM		0

#define	DIO42_PIN		PINL7
#define	DIO42_RPORT	PINL
#define	DIO42_WPORT	PORTL
#define	DIO42_DDR		DDRL
#define DIO42_PWM     NO_PWM_PIN
#define DIO42_TCCR    NO_TCCR_PIN
#define DIO42_COM		0

#define	DIO43_PIN		PINL6
#define	DIO43_RPORT	PINL
#define	DIO43_WPORT	PORTL
#define	DIO43_DDR		DDRL
#define DIO43_PWM     NO_PWM_PIN
#define DIO43_TCCR    NO_TCCR_PIN
#define DIO43_COM		0

#define	DIO44_PIN		PINL5
#define	DIO44_RPORT	PINL
#define	DIO44_WPORT	PORTL
#define	DIO44_DDR		DDRL
#define DIO44_PWM		&OCR5CL
#define DIO44_TCCR   TCCR5A
#define DIO44_COM    COM5C1

#define	DIO45_PIN		PINL4
#define	DIO45_RPORT	PINL
#define	DIO45_WPORT	PORTL
#define	DIO45_DDR		DDRL
#define DIO45_PWM		&OCR5BL
#define DIO45_TCCR   TCCR5A
#define DIO45_COM    COM5B1

#define	DIO46_PIN		PINL3
#define	DIO46_RPORT	PINL
#define	DIO46_WPORT	PORTL
#define	DIO46_DDR		DDRL
#define DIO46_PWM		&OCR5AL
#define DIO46_TCCR   TCCR5A
#define DIO46_COM    COM5A1

#define	DIO47_PIN		PINL2
#define	DIO47_RPORT	PINL
#define	DIO47_WPORT	PORTL
#define	DIO47_DDR		DDRL
#define DIO47_PWM     NO_PWM_PIN
#define DIO47_TCCR    NO_TCCR_PIN
#define DIO47_COM		0

#define	DIO48_PIN		PINL1
#define	DIO48_RPORT	PINL
#define	DIO48_WPORT	PORTL
#define	DIO48_DDR		DDRL
#define DIO48_PWM     NO_PWM_PIN
#define DIO48_TCCR    NO_TCCR_PIN
#define DIO48_COM		0

#define	DIO49_PIN		PINL0
#define	DIO49_RPORT	PINL
#define	DIO49_WPORT	PORTL
#define	DIO49_DDR		DDRL
#define DIO49_PWM     NO_PWM_PIN
#define DIO49_TCCR    NO_TCCR_PIN
#define DIO49_COM		0

#define	DIO50_PIN		PINB3
#define	DIO50_RPORT	PINB
#define	DIO50_WPORT	PORTB
#define	DIO50_DDR		DDRB
#define DIO50_PWM     NO_PWM_PIN
#define DIO50_TCCR    NO_TCCR_PIN
#define DIO50_COM		0

#define	DIO51_PIN		PINB2
#define	DIO51_RPORT	PINB
#define	DIO51_WPORT	PORTB
#define	DIO51_DDR		DDRB
#define DIO51_PWM     NO_PWM_PIN
#define DIO51_TCCR    NO_TCCR_PIN
#define DIO51_COM		0

#define	DIO52_PIN		PINB1
#define	DIO52_RPORT	PINB
#define	DIO52_WPORT	PORTB
#define	DIO52_DDR		DDRB
#define DIO52_PWM     NO_PWM_PIN
#define DIO52_TCCR    NO_TCCR_PIN
#define DIO52_COM		0

#define	DIO53_PIN		PINB0
#define	DIO53_RPORT	PINB
#define	DIO53_WPORT	PORTB
#define	DIO53_DDR		DDRB
#define DIO53_PWM     NO_PWM_PIN
#define DIO53_TCCR    NO_TCCR_PIN
#define DIO53_COM		0

/**
  DIO54 ... DIO69 are added for compatibility with other
  firmwares and duplicate names for AIO0 ... AIO15,
  so DIO54 == AIO0, DIO55 == AIO1, DIO56 == AIO2, ...
*/
#define DIO54_PIN   PINF0
#define DIO54_RPORT PINF
#define DIO54_WPORT PORTF
#define DIO54_DDR   DDRF
#define DIO54_PWM     NO_PWM_PIN
#define DIO54_TCCR    NO_TCCR_PIN
#define DIO54_COM		0

#define DIO55_PIN   PINF1
#define DIO55_RPORT PINF
#define DIO55_WPORT PORTF
#define DIO55_DDR   DDRF
#define DIO55_PWM     NO_PWM_PIN
#define DIO55_TCCR    NO_TCCR_PIN
#define DIO55_COM		0

#define DIO56_PIN   PINF2
#define DIO56_RPORT PINF
#define DIO56_WPORT PORTF
#define DIO56_DDR   DDRF
#define DIO56_PWM     NO_PWM_PIN
#define DIO56_TCCR    NO_TCCR_PIN
#define DIO56_COM		0

#define DIO57_PIN   PINF3
#define DIO57_RPORT PINF
#define DIO57_WPORT PORTF
#define DIO57_DDR   DDRF
#define DIO57_PWM     NO_PWM_PIN
#define DIO57_TCCR    NO_TCCR_PIN
#define DIO57_COM		0

#define DIO58_PIN   PINF4
#define DIO58_RPORT PINF
#define DIO58_WPORT PORTF
#define DIO58_DDR   DDRF
#define DIO58_PWM     NO_PWM_PIN
#define DIO58_TCCR    NO_TCCR_PIN
#define DIO58_COM		0

#define DIO59_PIN   PINF5
#define DIO59_RPORT PINF
#define DIO59_WPORT PORTF
#define DIO59_DDR   DDRF
#define DIO59_PWM     NO_PWM_PIN
#define DIO59_TCCR    NO_TCCR_PIN
#define DIO59_COM		0

#define DIO60_PIN   PINF6
#define DIO60_RPORT PINF
#define DIO60_WPORT PORTF
#define DIO60_DDR   DDRF
#define DIO60_PWM     NO_PWM_PIN
#define DIO60_TCCR    NO_TCCR_PIN
#define DIO60_COM		0

#define DIO61_PIN   PINF7
#define DIO61_RPORT PINF
#define DIO61_WPORT PORTF
#define DIO61_DDR   DDRF
#define DIO61_PWM     NO_PWM_PIN
#define DIO61_TCCR    NO_TCCR_PIN
#define DIO61_COM		0

#define DIO62_PIN   PINK0
#define DIO62_RPORT PINK
#define DIO62_WPORT PORTK
#define DIO62_DDR   DDRK
#define DIO62_PWM     NO_PWM_PIN
#define DIO62_TCCR    NO_TCCR_PIN
#define DIO62_COM		0

#define DIO63_PIN   PINK1
#define DIO63_RPORT PINK
#define DIO63_WPORT PORTK
#define DIO63_DDR   DDRK
#define DIO63_PWM     NO_PWM_PIN
#define DIO63_TCCR    NO_TCCR_PIN
#define DIO63_COM		0

#define DIO64_PIN   PINK2
#define DIO64_RPORT PINK
#define DIO64_WPORT PORTK
#define DIO64_DDR   DDRK
#define DIO64_PWM     NO_PWM_PIN
#define DIO64_TCCR    NO_TCCR_PIN
#define DIO64_COM		0

#define DIO65_PIN   PINK3
#define DIO65_RPORT PINK
#define DIO65_WPORT PORTK
#define DIO65_DDR   DDRK
#define DIO65_PWM     NO_PWM_PIN
#define DIO65_TCCR    NO_TCCR_PIN
#define DIO65_COM		0

#define DIO66_PIN   PINK4
#define DIO66_RPORT PINK
#define DIO66_WPORT PORTK
#define DIO66_DDR   DDRK
#define DIO66_PWM     NO_PWM_PIN
#define DIO66_TCCR    NO_TCCR_PIN
#define DIO66_COM		0

#define DIO67_PIN   PINK5
#define DIO67_RPORT PINK
#define DIO67_WPORT PORTK
#define DIO67_DDR   DDRK
#define DIO67_PWM     NO_PWM_PIN
#define DIO67_TCCR    NO_TCCR_PIN
#define DIO67_COM		0

#define DIO68_PIN   PINK6
#define DIO68_RPORT PINK
#define DIO68_WPORT PORTK
#define DIO68_DDR   DDRK
#define DIO68_PWM     NO_PWM_PIN
#define DIO68_TCCR    NO_TCCR_PIN
#define DIO68_COM		0

#define DIO69_PIN   PINK7
#define DIO69_RPORT PINK
#define DIO69_WPORT PORTK
#define DIO69_DDR   DDRK
#define DIO69_PWM     NO_PWM_PIN
#define DIO69_TCCR    NO_TCCR_PIN
#define DIO69_COM		0


// analog pins
#define AIO0_PIN		PINF0
#define AIO0_RPORT	PINF
#define AIO0_WPORT	PORTF
#define AIO0_DDR		DDRF
#define AIO0_PWM     NO_PWM_PIN
#define AIO0_TCCR    NO_TCCR_PIN
#define AIO0_COM		0
#define AIO0_ADC		0

#define AIO1_PIN		PINF1
#define AIO1_RPORT	PINF
#define AIO1_WPORT	PORTF
#define AIO1_DDR		DDRF
#define AIO1_PWM     NO_PWM_PIN
#define AIO1_TCCR    NO_TCCR_PIN
#define AIO1_COM		0
#define AIO1_ADC		1

#define AIO2_PIN		PINF2
#define AIO2_RPORT	PINF
#define AIO2_WPORT	PORTF
#define AIO2_DDR		DDRF
#define AIO2_PWM     NO_PWM_PIN
#define AIO2_TCCR    NO_TCCR_PIN
#define AIO2_COM		0
#define AIO2_ADC		2

#define AIO3_PIN		PINF3
#define AIO3_RPORT	PINF
#define AIO3_WPORT	PORTF
#define AIO3_DDR		DDRF
#define AIO3_PWM     NO_PWM_PIN
#define AIO3_TCCR    NO_TCCR_PIN
#define AIO3_COM		0
#define AIO3_ADC		3

#define AIO4_PIN		PINF4
#define AIO4_RPORT	PINF
#define AIO4_WPORT	PORTF
#define AIO4_DDR		DDRF
#define AIO4_PWM     NO_PWM_PIN
#define AIO4_TCCR    NO_TCCR_PIN
#define AIO4_COM		0
#define AIO4_ADC		4

#define AIO5_PIN		PINF5
#define AIO5_RPORT	PINF
#define AIO5_WPORT	PORTF
#define AIO5_DDR		DDRF
#define AIO5_PWM     NO_PWM_PIN
#define AIO5_TCCR    NO_TCCR_PIN
#define AIO5_COM		0
#define AIO5_ADC		5

#define AIO6_PIN		PINF6
#define AIO6_RPORT	PINF
#define AIO6_WPORT	PORTF
#define AIO6_DDR		DDRF
#define AIO6_PWM     NO_PWM_PIN
#define AIO6_TCCR    NO_TCCR_PIN
#define AIO6_COM		0
#define AIO6_ADC		6

#define AIO7_PIN		PINF7
#define AIO7_RPORT	PINF
#define AIO7_WPORT	PORTF
#define AIO7_DDR		DDRF
#define AIO7_PWM     NO_PWM_PIN
#define AIO7_TCCR    NO_TCCR_PIN
#define AIO7_COM		0
#define AIO7_ADC		7

#define AIO8_PIN		PINK0
#define AIO8_RPORT	PINK
#define AIO8_WPORT	PORTK
#define AIO8_DDR		DDRK
#define AIO8_PWM     NO_PWM_PIN
#define AIO8_TCCR    NO_TCCR_PIN
#define AIO8_COM		0
#define AIO8_ADC		8

#define AIO9_PIN		PINK1
#define AIO9_RPORT	PINK
#define AIO9_WPORT	PORTK
#define AIO9_DDR		DDRK
#define AIO9_PWM     NO_PWM_PIN
#define AIO9_TCCR    NO_TCCR_PIN
#define AIO9_COM		0
#define AIO9_ADC		9

#define AIO10_PIN		PINK2
#define AIO10_RPORT	PINK
#define AIO10_WPORT	PORTK
#define AIO10_DDR		DDRK
#define AIO10_PWM     NO_PWM_PIN
#define AIO10_TCCR    NO_TCCR_PIN
#define AIO10_COM		0
#define AIO10_ADC		10

#define AIO11_PIN		PINK3
#define AIO11_RPORT	PINK
#define AIO11_WPORT	PORTK
#define AIO11_DDR		DDRK
#define AIO11_PWM     NO_PWM_PIN
#define AIO11_TCCR    NO_TCCR_PIN
#define AIO11_COM		0
#define AIO11_ADC		11

#define AIO12_PIN		PINK4
#define AIO12_RPORT	PINK
#define AIO12_WPORT	PORTK
#define AIO12_DDR		DDRK
#define AIO12_PWM     NO_PWM_PIN
#define AIO12_TCCR    NO_TCCR_PIN
#define AIO12_COM		0
#define AIO12_ADC		12

#define AIO13_PIN		PINK5
#define AIO13_RPORT	PINK
#define AIO13_WPORT	PORTK
#define AIO13_DDR		DDRK
#define AIO13_PWM     NO_PWM_PIN
#define AIO13_TCCR    NO_TCCR_PIN
#define AIO13_COM		0
#define AIO13_ADC		13

#define AIO14_PIN		PINK6
#define AIO14_RPORT	PINK
#define AIO14_WPORT	PORTK
#define AIO14_DDR		DDRK
#define AIO14_PWM     NO_PWM_PIN
#define AIO14_TCCR    NO_TCCR_PIN
#define AIO14_COM		0
#define AIO14_ADC		14

#define AIO15_PIN		PINK7
#define AIO15_RPORT	PINK
#define AIO15_WPORT	PORTK
#define AIO15_DDR		DDRK
#define AIO15_PWM     NO_PWM_PIN
#define AIO15_TCCR    NO_TCCR_PIN
#define AIO15_COM		0
#define AIO15_ADC		15



#undef PA0
#define PA0_PIN			PINA0
#define PA0_RPORT		PINA
#define PA0_WPORT		PORTA
#define PA0_DDR			DDRA
#define PA0_PWM     NO_PWM_PIN
#define PA0_TCCR    NO_TCCR_PIN
#define PA0_COM		0
#undef PA1
#define PA1_PIN			PINA1
#define PA1_RPORT		PINA
#define PA1_WPORT		PORTA
#define PA1_DDR			DDRA
#define PA1_PWM     NO_PWM_PIN
#define PA1_TCCR    NO_TCCR_PIN
#define PA1_COM		0
#undef PA2
#define PA2_PIN			PINA2
#define PA2_RPORT		PINA
#define PA2_WPORT		PORTA
#define PA2_DDR			DDRA
#define PA2_PWM     NO_PWM_PIN
#define PA2_TCCR    NO_TCCR_PIN
#define PA2_COM		0
#undef PA3
#define PA3_PIN			PINA3
#define PA3_RPORT		PINA
#define PA3_WPORT		PORTA
#define PA3_DDR			DDRA
#define PA3_PWM     NO_PWM_PIN
#define PA3_TCCR    NO_TCCR_PIN
#define PA3_COM		0
#undef PA4
#define PA4_PIN			PINA4
#define PA4_RPORT		PINA
#define PA4_WPORT		PORTA
#define PA4_DDR			DDRA
#define PA4_PWM     NO_PWM_PIN
#define PA4_TCCR    NO_TCCR_PIN
#define PA4_COM		0
#undef PA5
#define PA5_PIN			PINA5
#define PA5_RPORT		PINA
#define PA5_WPORT		PORTA
#define PA5_DDR			DDRA
#define PA5_PWM     NO_PWM_PIN
#define PA5_TCCR    NO_TCCR_PIN
#define PA5_COM		0
#undef PA6
#define PA6_PIN			PINA6
#define PA6_RPORT		PINA
#define PA6_WPORT		PORTA
#define PA6_DDR			DDRA
#define PA6_PWM     NO_PWM_PIN
#define PA6_TCCR    NO_TCCR_PIN
#define PA6_COM		0
#undef PA7
#define PA7_PIN			PINA7
#define PA7_RPORT		PINA
#define PA7_WPORT		PORTA
#define PA7_DDR			DDRA
#define PA7_PWM     NO_PWM_PIN
#define PA7_TCCR    NO_TCCR_PIN
#define PA7_COM		0

#undef PB0
#define PB0_PIN			PINB0
#define PB0_RPORT		PINB
#define PB0_WPORT		PORTB
#define PB0_DDR			DDRB
#define PB0_PWM     NO_PWM_PIN
#define PB0_TCCR    NO_TCCR_PIN
#define PB0_COM		0
#undef PB1
#define PB1_PIN			PINB1
#define PB1_RPORT		PINB
#define PB1_WPORT		PORTB
#define PB1_DDR			DDRB
#define PB1_PWM     NO_PWM_PIN
#define PB1_TCCR    NO_TCCR_PIN
#define PB1_COM		0
#undef PB2
#define PB2_PIN			PINB2
#define PB2_RPORT		PINB
#define PB2_WPORT		PORTB
#define PB2_DDR			DDRB
#define PB2_PWM     NO_PWM_PIN
#define PB2_TCCR    NO_TCCR_PIN
#define PB2_COM		0
#undef PB3
#define PB3_PIN			PINB3
#define PB3_RPORT		PINB
#define PB3_WPORT		PORTB
#define PB3_DDR			DDRB
#define PB3_PWM     NO_PWM_PIN
#define PB3_TCCR    NO_TCCR_PIN
#define PB3_COM		0
#undef PB4
#define PB4_PIN			PINB4
#define PB4_RPORT		PINB
#define PB4_WPORT		PORTB
#define PB4_DDR			DDRB
#define PB4_PWM			&OCR2A
#define PB4_TCCR   TCCR2A
#define PB4_COM    COM2A1
#undef PB5
#define PB5_PIN			PINB5
#define PB5_RPORT		PINB
#define PB5_WPORT		PORTB
#define PB5_DDR			DDRB
#define PB5_PWM     NO_PWM_PIN
#define PB5_TCCR    NO_TCCR_PIN
#define PB5_COM		0
#undef PB6
#define PB6_PIN			PINB6
#define PB6_RPORT		PINB
#define PB6_WPORT		PORTB
#define PB6_DDR			DDRB
#define PB6_PWM     NO_PWM_PIN
#define PB6_TCCR    NO_TCCR_PIN
#define PB6_COM		0
#undef PB7
#define PB7_PIN			PINB7
#define PB7_RPORT		PINB
#define PB7_WPORT		PORTB
#define PB7_DDR			DDRB
#define PB7_PWM			&OCR0A
#define PB7_TCCR   TCCR0A
#define PB7_COM    COM0A1

#undef PC0
#define PC0_PIN			PINC0
#define PC0_RPORT		PINC
#define PC0_WPORT		PORTC
#define PC0_DDR			DDRC
#define PC0_PWM     NO_PWM_PIN
#define PC0_TCCR    NO_TCCR_PIN
#define PC0_COM		0
#undef PC1
#define PC1_PIN			PINC1
#define PC1_RPORT		PINC
#define PC1_WPORT		PORTC
#define PC1_DDR			DDRC
#define PC1_PWM     NO_PWM_PIN
#define PC1_TCCR    NO_TCCR_PIN
#define PC1_COM		0
#undef PC2
#define PC2_PIN			PINC2
#define PC2_RPORT		PINC
#define PC2_WPORT		PORTC
#define PC2_DDR			DDRC
#define PC2_PWM     NO_PWM_PIN
#define PC2_TCCR    NO_TCCR_PIN
#define PC2_COM		0
#undef PC3
#define PC3_PIN			PINC3
#define PC3_RPORT		PINC
#define PC3_WPORT		PORTC
#define PC3_DDR			DDRC
#define PC3_PWM     NO_PWM_PIN
#define PC3_TCCR    NO_TCCR_PIN
#define PC3_COM		0
#undef PC4
#define PC4_PIN			PINC4
#define PC4_RPORT		PINC
#define PC4_WPORT		PORTC
#define PC4_DDR			DDRC
#define PC4_PWM     NO_PWM_PIN
#define PC4_TCCR    NO_TCCR_PIN
#define PC4_COM		0
#undef PC5
#define PC5_PIN			PINC5
#define PC5_RPORT		PINC
#define PC5_WPORT		PORTC
#define PC5_DDR			DDRC
#define PC5_PWM     NO_PWM_PIN
#define PC5_TCCR    NO_TCCR_PIN
#define PC5_COM		0
#undef PC6
#define PC6_PIN			PINC6
#define PC6_RPORT		PINC
#define PC6_WPORT		PORTC
#define PC6_DDR			DDRC
#define PC6_PWM     NO_PWM_PIN
#define PC6_TCCR    NO_TCCR_PIN
#define PC6_COM		0
#undef PC7
#define PC7_PIN			PINC7
#define PC7_RPORT		PINC
#define PC7_WPORT		PORTC
#define PC7_DDR			DDRC
#define PC7_PWM     NO_PWM_PIN
#define PC7_TCCR    NO_TCCR_PIN
#define PC7_COM		0

#undef PD0
#define PD0_PIN			PIND0
#define PD0_RPORT		PIND
#define PD0_WPORT		PORTD
#define PD0_DDR			DDRD
#define PD0_PWM     NO_PWM_PIN
#define PD0_TCCR    NO_TCCR_PIN
#define PD0_COM		0
#undef PD1
#define PD1_PIN			PIND1
#define PD1_RPORT		PIND
#define PD1_WPORT		PORTD
#define PD1_DDR			DDRD
#define PD1_PWM     NO_PWM_PIN
#define PD1_TCCR    NO_TCCR_PIN
#define PD1_COM		0
#undef PD2
#define PD2_PIN			PIND2
#define PD2_RPORT		PIND
#define PD2_WPORT		PORTD
#define PD2_DDR			DDRD
#define PD2_PWM     NO_PWM_PIN
#define PD2_TCCR    NO_TCCR_PIN
#define PD2_COM		0
#undef PD3
#define PD3_PIN			PIND3
#define PD3_RPORT		PIND
#define PD3_WPORT		PORTD
#define PD3_DDR			DDRD
#define PD3_PWM     NO_PWM_PIN
#define PD3_TCCR    NO_TCCR_PIN
#define PD3_COM		0
#undef PD4
#define PD4_PIN			PIND4
#define PD4_RPORT		PIND
#define PD4_WPORT		PORTD
#define PD4_DDR			DDRD
#define PD4_PWM     NO_PWM_PIN
#define PD4_TCCR    NO_TCCR_PIN
#define PD4_COM		0
#undef PD5
#define PD5_PIN			PIND5
#define PD5_RPORT		PIND
#define PD5_WPORT		PORTD
#define PD5_DDR			DDRD
#define PD5_PWM     NO_PWM_PIN
#define PD5_TCCR    NO_TCCR_PIN
#define PD5_COM		0
#undef PD6
#define PD6_PIN			PIND6
#define PD6_RPORT		PIND
#define PD6_WPORT		PORTD
#define PD6_DDR			DDRD
#define PD6_PWM     NO_PWM_PIN
#define PD6_TCCR    NO_TCCR_PIN
#define PD6_COM		0
#undef PD7
#define PD7_PIN			PIND7
#define PD7_RPORT		PIND
#define PD7_WPORT		PORTD
#define PD7_DDR			DDRD
#define PD7_PWM     NO_PWM_PIN
#define PD7_TCCR    NO_TCCR_PIN
#define PD7_COM		0

#undef PE0
#define PE0_PIN			PINE0
#define PE0_RPORT		PINE
#define PE0_WPORT		PORTE
#define PE0_DDR			DDRE
#define PE0_PWM     NO_PWM_PIN
#define PE0_TCCR    NO_TCCR_PIN
#define PE0_COM		0
#undef PE1
#define PE1_PIN			PINE1
#define PE1_RPORT		PINE
#define PE1_WPORT		PORTE
#define PE1_DDR			DDRE
#define PE1_PWM     NO_PWM_PIN
#define PE1_TCCR    NO_TCCR_PIN
#define PE1_COM		0
#undef PE2
#define PE2_PIN			PINE2
#define PE2_RPORT		PINE
#define PE2_WPORT		PORTE
#define PE2_DDR			DDRE
#define PE2_PWM     NO_PWM_PIN
#define PE2_TCCR    NO_TCCR_PIN
#define PE2_COM		0
#undef PE3
#define PE3_PIN			PINE3
#define PE3_RPORT		PINE
#define PE3_WPORT		PORTE
#define PE3_DDR			DDRE
#define PE3_PWM			&OCR3AL
#define PE3_TCCR   TCCR3A
#define PE3_COM    COM3A1
#undef PE4
#define PE4_PIN			PINE4
#define PE4_RPORT		PINE
#define PE4_WPORT		PORTE
#define PE4_DDR			DDRE
#define PE4_PWM			&OCR3BL
#define PE4_TCCR   TCCR3A
#define PE4_COM    COM3B1
#undef PE5
#define PE5_PIN			PINE5
#define PE5_RPORT		PINE
#define PE5_WPORT		PORTE
#define PE5_DDR			DDRE
#define PE5_PWM			&OCR3CL
#define PE5_TCCR   TCCR3A
#define PE5_COM    COM3C1
#undef PE6
#define PE6_PIN			PINE6
#define PE6_RPORT		PINE
#define PE6_WPORT		PORTE
#define PE6_DDR			DDRE
#define PE6_PWM     NO_PWM_PIN
#define PE6_TCCR    NO_TCCR_PIN
#define PE6_COM		0
#undef PE7
#define PE7_PIN			PINE7
#define PE7_RPORT		PINE
#define PE7_WPORT		PORTE
#define PE7_DDR			DDRE
#define PE7_PWM     NO_PWM_PIN
#define PE7_TCCR    NO_TCCR_PIN
#define PE7_COM		0

#undef PF0
#define PF0_PIN			PINF0
#define PF0_RPORT		PINF
#define PF0_WPORT		PORTF
#define PF0_DDR			DDRF
#define PF0_PWM     NO_PWM_PIN
#define PF0_TCCR    NO_TCCR_PIN
#define PF0_COM		0
#undef PF1
#define PF1_PIN			PINF1
#define PF1_RPORT		PINF
#define PF1_WPORT		PORTF
#define PF1_DDR			DDRF
#define PF1_PWM     NO_PWM_PIN
#define PF1_TCCR    NO_TCCR_PIN
#define PF1_COM		0
#undef PF2
#define PF2_PIN			PINF2
#define PF2_RPORT		PINF
#define PF2_WPORT		PORTF
#define PF2_DDR			DDRF
#define PF2_PWM     NO_PWM_PIN
#define PF2_TCCR    NO_TCCR_PIN
#define PF2_COM		0
#undef PF3
#define PF3_PIN			PINF3
#define PF3_RPORT		PINF
#define PF3_WPORT		PORTF
#define PF3_DDR			DDRF
#define PF3_PWM     NO_PWM_PIN
#define PF3_TCCR    NO_TCCR_PIN
#define PF3_COM		0
#undef PF4
#define PF4_PIN			PINF4
#define PF4_RPORT		PINF
#define PF4_WPORT		PORTF
#define PF4_DDR			DDRF
#define PF4_PWM     NO_PWM_PIN
#define PF4_TCCR    NO_TCCR_PIN
#define PF4_COM		0
#undef PF5
#define PF5_PIN			PINF5
#define PF5_RPORT		PINF
#define PF5_WPORT		PORTF
#define PF5_DDR			DDRF
#define PF5_PWM     NO_PWM_PIN
#define PF5_TCCR    NO_TCCR_PIN
#define PF5_COM		0
#undef PF6
#define PF6_PIN			PINF6
#define PF6_RPORT		PINF
#define PF6_WPORT		PORTF
#define PF6_DDR			DDRF
#define PF6_PWM     NO_PWM_PIN
#define PF6_TCCR    NO_TCCR_PIN
#define PF6_COM		0
#undef PF7
#define PF7_PIN			PINF7
#define PF7_RPORT		PINF
#define PF7_WPORT		PORTF
#define PF7_DDR			DDRF
#define PF7_PWM     NO_PWM_PIN
#define PF7_TCCR    NO_TCCR_PIN
#define PF7_COM		0

#undef PG0
#define PG0_PIN			PING0
#define PG0_RPORT		PING
#define PG0_WPORT		PORTG
#define PG0_DDR			DDRG
#define PG0_PWM     NO_PWM_PIN
#define PG0_TCCR    NO_TCCR_PIN
#define PG0_COM		0
#undef PG1
#define PG1_PIN			PING1
#define PG1_RPORT		PING
#define PG1_WPORT		PORTG
#define PG1_DDR			DDRG
#define PG1_PWM     NO_PWM_PIN
#define PG1_TCCR    NO_TCCR_PIN
#define PG1_COM		0
#undef PG2
#define PG2_PIN			PING2
#define PG2_RPORT		PING
#define PG2_WPORT		PORTG
#define PG2_DDR			DDRG
#define PG2_PWM     NO_PWM_PIN
#define PG2_TCCR    NO_TCCR_PIN
#define PG2_COM		0
#undef PG3
#define PG3_PIN			PING3
#define PG3_RPORT		PING
#define PG3_WPORT		PORTG
#define PG3_DDR			DDRG
#define PG3_PWM     NO_PWM_PIN
#define PG3_TCCR    NO_TCCR_PIN
#define PG3_COM		0
#undef PG4
#define PG4_PIN			PING4
#define PG4_RPORT		PING
#define PG4_WPORT		PORTG
#define PG4_DDR			DDRG
#define PG4_PWM     NO_PWM_PIN
#define PG4_TCCR    NO_TCCR_PIN
#define PG4_COM		0
#undef PG5
#define PG5_PIN			PING5
#define PG5_RPORT		PING
#define PG5_WPORT		PORTG
#define PG5_DDR			DDRG
#define PG5_PWM			&OCR0B
#define PG5_TCCR   TCCR0A
#define PG5_COM    COM0B1
#undef PG6
#define PG6_PIN			PING6
#define PG6_RPORT		PING
#define PG6_WPORT		PORTG
#define PG6_DDR			DDRG
#define PG6_PWM     NO_PWM_PIN
#define PG6_TCCR    NO_TCCR_PIN
#define PG6_COM		0
#undef PG7
#define PG7_PIN			PING7
#define PG7_RPORT		PING
#define PG7_WPORT		PORTG
#define PG7_DDR			DDRG
#define PG7_PWM     NO_PWM_PIN
#define PG7_TCCR    NO_TCCR_PIN
#define PG7_COM		0

#undef PH0
#define PH0_PIN			PINH0
#define PH0_RPORT		PINH
#define PH0_WPORT		PORTH
#define PH0_DDR			DDRH
#define PH0_PWM     NO_PWM_PIN
#define PH0_TCCR    NO_TCCR_PIN
#define PH0_COM		0
#undef PH1
#define PH1_PIN			PINH1
#define PH1_RPORT		PINH
#define PH1_WPORT		PORTH
#define PH1_DDR			DDRH
#define PH1_PWM     NO_PWM_PIN
#define PH1_TCCR    NO_TCCR_PIN
#define PH1_COM		0
#undef PH2
#define PH2_PIN			PINH2
#define PH2_RPORT		PINH
#define PH2_WPORT		PORTH
#define PH2_DDR			DDRH
#define PH2_PWM     NO_PWM_PIN
#define PH2_TCCR    NO_TCCR_PIN
#define PH2_COM		0
#undef PH3
#define PH3_PIN			PINH3
#define PH3_RPORT		PINH
#define PH3_WPORT		PORTH
#define PH3_DDR			DDRH
#define PH3_PWM			&OCR4AL
#define PH3_TCCR   TCCR4A
#define PH3_COM    COM4A1
#undef PH4
#define PH4_PIN			PINH4
#define PH4_RPORT		PINH
#define PH4_WPORT		PORTH
#define PH4_DDR			DDRH
#define PH4_PWM			&OCR4BL
#define PH4_TCCR   TCCR4A
#define PH4_COM    COM4B1
#undef PH5
#define PH5_PIN			PINH5
#define PH5_RPORT		PINH
#define PH5_WPORT		PORTH
#define PH5_DDR			DDRH
#define PH5_PWM			&OCR4CL
#define PH5_TCCR   TCCR4A
#define PH5_COM    COM4C1
#undef PH6
#define PH6_PIN			PINH6
#define PH6_RPORT		PINH
#define PH6_WPORT		PORTH
#define PH6_DDR			DDRH
#define PH6_PWM			&OCR2B
#define PH6_TCCR   TCCR2A
#define PH6_COM    COM2B1
#undef PH7
#define PH7_PIN			PINH7
#define PH7_RPORT		PINH
#define PH7_WPORT		PORTH
#define PH7_DDR			DDRH
#define PH7_PWM     NO_PWM_PIN
#define PH7_TCCR    NO_TCCR_PIN
#define PH7_COM		0

#undef PJ0
#define PJ0_PIN			PINJ0
#define PJ0_RPORT		PINJ
#define PJ0_WPORT		PORTJ
#define PJ0_DDR			DDRJ
#define PJ0_PWM     NO_PWM_PIN
#define PJ0_TCCR    NO_TCCR_PIN
#define PJ0_COM		0
#undef PJ1
#define PJ1_PIN			PINJ1
#define PJ1_RPORT		PINJ
#define PJ1_WPORT		PORTJ
#define PJ1_DDR			DDRJ
#define PJ1_PWM     NO_PWM_PIN
#define PJ1_TCCR    NO_TCCR_PIN
#define PJ1_COM		0
#undef PJ2
#define PJ2_PIN			PINJ2
#define PJ2_RPORT		PINJ
#define PJ2_WPORT		PORTJ
#define PJ2_DDR			DDRJ
#define PJ2_PWM     NO_PWM_PIN
#define PJ2_TCCR    NO_TCCR_PIN
#define PJ2_COM		0
#undef PJ3
#define PJ3_PIN			PINJ3
#define PJ3_RPORT		PINJ
#define PJ3_WPORT		PORTJ
#define PJ3_DDR			DDRJ
#define PJ3_PWM     NO_PWM_PIN
#define PJ3_TCCR    NO_TCCR_PIN
#define PJ3_COM		0
#undef PJ4
#define PJ4_PIN			PINJ4
#define PJ4_RPORT		PINJ
#define PJ4_WPORT		PORTJ
#define PJ4_DDR			DDRJ
#define PJ4_PWM     NO_PWM_PIN
#define PJ4_TCCR    NO_TCCR_PIN
#define PJ4_COM		0
#undef PJ5
#define PJ5_PIN			PINJ5
#define PJ5_RPORT		PINJ
#define PJ5_WPORT		PORTJ
#define PJ5_DDR			DDRJ
#define PJ5_PWM     NO_PWM_PIN
#define PJ5_TCCR    NO_TCCR_PIN
#define PJ5_COM		0
#undef PJ6
#define PJ6_PIN			PINJ6
#define PJ6_RPORT		PINJ
#define PJ6_WPORT		PORTJ
#define PJ6_DDR			DDRJ
#define PJ6_PWM     NO_PWM_PIN
#define PJ6_TCCR    NO_TCCR_PIN
#define PJ6_COM		0
#undef PJ7
#define PJ7_PIN			PINJ7
#define PJ7_RPORT		PINJ
#define PJ7_WPORT		PORTJ
#define PJ7_DDR			DDRJ
#define PJ7_PWM     NO_PWM_PIN
#define PJ7_TCCR    NO_TCCR_PIN
#define PJ7_COM		0

#undef PK0
#define PK0_PIN			PINK0
#define PK0_RPORT		PINK
#define PK0_WPORT		PORTK
#define PK0_DDR			DDRK
#define PK0_PWM     NO_PWM_PIN
#define PK0_TCCR    NO_TCCR_PIN
#define PK0_COM		0
#undef PK1
#define PK1_PIN			PINK1
#define PK1_RPORT		PINK
#define PK1_WPORT		PORTK
#define PK1_DDR			DDRK
#define PK1_PWM     NO_PWM_PIN
#define PK1_TCCR    NO_TCCR_PIN
#define PK1_COM		0
#undef PK2
#define PK2_PIN			PINK2
#define PK2_RPORT		PINK
#define PK2_WPORT		PORTK
#define PK2_DDR			DDRK
#define PK2_PWM     NO_PWM_PIN
#define PK2_TCCR    NO_TCCR_PIN
#define PK2_COM		0
#undef PK3
#define PK3_PIN			PINK3
#define PK3_RPORT		PINK
#define PK3_WPORT		PORTK
#define PK3_DDR			DDRK
#define PK3_PWM     NO_PWM_PIN
#define PK3_TCCR    NO_TCCR_PIN
#define PK3_COM		0
#undef PK4
#define PK4_PIN			PINK4
#define PK4_RPORT		PINK
#define PK4_WPORT		PORTK
#define PK4_DDR			DDRK
#define PK4_PWM     NO_PWM_PIN
#define PK4_TCCR    NO_TCCR_PIN
#define PK4_COM		0
#undef PK5
#define PK5_PIN			PINK5
#define PK5_RPORT		PINK
#define PK5_WPORT		PORTK
#define PK5_DDR			DDRK
#define PK5_PWM     NO_PWM_PIN
#define PK5_TCCR    NO_TCCR_PIN
#define PK5_COM		0
#undef PK6
#define PK6_PIN			PINK6
#define PK6_RPORT		PINK
#define PK6_WPORT		PORTK
#define PK6_DDR			DDRK
#define PK6_PWM     NO_PWM_PIN
#define PK6_TCCR    NO_TCCR_PIN
#define PK6_COM		0
#undef PK7
#define PK7_PIN			PINK7
#define PK7_RPORT		PINK
#define PK7_WPORT		PORTK
#define PK7_DDR			DDRK
#define PK7_PWM     NO_PWM_PIN
#define PK7_TCCR    NO_TCCR_PIN
#define PK7_COM		0

#undef PL0
#define PL0_PIN			PINL0
#define PL0_RPORT		PINL
#define PL0_WPORT		PORTL
#define PL0_DDR			DDRL
#define PL0_PWM     NO_PWM_PIN
#define PL0_TCCR    NO_TCCR_PIN
#define PL0_COM		0
#undef PL1
#define PL1_PIN			PINL1
#define PL1_RPORT		PINL
#define PL1_WPORT		PORTL
#define PL1_DDR			DDRL
#define PL1_PWM     NO_PWM_PIN
#define PL1_TCCR    NO_TCCR_PIN
#define PL1_COM		0
#undef PL2
#define PL2_PIN			PINL2
#define PL2_RPORT		PINL
#define PL2_WPORT		PORTL
#define PL2_DDR			DDRL
#define PL2_PWM     NO_PWM_PIN
#define PL2_TCCR    NO_TCCR_PIN
#define PL2_COM		0
#undef PL3
#define PL3_PIN			PINL3
#define PL3_RPORT		PINL
#define PL3_WPORT		PORTL
#define PL3_DDR			DDRL
#define PL3_PWM			&OCR5AL
#define PL3_TCCR   TCCR5A
#define PL3_COM    COM5A1
#undef PL4
#define PL4_PIN			PINL4
#define PL4_RPORT		PINL
#define PL4_WPORT		PORTL
#define PL4_DDR			DDRL
#define PL4_PWM			&OCR5BL
#define PL4_TCCR   TCCR5A
#define PL4_COM    COM5B1
#undef PL5
#define PL5_PIN			PINL5
#define PL5_RPORT		PINL
#define PL5_WPORT		PORTL
#define PL5_DDR			DDRL
#define PL5_PWM			&OCR5CL
#define PL5_TCCR   TCCR5A
#define PL5_COM    COM5C1
#undef PL6
#define PL6_PIN			PINL6
#define PL6_RPORT		PINL
#define PL6_WPORT		PORTL
#define PL6_DDR			DDRL
#define PL6_PWM     NO_PWM_PIN
#define PL6_TCCR    NO_TCCR_PIN
#define PL6_COM		0
#undef PL7
#define PL7_PIN			PINL7
#define PL7_RPORT		PINL
#define PL7_WPORT		PORTL
#define PL7_DDR			DDRL
#define PL7_PWM     NO_PWM_PIN
#define PL7_TCCR    NO_TCCR_PIN
#define PL7_COM		0
