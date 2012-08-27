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

// change for your board
#define	DEBUG_LED		DIO21

/*
pins
*/
#define	DIO0_PIN		PINE0
#define	DIO0_RPORT	PINE
#define	DIO0_WPORT	PORTE
#define	DIO0_DDR		DDRE
#define DIO0_PWM		NULL

#define	DIO1_PIN		PINE1
#define	DIO1_RPORT	PINE
#define	DIO1_WPORT	PORTE
#define	DIO1_DDR		DDRE
#define DIO1_PWM		NULL

#define	DIO2_PIN		PINE4
#define	DIO2_RPORT	PINE
#define	DIO2_WPORT	PORTE
#define	DIO2_DDR		DDRE
#define DIO2_PWM		&OCR3BL

#define	DIO3_PIN		PINE5
#define	DIO3_RPORT	PINE
#define	DIO3_WPORT	PORTE
#define	DIO3_DDR		DDRE
#define DIO3_PWM		&OCR3CL

#define	DIO4_PIN		PING5
#define	DIO4_RPORT	PING
#define	DIO4_WPORT	PORTG
#define	DIO4_DDR		DDRG
#define DIO4_PWM		&OCR0B

#define	DIO5_PIN		PINE3
#define	DIO5_RPORT	PINE
#define	DIO5_WPORT	PORTE
#define	DIO5_DDR		DDRE
#define DIO5_PWM		&OCR3AL

#define	DIO6_PIN		PINH3
#define	DIO6_RPORT	PINH
#define	DIO6_WPORT	PORTH
#define	DIO6_DDR		DDRH
#define DIO6_PWM		&OCR4AL

#define	DIO7_PIN		PINH4
#define	DIO7_RPORT	PINH
#define	DIO7_WPORT	PORTH
#define	DIO7_DDR		DDRH
#define DIO7_PWM		&OCR4BL

#define	DIO8_PIN		PINH5
#define	DIO8_RPORT	PINH
#define	DIO8_WPORT	PORTH
#define	DIO8_DDR		DDRH
#define DIO8_PWM		&OCR4CL

#define	DIO9_PIN		PINH6
#define	DIO9_RPORT	PINH
#define	DIO9_WPORT	PORTH
#define	DIO9_DDR		DDRH
#define DIO9_PWM		&OCR2B

#define	DIO10_PIN		PINB4
#define	DIO10_RPORT	PINB
#define	DIO10_WPORT	PORTB
#define	DIO10_DDR		DDRB
#define DIO10_PWM		&OCR2A

#define	DIO11_PIN		PINB5
#define	DIO11_RPORT	PINB
#define	DIO11_WPORT	PORTB
#define	DIO11_DDR		DDRB
#define DIO11_PWM		NULL

#define	DIO12_PIN		PINB6
#define	DIO12_RPORT	PINB
#define	DIO12_WPORT	PORTB
#define	DIO12_DDR		DDRB
#define DIO12_PWM		NULL

#define	DIO13_PIN		PINB7
#define	DIO13_RPORT	PINB
#define	DIO13_WPORT	PORTB
#define	DIO13_DDR		DDRB
#define DIO13_PWM		&OCR0A

#define	DIO14_PIN		PINJ1
#define	DIO14_RPORT	PINJ
#define	DIO14_WPORT	PORTJ
#define	DIO14_DDR		DDRJ
#define DIO14_PWM		NULL

#define	DIO15_PIN		PINJ0
#define	DIO15_RPORT	PINJ
#define	DIO15_WPORT	PORTJ
#define	DIO15_DDR		DDRJ
#define DIO15_PWM		NULL

#define	DIO16_PIN		PINH1
#define	DIO16_RPORT	PINH
#define	DIO16_WPORT	PORTH
#define	DIO16_DDR		DDRH
#define DIO16_PWM		NULL

#define	DIO17_PIN		PINH0
#define	DIO17_RPORT	PINH
#define	DIO17_WPORT	PORTH
#define	DIO17_DDR		DDRH
#define DIO17_PWM		NULL

#define	DIO18_PIN		PIND3
#define	DIO18_RPORT	PIND
#define	DIO18_WPORT	PORTD
#define	DIO18_DDR		DDRD
#define DIO18_PWM		NULL

#define	DIO19_PIN		PIND2
#define	DIO19_RPORT	PIND
#define	DIO19_WPORT	PORTD
#define	DIO19_DDR		DDRD
#define DIO19_PWM		NULL

#define	DIO20_PIN		PIND1
#define	DIO20_RPORT	PIND
#define	DIO20_WPORT	PORTD
#define	DIO20_DDR		DDRD
#define DIO20_PWM		NULL

#define	DIO21_PIN		PIND0
#define	DIO21_RPORT	PIND
#define	DIO21_WPORT	PORTD
#define	DIO21_DDR		DDRD
#define DIO21_PWM		NULL

#define	DIO22_PIN		PINA0
#define	DIO22_RPORT	PINA
#define	DIO22_WPORT	PORTA
#define	DIO22_DDR		DDRA
#define DIO22_PWM		NULL

#define	DIO23_PIN		PINA1
#define	DIO23_RPORT	PINA
#define	DIO23_WPORT	PORTA
#define	DIO23_DDR		DDRA
#define DIO23_PWM		NULL

#define	DIO24_PIN		PINA2
#define	DIO24_RPORT	PINA
#define	DIO24_WPORT	PORTA
#define	DIO24_DDR		DDRA
#define DIO24_PWM		NULL

#define	DIO25_PIN		PINA3
#define	DIO25_RPORT	PINA
#define	DIO25_WPORT	PORTA
#define	DIO25_DDR		DDRA
#define DIO25_PWM		NULL

#define	DIO26_PIN		PINA4
#define	DIO26_RPORT	PINA
#define	DIO26_WPORT	PORTA
#define	DIO26_DDR		DDRA
#define DIO26_PWM		NULL

#define	DIO27_PIN		PINA5
#define	DIO27_RPORT	PINA
#define	DIO27_WPORT	PORTA
#define	DIO27_DDR		DDRA
#define DIO27_PWM		NULL

#define	DIO28_PIN		PINA6
#define	DIO28_RPORT	PINA
#define	DIO28_WPORT	PORTA
#define	DIO28_DDR		DDRA
#define DIO28_PWM		NULL

#define	DIO29_PIN		PINA7
#define	DIO29_RPORT	PINA
#define	DIO29_WPORT	PORTA
#define	DIO29_DDR		DDRA
#define DIO29_PWM		NULL

#define	DIO30_PIN		PINC7
#define	DIO30_RPORT	PINC
#define	DIO30_WPORT	PORTC
#define	DIO30_DDR		DDRC
#define DIO30_PWM		NULL

#define	DIO31_PIN		PINC6
#define	DIO31_RPORT	PINC
#define	DIO31_WPORT	PORTC
#define	DIO31_DDR		DDRC
#define DIO31_PWM		NULL

#define	DIO32_PIN		PINC5
#define	DIO32_RPORT	PINC
#define	DIO32_WPORT	PORTC
#define	DIO32_DDR		DDRC
#define DIO32_PWM		NULL

#define	DIO33_PIN		PINC4
#define	DIO33_RPORT	PINC
#define	DIO33_WPORT	PORTC
#define	DIO33_DDR		DDRC
#define DIO33_PWM		NULL

#define	DIO34_PIN		PINC3
#define	DIO34_RPORT	PINC
#define	DIO34_WPORT	PORTC
#define	DIO34_DDR		DDRC
#define DIO34_PWM		NULL

#define	DIO35_PIN		PINC2
#define	DIO35_RPORT	PINC
#define	DIO35_WPORT	PORTC
#define	DIO35_DDR		DDRC
#define DIO35_PWM		NULL

#define	DIO36_PIN		PINC1
#define	DIO36_RPORT	PINC
#define	DIO36_WPORT	PORTC
#define	DIO36_DDR		DDRC
#define DIO36_PWM		NULL

#define	DIO37_PIN		PINC0
#define	DIO37_RPORT	PINC
#define	DIO37_WPORT	PORTC
#define	DIO37_DDR		DDRC
#define DIO37_PWM		NULL

#define	DIO38_PIN		PIND7
#define	DIO38_RPORT	PIND
#define	DIO38_WPORT	PORTD
#define	DIO38_DDR		DDRD
#define DIO38_PWM		NULL

#define	DIO39_PIN		PING2
#define	DIO39_RPORT	PING
#define	DIO39_WPORT	PORTG
#define	DIO39_DDR		DDRG
#define DIO39_PWM		NULL

#define	DIO40_PIN		PING1
#define	DIO40_RPORT	PING
#define	DIO40_WPORT	PORTG
#define	DIO40_DDR		DDRG
#define DIO40_PWM		NULL

#define	DIO41_PIN		PING0
#define	DIO41_RPORT	PING
#define	DIO41_WPORT	PORTG
#define	DIO41_DDR		DDRG
#define DIO41_PWM		NULL

#define	DIO42_PIN		PINL7
#define	DIO42_RPORT	PINL
#define	DIO42_WPORT	PORTL
#define	DIO42_DDR		DDRL
#define DIO42_PWM		NULL

#define	DIO43_PIN		PINL6
#define	DIO43_RPORT	PINL
#define	DIO43_WPORT	PORTL
#define	DIO43_DDR		DDRL
#define DIO43_PWM		NULL

#define	DIO44_PIN		PINL5
#define	DIO44_RPORT	PINL
#define	DIO44_WPORT	PORTL
#define	DIO44_DDR		DDRL
#define DIO44_PWM		&OCR5CL

#define	DIO45_PIN		PINL4
#define	DIO45_RPORT	PINL
#define	DIO45_WPORT	PORTL
#define	DIO45_DDR		DDRL
#define DIO45_PWM		&OCR5BL

#define	DIO46_PIN		PINL3
#define	DIO46_RPORT	PINL
#define	DIO46_WPORT	PORTL
#define	DIO46_DDR		DDRL
#define DIO46_PWM		&OCR5AL

#define	DIO47_PIN		PINL2
#define	DIO47_RPORT	PINL
#define	DIO47_WPORT	PORTL
#define	DIO47_DDR		DDRL
#define DIO47_PWM		NULL

#define	DIO48_PIN		PINL1
#define	DIO48_RPORT	PINL
#define	DIO48_WPORT	PORTL
#define	DIO48_DDR		DDRL
#define DIO48_PWM		NULL

#define	DIO49_PIN		PINL0
#define	DIO49_RPORT	PINL
#define	DIO49_WPORT	PORTL
#define	DIO49_DDR		DDRL
#define DIO49_PWM		NULL

#define	DIO50_PIN		PINB3
#define	DIO50_RPORT	PINB
#define	DIO50_WPORT	PORTB
#define	DIO50_DDR		DDRB
#define DIO50_PWM		NULL

#define	DIO51_PIN		PINB2
#define	DIO51_RPORT	PINB
#define	DIO51_WPORT	PORTB
#define	DIO51_DDR		DDRB
#define DIO51_PWM		NULL

#define	DIO52_PIN		PINB1
#define	DIO52_RPORT	PINB
#define	DIO52_WPORT	PORTB
#define	DIO52_DDR		DDRB
#define DIO52_PWM		NULL

#define	DIO53_PIN		PINB0
#define	DIO53_RPORT	PINB
#define	DIO53_WPORT	PORTB
#define	DIO53_DDR		DDRB
#define DIO53_PWM		NULL

#define AIO0_PIN		PINF0
#define AIO0_RPORT	PINF
#define AIO0_WPORT	PORTF
#define AIO0_DDR		DDRF
#define AIO0_PWM		NULL
#define AIO0_ADC		0

#define AIO1_PIN		PINF1
#define AIO1_RPORT	PINF
#define AIO1_WPORT	PORTF
#define AIO1_DDR		DDRF
#define AIO1_PWM		NULL
#define AIO1_ADC		1

#define AIO2_PIN		PINF2
#define AIO2_RPORT	PINF
#define AIO2_WPORT	PORTF
#define AIO2_DDR		DDRF
#define AIO2_PWM		NULL
#define AIO2_ADC		2

#define AIO3_PIN		PINF3
#define AIO3_RPORT	PINF
#define AIO3_WPORT	PORTF
#define AIO3_DDR		DDRF
#define AIO3_PWM		NULL
#define AIO3_ADC		3

#define AIO4_PIN		PINF4
#define AIO4_RPORT	PINF
#define AIO4_WPORT	PORTF
#define AIO4_DDR		DDRF
#define AIO4_PWM		NULL
#define AIO4_ADC		4

#define AIO5_PIN		PINF5
#define AIO5_RPORT	PINF
#define AIO5_WPORT	PORTF
#define AIO5_DDR		DDRF
#define AIO5_PWM		NULL
#define AIO5_ADC		5

#define AIO6_PIN		PINF6
#define AIO6_RPORT	PINF
#define AIO6_WPORT	PORTF
#define AIO6_DDR		DDRF
#define AIO6_PWM		NULL
#define AIO6_ADC		6

#define AIO7_PIN		PINF7
#define AIO7_RPORT	PINF
#define AIO7_WPORT	PORTF
#define AIO7_DDR		DDRF
#define AIO7_PWM		NULL
#define AIO7_ADC		7

#define AIO8_PIN		PINK0
#define AIO8_RPORT	PINK
#define AIO8_WPORT	PORTK
#define AIO8_DDR		DDRK
#define AIO8_PWM		NULL
#define AIO8_ADC		8

#define AIO9_PIN		PINK1
#define AIO9_RPORT	PINK
#define AIO9_WPORT	PORTK
#define AIO9_DDR		DDRK
#define AIO9_PWM		NULL
#define AIO9_ADC		9

#define AIO10_PIN		PINK2
#define AIO10_RPORT	PINK
#define AIO10_WPORT	PORTK
#define AIO10_DDR		DDRK
#define AIO10_PWM		NULL
#define AIO10_ADC		10

#define AIO11_PIN		PINK3
#define AIO11_RPORT	PINK
#define AIO11_WPORT	PORTK
#define AIO11_DDR		DDRK
#define AIO11_PWM		NULL
#define AIO11_ADC		11

#define AIO12_PIN		PINK4
#define AIO12_RPORT	PINK
#define AIO12_WPORT	PORTK
#define AIO12_DDR		DDRK
#define AIO12_PWM		NULL
#define AIO12_ADC		12

#define AIO13_PIN		PINK5
#define AIO13_RPORT	PINK
#define AIO13_WPORT	PORTK
#define AIO13_DDR		DDRK
#define AIO13_PWM		NULL
#define AIO13_ADC		13

#define AIO14_PIN		PINK6
#define AIO14_RPORT	PINK
#define AIO14_WPORT	PORTK
#define AIO14_DDR		DDRK
#define AIO14_PWM		NULL
#define AIO14_ADC		14

#define AIO15_PIN		PINK7
#define AIO15_RPORT	PINK
#define AIO15_WPORT	PORTK
#define AIO15_DDR		DDRK
#define AIO15_PWM		NULL
#define AIO15_ADC		15



#undef PA0
#define PA0_PIN			PINA0
#define PA0_RPORT		PINA
#define PA0_WPORT		PORTA
#define PA0_DDR			DDRA
#define PA0_PWM			NULL
#undef PA1
#define PA1_PIN			PINA1
#define PA1_RPORT		PINA
#define PA1_WPORT		PORTA
#define PA1_DDR			DDRA
#define PA1_PWM			NULL
#undef PA2
#define PA2_PIN			PINA2
#define PA2_RPORT		PINA
#define PA2_WPORT		PORTA
#define PA2_DDR			DDRA
#define PA2_PWM			NULL
#undef PA3
#define PA3_PIN			PINA3
#define PA3_RPORT		PINA
#define PA3_WPORT		PORTA
#define PA3_DDR			DDRA
#define PA3_PWM			NULL
#undef PA4
#define PA4_PIN			PINA4
#define PA4_RPORT		PINA
#define PA4_WPORT		PORTA
#define PA4_DDR			DDRA
#define PA4_PWM			NULL
#undef PA5
#define PA5_PIN			PINA5
#define PA5_RPORT		PINA
#define PA5_WPORT		PORTA
#define PA5_DDR			DDRA
#define PA5_PWM			NULL
#undef PA6
#define PA6_PIN			PINA6
#define PA6_RPORT		PINA
#define PA6_WPORT		PORTA
#define PA6_DDR			DDRA
#define PA6_PWM			NULL
#undef PA7
#define PA7_PIN			PINA7
#define PA7_RPORT		PINA
#define PA7_WPORT		PORTA
#define PA7_DDR			DDRA
#define PA7_PWM			NULL

#undef PB0
#define PB0_PIN			PINB0
#define PB0_RPORT		PINB
#define PB0_WPORT		PORTB
#define PB0_DDR			DDRB
#define PB0_PWM			NULL
#undef PB1
#define PB1_PIN			PINB1
#define PB1_RPORT		PINB
#define PB1_WPORT		PORTB
#define PB1_DDR			DDRB
#define PB1_PWM			NULL
#undef PB2
#define PB2_PIN			PINB2
#define PB2_RPORT		PINB
#define PB2_WPORT		PORTB
#define PB2_DDR			DDRB
#define PB2_PWM			NULL
#undef PB3
#define PB3_PIN			PINB3
#define PB3_RPORT		PINB
#define PB3_WPORT		PORTB
#define PB3_DDR			DDRB
#define PB3_PWM			NULL
#undef PB4
#define PB4_PIN			PINB4
#define PB4_RPORT		PINB
#define PB4_WPORT		PORTB
#define PB4_DDR			DDRB
#define PB4_PWM			&OCR2A
#undef PB5
#define PB5_PIN			PINB5
#define PB5_RPORT		PINB
#define PB5_WPORT		PORTB
#define PB5_DDR			DDRB
#define PB5_PWM			NULL
#undef PB6
#define PB6_PIN			PINB6
#define PB6_RPORT		PINB
#define PB6_WPORT		PORTB
#define PB6_DDR			DDRB
#define PB6_PWM			NULL
#undef PB7
#define PB7_PIN			PINB7
#define PB7_RPORT		PINB
#define PB7_WPORT		PORTB
#define PB7_DDR			DDRB
#define PB7_PWM			&OCR0A

#undef PC0
#define PC0_PIN			PINC0
#define PC0_RPORT		PINC
#define PC0_WPORT		PORTC
#define PC0_DDR			DDRC
#define PC0_PWM			NULL
#undef PC1
#define PC1_PIN			PINC1
#define PC1_RPORT		PINC
#define PC1_WPORT		PORTC
#define PC1_DDR			DDRC
#define PC1_PWM			NULL
#undef PC2
#define PC2_PIN			PINC2
#define PC2_RPORT		PINC
#define PC2_WPORT		PORTC
#define PC2_DDR			DDRC
#define PC2_PWM			NULL
#undef PC3
#define PC3_PIN			PINC3
#define PC3_RPORT		PINC
#define PC3_WPORT		PORTC
#define PC3_DDR			DDRC
#define PC3_PWM			NULL
#undef PC4
#define PC4_PIN			PINC4
#define PC4_RPORT		PINC
#define PC4_WPORT		PORTC
#define PC4_DDR			DDRC
#define PC4_PWM			NULL
#undef PC5
#define PC5_PIN			PINC5
#define PC5_RPORT		PINC
#define PC5_WPORT		PORTC
#define PC5_DDR			DDRC
#define PC5_PWM			NULL
#undef PC6
#define PC6_PIN			PINC6
#define PC6_RPORT		PINC
#define PC6_WPORT		PORTC
#define PC6_DDR			DDRC
#define PC6_PWM			NULL
#undef PC7
#define PC7_PIN			PINC7
#define PC7_RPORT		PINC
#define PC7_WPORT		PORTC
#define PC7_DDR			DDRC
#define PC7_PWM			NULL

#undef PD0
#define PD0_PIN			PIND0
#define PD0_RPORT		PIND
#define PD0_WPORT		PORTD
#define PD0_DDR			DDRD
#define PD0_PWM			NULL
#undef PD1
#define PD1_PIN			PIND1
#define PD1_RPORT		PIND
#define PD1_WPORT		PORTD
#define PD1_DDR			DDRD
#define PD1_PWM			NULL
#undef PD2
#define PD2_PIN			PIND2
#define PD2_RPORT		PIND
#define PD2_WPORT		PORTD
#define PD2_DDR			DDRD
#define PD2_PWM			NULL
#undef PD3
#define PD3_PIN			PIND3
#define PD3_RPORT		PIND
#define PD3_WPORT		PORTD
#define PD3_DDR			DDRD
#define PD3_PWM			NULL
#undef PD4
#define PD4_PIN			PIND4
#define PD4_RPORT		PIND
#define PD4_WPORT		PORTD
#define PD4_DDR			DDRD
#define PD4_PWM			NULL
#undef PD5
#define PD5_PIN			PIND5
#define PD5_RPORT		PIND
#define PD5_WPORT		PORTD
#define PD5_DDR			DDRD
#define PD5_PWM			NULL
#undef PD6
#define PD6_PIN			PIND6
#define PD6_RPORT		PIND
#define PD6_WPORT		PORTD
#define PD6_DDR			DDRD
#define PD6_PWM			NULL
#undef PD7
#define PD7_PIN			PIND7
#define PD7_RPORT		PIND
#define PD7_WPORT		PORTD
#define PD7_DDR			DDRD
#define PD7_PWM			NULL

#undef PE0
#define PE0_PIN			PINE0
#define PE0_RPORT		PINE
#define PE0_WPORT		PORTE
#define PE0_DDR			DDRE
#define PE0_PWM			NULL
#undef PE1
#define PE1_PIN			PINE1
#define PE1_RPORT		PINE
#define PE1_WPORT		PORTE
#define PE1_DDR			DDRE
#define PE1_PWM			NULL
#undef PE2
#define PE2_PIN			PINE2
#define PE2_RPORT		PINE
#define PE2_WPORT		PORTE
#define PE2_DDR			DDRE
#define PE2_PWM			NULL
#undef PE3
#define PE3_PIN			PINE3
#define PE3_RPORT		PINE
#define PE3_WPORT		PORTE
#define PE3_DDR			DDRE
#define PE3_PWM			&OCR3AL
#undef PE4
#define PE4_PIN			PINE4
#define PE4_RPORT		PINE
#define PE4_WPORT		PORTE
#define PE4_DDR			DDRE
#define PE4_PWM			&OCR3BL
#undef PE5
#define PE5_PIN			PINE5
#define PE5_RPORT		PINE
#define PE5_WPORT		PORTE
#define PE5_DDR			DDRE
#define PE5_PWM			&OCR3CL
#undef PE6
#define PE6_PIN			PINE6
#define PE6_RPORT		PINE
#define PE6_WPORT		PORTE
#define PE6_DDR			DDRE
#define PE6_PWM			NULL
#undef PE7
#define PE7_PIN			PINE7
#define PE7_RPORT		PINE
#define PE7_WPORT		PORTE
#define PE7_DDR			DDRE
#define PE7_PWM			NULL

#undef PF0
#define PF0_PIN			PINF0
#define PF0_RPORT		PINF
#define PF0_WPORT		PORTF
#define PF0_DDR			DDRF
#define PF0_PWM			NULL
#undef PF1
#define PF1_PIN			PINF1
#define PF1_RPORT		PINF
#define PF1_WPORT		PORTF
#define PF1_DDR			DDRF
#define PF1_PWM			NULL
#undef PF2
#define PF2_PIN			PINF2
#define PF2_RPORT		PINF
#define PF2_WPORT		PORTF
#define PF2_DDR			DDRF
#define PF2_PWM			NULL
#undef PF3
#define PF3_PIN			PINF3
#define PF3_RPORT		PINF
#define PF3_WPORT		PORTF
#define PF3_DDR			DDRF
#define PF3_PWM			NULL
#undef PF4
#define PF4_PIN			PINF4
#define PF4_RPORT		PINF
#define PF4_WPORT		PORTF
#define PF4_DDR			DDRF
#define PF4_PWM			NULL
#undef PF5
#define PF5_PIN			PINF5
#define PF5_RPORT		PINF
#define PF5_WPORT		PORTF
#define PF5_DDR			DDRF
#define PF5_PWM			NULL
#undef PF6
#define PF6_PIN			PINF6
#define PF6_RPORT		PINF
#define PF6_WPORT		PORTF
#define PF6_DDR			DDRF
#define PF6_PWM			NULL
#undef PF7
#define PF7_PIN			PINF7
#define PF7_RPORT		PINF
#define PF7_WPORT		PORTF
#define PF7_DDR			DDRF
#define PF7_PWM			NULL

#undef PG0
#define PG0_PIN			PING0
#define PG0_RPORT		PING
#define PG0_WPORT		PORTG
#define PG0_DDR			DDRG
#define PG0_PWM			NULL
#undef PG1
#define PG1_PIN			PING1
#define PG1_RPORT		PING
#define PG1_WPORT		PORTG
#define PG1_DDR			DDRG
#define PG1_PWM			NULL
#undef PG2
#define PG2_PIN			PING2
#define PG2_RPORT		PING
#define PG2_WPORT		PORTG
#define PG2_DDR			DDRG
#define PG2_PWM			NULL
#undef PG3
#define PG3_PIN			PING3
#define PG3_RPORT		PING
#define PG3_WPORT		PORTG
#define PG3_DDR			DDRG
#define PG3_PWM			NULL
#undef PG4
#define PG4_PIN			PING4
#define PG4_RPORT		PING
#define PG4_WPORT		PORTG
#define PG4_DDR			DDRG
#define PG4_PWM			NULL
#undef PG5
#define PG5_PIN			PING5
#define PG5_RPORT		PING
#define PG5_WPORT		PORTG
#define PG5_DDR			DDRG
#define PG5_PWM			&OCR0B
#undef PG6
#define PG6_PIN			PING6
#define PG6_RPORT		PING
#define PG6_WPORT		PORTG
#define PG6_DDR			DDRG
#define PG6_PWM			NULL
#undef PG7
#define PG7_PIN			PING7
#define PG7_RPORT		PING
#define PG7_WPORT		PORTG
#define PG7_DDR			DDRG
#define PG7_PWM			NULL

#undef PH0
#define PH0_PIN			PINH0
#define PH0_RPORT		PINH
#define PH0_WPORT		PORTH
#define PH0_DDR			DDRH
#define PH0_PWM			NULL
#undef PH1
#define PH1_PIN			PINH1
#define PH1_RPORT		PINH
#define PH1_WPORT		PORTH
#define PH1_DDR			DDRH
#define PH1_PWM			NULL
#undef PH2
#define PH2_PIN			PINH2
#define PH2_RPORT		PINH
#define PH2_WPORT		PORTH
#define PH2_DDR			DDRH
#define PH2_PWM			NULL
#undef PH3
#define PH3_PIN			PINH3
#define PH3_RPORT		PINH
#define PH3_WPORT		PORTH
#define PH3_DDR			DDRH
#define PH3_PWM			&OCR4AL
#undef PH4
#define PH4_PIN			PINH4
#define PH4_RPORT		PINH
#define PH4_WPORT		PORTH
#define PH4_DDR			DDRH
#define PH4_PWM			&OCR4BL
#undef PH5
#define PH5_PIN			PINH5
#define PH5_RPORT		PINH
#define PH5_WPORT		PORTH
#define PH5_DDR			DDRH
#define PH5_PWM			&OCR4CL
#undef PH6
#define PH6_PIN			PINH6
#define PH6_RPORT		PINH
#define PH6_WPORT		PORTH
#define PH6_DDR			DDRH
#define PH6_PWM			&OCR2B
#undef PH7
#define PH7_PIN			PINH7
#define PH7_RPORT		PINH
#define PH7_WPORT		PORTH
#define PH7_DDR			DDRH
#define PH7_PWM			NULL

#undef PJ0
#define PJ0_PIN			PINJ0
#define PJ0_RPORT		PINJ
#define PJ0_WPORT		PORTJ
#define PJ0_DDR			DDRJ
#define PJ0_PWM			NULL
#undef PJ1
#define PJ1_PIN			PINJ1
#define PJ1_RPORT		PINJ
#define PJ1_WPORT		PORTJ
#define PJ1_DDR			DDRJ
#define PJ1_PWM			NULL
#undef PJ2
#define PJ2_PIN			PINJ2
#define PJ2_RPORT		PINJ
#define PJ2_WPORT		PORTJ
#define PJ2_DDR			DDRJ
#define PJ2_PWM			NULL
#undef PJ3
#define PJ3_PIN			PINJ3
#define PJ3_RPORT		PINJ
#define PJ3_WPORT		PORTJ
#define PJ3_DDR			DDRJ
#define PJ3_PWM			NULL
#undef PJ4
#define PJ4_PIN			PINJ4
#define PJ4_RPORT		PINJ
#define PJ4_WPORT		PORTJ
#define PJ4_DDR			DDRJ
#define PJ4_PWM			NULL
#undef PJ5
#define PJ5_PIN			PINJ5
#define PJ5_RPORT		PINJ
#define PJ5_WPORT		PORTJ
#define PJ5_DDR			DDRJ
#define PJ5_PWM			NULL
#undef PJ6
#define PJ6_PIN			PINJ6
#define PJ6_RPORT		PINJ
#define PJ6_WPORT		PORTJ
#define PJ6_DDR			DDRJ
#define PJ6_PWM			NULL
#undef PJ7
#define PJ7_PIN			PINJ7
#define PJ7_RPORT		PINJ
#define PJ7_WPORT		PORTJ
#define PJ7_DDR			DDRJ
#define PJ7_PWM			NULL

#undef PK0
#define PK0_PIN			PINK0
#define PK0_RPORT		PINK
#define PK0_WPORT		PORTK
#define PK0_DDR			DDRK
#define PK0_PWM			NULL
#undef PK1
#define PK1_PIN			PINK1
#define PK1_RPORT		PINK
#define PK1_WPORT		PORTK
#define PK1_DDR			DDRK
#define PK1_PWM			NULL
#undef PK2
#define PK2_PIN			PINK2
#define PK2_RPORT		PINK
#define PK2_WPORT		PORTK
#define PK2_DDR			DDRK
#define PK2_PWM			NULL
#undef PK3
#define PK3_PIN			PINK3
#define PK3_RPORT		PINK
#define PK3_WPORT		PORTK
#define PK3_DDR			DDRK
#define PK3_PWM			NULL
#undef PK4
#define PK4_PIN			PINK4
#define PK4_RPORT		PINK
#define PK4_WPORT		PORTK
#define PK4_DDR			DDRK
#define PK4_PWM			NULL
#undef PK5
#define PK5_PIN			PINK5
#define PK5_RPORT		PINK
#define PK5_WPORT		PORTK
#define PK5_DDR			DDRK
#define PK5_PWM			NULL
#undef PK6
#define PK6_PIN			PINK6
#define PK6_RPORT		PINK
#define PK6_WPORT		PORTK
#define PK6_DDR			DDRK
#define PK6_PWM			NULL
#undef PK7
#define PK7_PIN			PINK7
#define PK7_RPORT		PINK
#define PK7_WPORT		PORTK
#define PK7_DDR			DDRK
#define PK7_PWM			NULL

#undef PL0
#define PL0_PIN			PINL0
#define PL0_RPORT		PINL
#define PL0_WPORT		PORTL
#define PL0_DDR			DDRL
#define PL0_PWM			NULL
#undef PL1
#define PL1_PIN			PINL1
#define PL1_RPORT		PINL
#define PL1_WPORT		PORTL
#define PL1_DDR			DDRL
#define PL1_PWM			NULL
#undef PL2
#define PL2_PIN			PINL2
#define PL2_RPORT		PINL
#define PL2_WPORT		PORTL
#define PL2_DDR			DDRL
#define PL2_PWM			NULL
#undef PL3
#define PL3_PIN			PINL3
#define PL3_RPORT		PINL
#define PL3_WPORT		PORTL
#define PL3_DDR			DDRL
#define PL3_PWM			&OCR5AL
#undef PL4
#define PL4_PIN			PINL4
#define PL4_RPORT		PINL
#define PL4_WPORT		PORTL
#define PL4_DDR			DDRL
#define PL4_PWM			&OCR5BL
#undef PL5
#define PL5_PIN			PINL5
#define PL5_RPORT		PINL
#define PL5_WPORT		PORTL
#define PL5_DDR			DDRL
#define PL5_PWM			&OCR5CL
#undef PL6
#define PL6_PIN			PINL6
#define PL6_RPORT		PINL
#define PL6_WPORT		PORTL
#define PL6_DDR			DDRL
#define PL6_PWM			NULL
#undef PL7
#define PL7_PIN			PINL7
#define PL7_RPORT		PINL
#define PL7_WPORT		PORTL
#define PL7_DDR			DDRL
#define PL7_PWM			NULL
