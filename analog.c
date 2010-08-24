#include "analog.h"

#include	"arduino.h"

void analog_init() {
	PRR &= ~MASK(PRADC);
	ADMUX = REFERENCE;
	ADCSRA = MASK(ADEN) | MASK(ADPS2) | MASK(ADPS1) | MASK(ADPS0);
}

uint16_t	analog_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | channel;
	ADCSRA |= MASK(ADSC);
	// waits. I hate waiting
	for (;ADCSRA | MASK(ADSC););
	return ADC;
}
