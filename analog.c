#include "analog.h"

#include	<avr/interrupt.h>
#include	<util/atomic.h>

#ifndef	ANALOG_MASK
	#warning	define ANALOG_MASK as a bitmask of all the analog channels you wish to use
	#error		ANALOG_MASK not defined
#endif

uint8_t adc_running_mask, adc_counter;

volatile uint16_t adc_result[8] __attribute__ ((__section__ (".bss")));

void analog_init() {
	#if ANALOG_MASK > 0
	#ifdef	PRR
		PRR &= ~MASK(PRADC);
	#elif defined PRR0
		PRR0 &= ~MASK(PRADC);
	#endif
	ADMUX = REFERENCE;
	// ADC frequency must be less than 200khz or we lose precision. At 16MHz system clock, we must use the full prescale value of 128 to get an ADC clock of 125khz.
	ADCSRA = MASK(ADEN) | MASK(ADPS2) | MASK(ADPS1) | MASK(ADPS0);

	adc_counter = 0;
	adc_running_mask = 1;

	DIDR0 = ANALOG_MASK & 0x1F;

	// now we start the first conversion and leave the rest to the interrupt
	ADCSRA |= MASK(ADIE) | MASK(ADSC);
	#endif
}

ISR(ADC_vect) {
	// emulate free-running mode but be more deterministic about exactly which result we have, since this project has long-running interrupts
	adc_result[ADMUX & 0x0F] = ADC;
	// find next channel
	do {
		adc_counter++;
		adc_running_mask <<= 1;
		if (adc_counter == 8) {
			adc_counter = 0;
			adc_running_mask = 1;

			// relax interrupt use for analog subsystem- stop after last analog read
			ADCSRA &= ~MASK(ADIE);
		}
	} while ((adc_running_mask & ANALOG_MASK) == 0);

	// start next conversion
	ADMUX = (adc_counter) | REFERENCE;
	ADCSRA |= MASK(ADSC);
}

uint16_t	analog_read(uint8_t channel) {
	uint16_t r;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// atomic 16-bit copy
		r = adc_result[channel];
	}

	// re-enable analog read loop so we can get new values
	ADCSRA |= MASK(ADIE);
	
	return r;
}
