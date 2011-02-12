#include "analog.h"

#include	<avr/interrupt.h>

#ifndef	ANALOG_MASK
	#warning	ANALOG_MASK not defined - analog subsystem disabled
	#define	ANALOG_MASK	0
#endif

uint8_t adc_running_mask, adc_counter;

#if	ANALOG_MASK & 2
	#define	ANALOG_START			1
	#define	ANALOG_START_MASK	2
#elif	ANALOG_MASK & 4
	#define	ANALOG_START			2
	#define	ANALOG_START_MASK	4
#elif	ANALOG_MASK & 8
	#define	ANALOG_START			3
	#define	ANALOG_START_MASK	8
#elif	ANALOG_MASK & 16
	#define	ANALOG_START			4
	#define	ANALOG_START_MASK	16
#elif	ANALOG_MASK & 32
	#define	ANALOG_START			5
	#define	ANALOG_START_MASK	32
#elif	ANALOG_MASK & 64
	#define	ANALOG_START			6
	#define	ANALOG_START_MASK	64
#elif	ANALOG_MASK & 128
	#define	ANALOG_START			7
	#define	ANALOG_START_MASK	128
#else
	// ANALOG_MASK == 1 or 0, either way defines are the same except they're not used if ANALOG_MASK == 0
	#define	ANALOG_START			0
	#define	ANALOG_START_MASK	1
#endif

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

		AIO0_DDR &= ~(ANALOG_MASK);
		DIDR0 = (ANALOG_MASK) & 0x3F;

		// now we start the first conversion and leave the rest to the interrupt
		ADCSRA |= MASK(ADIE) | MASK(ADSC);
	#endif /* ANALOG_MASK > 0 */
}

ISR(ADC_vect, ISR_NOBLOCK) {
	// emulate free-running mode but be more deterministic about exactly which result we have, since this project has long-running interrupts
	adc_result[ADMUX & 0x0F] = ADC;
	// find next channel
	do {
		adc_counter++;
		adc_running_mask <<= 1;
		if (adc_counter == 8) {
			adc_counter = ANALOG_START;
			adc_running_mask = ANALOG_START_MASK;
		}
	} while ((adc_running_mask & (ANALOG_MASK)) == 0);

	// start next conversion
	ADMUX = (adc_counter) | REFERENCE;
	ADCSRA |= MASK(ADSC);
}

uint16_t	analog_read(uint8_t channel) {
	#if ANALOG_MASK > 0
		uint16_t r;

		uint8_t sreg;
		// save interrupt flag
		sreg = SREG;
		// disable interrupts
		cli();

		// atomic 16-bit copy
		r = adc_result[channel];

		// restore interrupt flag
		SREG = sreg;

		return r;
	#else
		return 0;
	#endif
}
