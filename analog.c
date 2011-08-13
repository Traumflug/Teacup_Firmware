#include	"analog.h"

/** \file
	\brief Analog subsystem
*/

#include "temp.h"

#include	<avr/interrupt.h>

/* OR-combined mask of all channels */
#undef DEFINE_TEMP_SENSOR
//! automagically generate analog_mask from DEFINE_TEMP_SENSOR entries in config.h
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) | (((type == TT_THERMISTOR) || (type == TT_AD595)) ? 1 << (pin) : 0)

#ifdef	AIO8_PIN
	static const uint16_t analog_mask = 0
#else
	static const uint8_t analog_mask = 0
#endif

#include "config.h"
;
#undef DEFINE_TEMP_SENSOR

#ifdef AIO8_PIN
	#define	AINDEX_MASK	0x1F
	#define	AINDEX_MAX 15
	#define	AINDEX_CURRENT ((ADMUX & 0x07) | ((ADCSRB & MASK(MUX5))?0x08:0))
#else
	#define	AINDEX_MASK	0x0F
	#define	AINDEX_MAX 7
	#define	AINDEX_CURRENT (ADMUX & 0x07)
#endif

static uint8_t adc_counter;
static volatile uint16_t adc_result[AINDEX_MAX + 1] __attribute__ ((__section__ (".bss")));

//! Configure all registers, start interrupt loop
void analog_init() {
	if (analog_mask > 0) {
		// clear ADC bit in power reduction register because of ADC use.
		#ifdef	PRR
			PRR &= ~MASK(PRADC);
		#elif defined PRR0
			PRR0 &= ~MASK(PRADC);
		#endif

		// select reference signal to use, set right adjusted results and select ADC input 0
		ADMUX = REFERENCE;

		// ADC frequency must be less than 200khz or we lose precision. At 16MHz system clock, we must use the full prescale value of 128 to get an ADC clock of 125khz.
		ADCSRA = MASK(ADEN) | MASK(ADPS2) | MASK(ADPS1) | MASK(ADPS0);
		#ifdef	ADCSRB
			ADCSRB = 0;
		#endif

		adc_counter = 0;

		// clear analog inputs in the data direction register(s)
		AIO0_DDR &= ~analog_mask;
		#ifdef	AIO8_DDR
			AIO8_DDR &= ~(analog_mask >> 8);
		#endif

		// disable the analog inputs for digital use.
		DIDR0 = analog_mask & 0xFF;
		#ifdef	DIDR2
			DIDR2 = (analog_mask >> 8) & 0xFF;
		#endif

		// now we start the first conversion and leave the rest to the interrupt
		ADCSRA |= MASK(ADIE) | MASK(ADSC);
	} /* analog_mask > 0 */
}

/*! Analog Interrupt

	This is where we read our analog value and store it in an array for later retrieval
*/
ISR(ADC_vect, ISR_NOBLOCK) {
	// emulate free-running mode but be more deterministic about exactly which result we have, since this project has long-running interrupts
	if (analog_mask > 0) {
		// store next result
		adc_result[AINDEX_CURRENT] = ADC;

		// find next channel
		do {
			adc_counter++;
			adc_counter &= AINDEX_MAX;
		} while ((analog_mask & (1 << adc_counter)) == 0);

		// start next conversion
		ADMUX = (adc_counter & 0x07) | REFERENCE;
		#ifdef	MUX5
			if (adc_counter & 0x08)
				ADCSRB |= MASK(MUX5);
			else
				ADCSRB &= ~MASK(MUX5);
		#endif

		// After the mux has been set, start a new conversion 
		ADCSRA |= MASK(ADSC);
	}
}

/*! Read analog value from saved result array
	\param channel Channel to be read
	\return analog reading, 10-bit right aligned
*/
uint16_t	analog_read(uint8_t channel) {
	if (analog_mask > 0) {
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
	} else {
		return 0;
	}
}
