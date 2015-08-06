
/** \file
  \brief Analog subsystem, AVR specific part.
*/

#if defined TEACUP_C_INCLUDE && defined __AVR__

#include "pinio.h"
#include	"memory_barrier.h"


static uint8_t adc_counter;
static volatile uint16_t BSS adc_result[NUM_TEMP_SENSORS];

//! Configure all registers, start interrupt loop
void analog_init() {

  if (analog_mask) {  // At least one temp sensor uses an analog channel.
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
  } /* analog_mask */
}

/*! Analog Interrupt

	This is where we read our analog value and store it in an array for later retrieval
*/
ISR(ADC_vect, ISR_NOBLOCK) {
	// emulate free-running mode but be more deterministic about exactly which result we have, since this project has long-running interrupts
  if (analog_mask) {
		// store next result
		adc_result[adc_counter] = ADC;

		// next channel
		do {
			adc_counter++;
			if (adc_counter >= sizeof(adc_channel))
				adc_counter = 0;
		} while (adc_channel[adc_counter] == 255);

		// start next conversion
		ADMUX = (adc_channel[adc_counter] & 0x07) | REFERENCE;
		#ifdef	MUX5
			if (adc_channel[adc_counter] & 0x08)
				ADCSRB |= MASK(MUX5);
			else
				ADCSRB &= ~MASK(MUX5);
		#endif

		// After the mux has been set, start a new conversion
		ADCSRA |= MASK(ADSC);
	}
}

/** Read analog value from saved result array.

  \param channel Channel to be read. Channel numbering starts at zero.

  \return Analog reading, 10-bit right aligned.
*/
uint16_t analog_read(uint8_t index) {
  uint16_t result = 0;

  #ifdef AIO8_PIN
    if (index < sizeof(adc_channel) && adc_channel[index] < 16) {
  #else
    if (index < sizeof(adc_channel) && adc_channel[index] < 8) {
  #endif

    ATOMIC_START
      // Atomic 16-bit copy.
      result = adc_result[index];
    ATOMIC_END
  }

  return result;
}

#endif /* defined TEACUP_C_INCLUDE && defined __AVR__ */
