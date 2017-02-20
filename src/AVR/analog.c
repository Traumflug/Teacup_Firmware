
/** \file
  \brief Analog subsystem, AVR specific part.
*/

#ifdef __AVR__

#include "analog.h"
#include "temp.h"
#include "pinio.h"
#include "memory_barrier.h"

  // TODO: these reference selectors should go away. A nice feature, but
  //       none of the RepRap controllers has use for it.

/** \def REFERENCE_AREF
  This compares the voltage to be measured against the voltage on the Aref pin.
  This also requires a voltage to be actually provided on the Aref pin, which
  none of the commonly available controllers or Arduinos do.
*/
#define REFERENCE_AREF  0

/** \def REFERENCE_AVCC
  This compares the voltage to be measured against the voltage on the Aref pin,
  but also connects AVcc to Aref, so no external voltage is required. Using
  this is said to be more accurate than doing this connection externally, on
  the pins of the chip, so this is the most commonly used option.
*/
#define REFERENCE_AVCC  64

/** \def REFERENCE_1V1
    \def REFERENCE_2V56
  These compare the voltage to be measured against internally created
  1.1 or 2.56 volts. Not useful on commonly available RepRap controllers,
  but might be a good choice for custom devices providing only a low voltage.
*/
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega168P__) || \
    defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
  #define REFERENCE_1V1   192
#else
  #define REFERENCE_1V1   128
  #define REFERENCE_2V56  192
#endif

/** \def REFERENCE
  Which analog reference to use. As none of the known controllers provides a
  fixed voltage on the Aref pin and all of them have a thermistor measurement
  range of 0..5 volts, let's clamp this here to the only choice, AVcc, instead
  of confusing the user with a choice.
*/
//#include "config_wrapper.h"
#define REFERENCE REFERENCE_AVCC

#ifndef REFERENCE
  #error REFERENCE undefined. See analog.h on how to choose it.
#endif

#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  | (((type == TT_THERMISTOR) || (type == TT_AD595)) ? (1 << (pin ## _ADC)) : 0)
#if defined(AIO8_PIN)
  static const uint16_t analog_mask = 0
#else
  static const uint8_t analog_mask = 0
#endif
#include "../../config_wrapper.h"
;
#undef DEFINE_TEMP_SENSOR

/**
  A map of the ADC channels of the defined sensors.
*/
#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  ((type == TT_THERMISTOR) || (type == TT_AD595)) ? (pin ## _ADC) : 255,
static uint8_t adc_channel[NUM_TEMP_SENSORS] = {
  #include "../../config_wrapper.h"
};
#undef DEFINE_TEMP_SENSOR

static uint8_t adc_counter = 0;
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

    // Enable the ADC.
    ADCSRA |= MASK(ADIE);
  } /* analog_mask */
}

/**
  Start a new ADC conversion.
*/
void start_adc() {
  ADCSRA |= MASK(ADSC);
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

    // If there is another channel to read, start a new conversion.
    if (adc_counter != 0) {
      ADCSRA |= MASK(ADSC);
    }
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

#endif /* __AVR__ */
