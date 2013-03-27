#include	"analog.h"

/** \file
	\brief Analog subsystem
*/

#include "temp.h"

#if defined (__ARMEL__)  // test ARM versus others

#include	<avr/interrupt.h>
#include	"memory_barrier.h"

/* OR-combined mask of all channels */
#undef DEFINE_TEMP_SENSOR
//! automagically generate analog_mask from DEFINE_TEMP_SENSOR entries in config.h
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
	| (((type == TT_THERMISTOR) || (type == TT_AD595)) ? (1 << (pin ## _ADC)) : 0)
#ifdef	AIO8_PIN
	static const uint16_t analog_mask = 0
#else
	static const uint8_t analog_mask = 0
#endif
#include "config.h"
;
#undef DEFINE_TEMP_SENSOR

static uint8_t adc_counter;
static volatile uint16_t adc_result[NUM_TEMP_SENSORS] __attribute__ ((__section__ (".bss")));

#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
	((type == TT_THERMISTOR) || (type == TT_AD595)) ? (pin ## _ADC) : 255,
static uint8_t adc_channel[NUM_TEMP_SENSORS] =
{
  #include "config.h"
};
#undef DEFINE_TEMP_SENSOR

// copied from /Applications/Arduino.app/Contents/Resources/Java/hardware/teensy/cores/teensy3/analog.c
// mapping from AIOn to ADCx_SC1n ADCH register
static const uint8_t channel2sc1a[] = {  
        5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
        0, 19, 3, 21, 26, 22
};


//! Configure all registers, start interrupt loop
void analog_initialize() {

  


	if (analog_mask > 0) {
	  analogReference(EXTERNAL); // 
	  analogReadRes(10);  // bits
	  analogReadAveraging(32); // hardware averaging
	      

	  for (int i =sizeof(adc_counter);i>=0; i--){
	    pinMode(adc_channel[i],INPUT);
	  }

		adc_counter = 0;

                        
		adc_result[adc_counter]=analogRead(adc_channel[adc_counter]); // 
		adc_counter++;

		// now we start the first conversion and leave the rest to the interrupt
		// AVR: ADCSRA |= MASK(ADIE) | MASK(ADSC);
		ADC0_SC1A = channel2sc1a[adc_channel[adc_counter]];
	} /* analog_mask > 0 */
}

/*! Analog Interrupt

	This is where we read our analog value and store it in an array for later retrieval
*/
ISR(ADC_vect) {


	// emulate free-running mode but be more deterministic about exactly which result we have, since this project has long-running interrupts
	if (analog_mask > 0) { // at least one temp sensor uses an analog channel
		// store next result
		adc_result[adc_counter] = ADC0_RA;

		// next channel
		do {
			adc_counter++;
			if (adc_counter >= sizeof(adc_channel))
				adc_counter = 0;
		} while (adc_channel[adc_counter] == 255);

		// start next conversion

		ADC0_SC1A = channel2sc1a[adc_channel[adc_counter]];
	}

}

/*! Read analog value from saved result array
	\param channel Channel to be read
	\return analog reading, 10-bit right aligned
*/
uint16_t	analog_read(uint8_t index) {
	if (analog_mask > 0) {
		uint16_t r;
		// disable interrupts
		cli();
		// atomic 16-bit copy
		r = adc_result[index];
		sei();

		return r;
	} else {
		return 0;
	}
}

#endif
