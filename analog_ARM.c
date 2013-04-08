#include	"analog.h"

/** \file
	\brief Analog subsystem
*/

#include "temp.h"

#if defined (__ARMEL__)  // test ARM versus others

#include	<avr/interrupt.h>
#include	"memory_barrier.h"

/// For debugging:
#include	"sermsg.h"
#include	"sersendf.h"



//
/* OR-combined mask of all ADC channels */
//AIO0-AIO9 end up in bits 0-9 
//AIO10-AIO13 end up in bits 20-23
//
#undef DEFINE_TEMP_SENSOR
//! automagically generate analog_mask from DEFINE_TEMP_SENSOR entries in config.h
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  | (((type == TT_THERMISTOR) || (type == TT_AD595)) ? (1 << ((pin ## _PIN)-AIO0_PIN)) : 0)
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

// Map from AIOx to n of DIOn
//                  [AIO0, AIO1]  = [14, 15] 
// setup adc_channel[0,1,...]= [14,15...] DIO pin
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
	((type == TT_THERMISTOR) || (type == TT_AD595)) ? (pin ## _DIO) : 255,
static uint8_t adc_dio[NUM_TEMP_SENSORS] =
{
  #include "config.h"
};
#undef DEFINE_TEMP_SENSOR


// Map from AIOx to n of DIOn
//                  [AIO0, AIO1]  = [14, 15] 
// setup adc_channel[0,1,...]= [14,15...] DIO pin
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
	((type == TT_THERMISTOR) || (type == TT_AD595)) ? (pin ## _CHANNEL) : 255,
static uint8_t adc_channel[NUM_TEMP_SENSORS] =
{
  #include "config.h"
};
#undef DEFINE_TEMP_SENSOR



//! Configure all registers, start interrupt loop
void analog_initialize() {

  


	if (analog_mask > 0) {
	  uint16_t sum;


          //analog_init(); Teensyduino .../analog.c:analog_init() elements:
	  
	  VREF_TRM = 0x60;        // Chop and trim
	  VREF_SC = 0xE1;         // enable 1.2 volt ref
#define ADC0_CFG1_12MHZ  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1)
	  ADC0_CFG1 = ADC0_CFG1_12MHZ + ADC_CFG1_MODE(2) + ADC_CFG1_ADLSMP; // 12MHz, Long samples, 10bit
	  ADC0_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(0); //ADxxb channels, long 24 cycle samples
	  // analogReference(EXTERNAL); // Teensyduino .../analog.c:analong_init() and analogReference() 
	  ADC0_SC2 = ADC_SC2_REFSEL(0); // vcc/ext ref

          ADC0_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(3); // start Calibration, hardware 32 sample averages

	  while (ADC0_SC3 & ADC_SC3_CAL) {  //wait for calibration
                // wait
	  }
	  sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
	  sum = (sum / 2) | 0x8000;
	  ADC0_PG = sum;
 	  sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
	  sum = (sum / 2) | 0x8000;
	  ADC0_MG = sum;                   // calibration complete
 	      

	  for (int i =sizeof(adc_counter);i>=0; i--){
	    pinMode(adc_dio[i],INPUT);
	  }

	  adc_counter = 0;

                        
	  //  adc_result[adc_counter]=analogRead(adc_channel[adc_counter]); // 
	  //adc_counter++;

	  // now we start the first conversion and leave the rest to the interrupt
	  // AVR: ADCSRA |= MASK(ADIE) | MASK(ADSC);
	  //sersendf_P(PSTR("channel2sc1a[adc_channel[adc_counter==%d]==%d]==%d"),adc_counter,adc_channel[adc_counter],channel2sc1a[adc_channel[adc_counter]]);
	  ADC0_SC1A =  ADC_SC1_AIEN | adc_channel[adc_counter];
	  NVIC_ENABLE_IRQ(IRQ_ADC0); // enable interrupt handler for ADC0 interrupts
	} /* analog_mask > 0 */
}

/*! Analog Interrupt

	This is where we read our analog value and store it in an array for later retrieval
*/
ISR(adc0_isr) {
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
		ADC0_SC1A = ADC_SC1_AIEN | adc_channel[adc_counter];
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
