#include	"adc.h"

void adc_init(uint8_t vsel)
{
	PRR &= ~_BV(PRADC);
	ADMUX = (ADMUX & ~(_BV(REFS1) | _BV(REFS0))) | vsel;
	ADCSRA = _BV(ADEN);

	#if		F_CPU >= (ADC_MAX_FREQ * 128)
		#error	F_CPU is too high for ADC prescaler!
	#elif	F_CPU >= (ADC_MAX_FREQ * 64)
		ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
	#elif	F_CPU >= (ADC_MAX_FREQ * 32)
		ADCSRA |= _BV(ADPS2) | _BV(ADPS1);
	#elif	F_CPU >= (ADC_MAX_FREQ * 16)
		ADCSRA |= _BV(ADPS2) | _BV(ADPS0);
	#elif	F_CPU >= (ADC_MAX_FREQ * 8)
		ADCSRA |= _BV(ADPS2);
	#elif	F_CPU >= (ADC_MAX_FREQ * 4)
		ADCSRA |= _BV(ADPS1) | _BV(ADPS0);
	#elif	F_CPU >= (ADC_MAX_FREQ * 2)
		ADCSRA |= _BV(ADPS1);
	#elif	F_CPU >= (ADC_MAX_FREQ >> 1)
		ADSRA |= _BV(ADPS0);
	#else
		#error F_CPU is too slow for ADC to run well!
	#endif
}

uint16_t adc_read(uint8_t	pin)
{
	adc_start(pin);
	adc_wait();
	return adc_result();
}

void adc_start(uint8_t	pin)
{
	ADMUX = (ADMUX & 0xF0) | pin;
	ADCSRA |= _BV(ADSC);
}

uint16_t adc_result()
{
	return ADC;
}

uint8_t	adc_finished()
{
	return (ADCSRA & _BV(ADSC))?0xFF:0;
}

void adc_wait()
{
	for (;adc_finished() == 0;);
}
