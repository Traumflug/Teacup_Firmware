#ifndef	_ADC_H
#define	_ADC_H

#include	<stdint.h>
#include	<avr/io.h>

#define		VSEL_AREF	0
#define		VSEL_AVCC	_BV(REFS0)
#define		VSEL_1V1	_BV(REFS1)

#define		ADC_MAX_FREQ	200000

void adc_init(uint8_t	vsel);

uint16_t adc_read(uint8_t	pin);

void adc_start(uint8_t	pin);
inline uint16_t adc_result(void);

uint8_t	adc_finished(void);
void adc_wait(void);

#endif	/* _ADC_H */
