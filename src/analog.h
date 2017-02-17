#ifndef	_ANALOG_H
#define	_ANALOG_H

#include	<stdint.h>

#ifdef __AVR__
/** \def NEEDS_START_ADC

  Whether the board needs and implements start_adc(), i.e. analog reads
  on-demand (as opposed to free-running ADC conversions).
*/
#define NEEDS_START_ADC

/* #endif __AVR__ */
#elif __ARM_STM32F411__
  #define NEEDS_START_ADC
#endif /* __ARM_STM32F411__ */

void 			analog_init(void);

uint16_t	analog_read(uint8_t index);

#ifdef NEEDS_START_ADC
  void start_adc(void);
#endif

#endif	/* _ANALOG_H */
