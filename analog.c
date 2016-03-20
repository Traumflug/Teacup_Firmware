
/** \file
	\brief Analog subsystem
*/

#include "analog.h"
#include "temp.h"

/**
  OR-combined mask of all channels. Auto-magically generated from
  DEFINE_TEMP_SENSOR() entries in config_wrapper.h
*/
#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  | (((type == TT_THERMISTOR) || (type == TT_AD595)) ? (1 << (pin ## _ADC)) : 0)
#if defined(AIO8_PIN) || defined(__ARM_STM32F411__)
  static const uint16_t analog_mask = 0
#else
  static const uint8_t analog_mask = 0
#endif
#include "config_wrapper.h"
;
#undef DEFINE_TEMP_SENSOR

/**
  A map of the ADC channels of the defined sensors.
*/
#ifndef __ARM_STM32F411__
#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  ((type == TT_THERMISTOR) || (type == TT_AD595)) ? (pin ## _ADC) : 255,
static uint8_t adc_channel[NUM_TEMP_SENSORS] = {
  #include "config_wrapper.h"
};
#undef DEFINE_TEMP_SENSOR
#endif

#define TEACUP_C_INCLUDE
#include "analog-avr.c"
#include "analog-arm_lpc11xx.c"
#include "analog-arm_stm32f4xx.c"
#undef TEACUP_C_INCLUDE

// No common code so far.
