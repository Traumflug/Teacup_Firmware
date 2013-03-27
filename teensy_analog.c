#ifdef __ARMEL__

#include "core_pins.h"

static uint8_t calibrating;
static uint8_t analog_right_shift = 0;
static uint8_t analog_config_bits = 10;
static uint8_t analog_num_average = 4;
static uint8_t analog_reference_internal = 0;

// The alternate clock is connected to OSCERCLK (16 MHz).
// Datasheet says ADC clock should be 2 to 12 MHz for 16 bit mode.
// Datasheet says ADC clock should be 1 to 18 MHz for 8-12 bit mode.

#if F_BUS == 48000000
  #define ADC0_CFG1_6MHZ   ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(1)
  #define ADC0_CFG1_12MHZ  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(1)
  #define ADC0_CFG1_24MHZ  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(1)
#elif F_BUS == 24000000
  #define ADC0_CFG1_6MHZ   ADC_CFG1_ADIV(2) + ADC_CFG1_ADICLK(0)
  #define ADC0_CFG1_12MHZ  ADC_CFG1_ADIV(1) + ADC_CFG1_ADICLK(0)
  #define ADC0_CFG1_24MHZ  ADC_CFG1_ADIV(0) + ADC_CFG1_ADICLK(0)
#else
#error
#endif

void analog_init(void) {
  uint32_t num;

  VREF_TRM = 0x60;
  VREF_SC = 0xE1;   // enable 1.2 volt ref

  if (analog_config_bits == 8) {
    ADC0_CFG1 = ADC0_CFG1_24MHZ + ADC_CFG1_MODE(0);
    ADC0_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(3);
  } else if (analog_config_bits == 10) {
    ADC0_CFG1 = ADC0_CFG1_12MHZ + ADC_CFG1_MODE(2) + ADC_CFG1_ADLSMP;
    ADC0_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(3);
  } else if (analog_config_bits == 12) {
    ADC0_CFG1 = ADC0_CFG1_12MHZ + ADC_CFG1_MODE(1) + ADC_CFG1_ADLSMP;
    ADC0_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(2);
  } else {
    ADC0_CFG1 = ADC0_CFG1_12MHZ + ADC_CFG1_MODE(3) + ADC_CFG1_ADLSMP;
    ADC0_CFG2 = ADC_CFG2_MUXSEL + ADC_CFG2_ADLSTS(2);
  }

  if (analog_reference_internal) {
    ADC0_SC2 = ADC_SC2_REFSEL(1); // 1.2V ref
  } else {
    ADC0_SC2 = ADC_SC2_REFSEL(0); // vcc/ext ref
  }

  num = analog_num_average;
  if (num <= 1) {
    ADC0_SC3 = ADC_SC3_CAL;  // begin cal
  } else if (num <= 4) {
    ADC0_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(0);
  } else if (num <= 8) {
    ADC0_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(1);
  } else if (num <= 16) {
    ADC0_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(2);
  } else {
    ADC0_SC3 = ADC_SC3_CAL + ADC_SC3_AVGE + ADC_SC3_AVGS(3);
  }
  calibrating = 1;
}

static void wait_for_cal(void) {
  uint16_t sum;

  while (ADC0_SC3 & ADC_SC3_CAL) {
    // wait
  }

  sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
  sum = (sum / 2) | 0x8000;
  ADC0_PG = sum;

  sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
  sum = (sum / 2) | 0x8000;
  ADC0_MG = sum;

  calibrating = 0;
}

// ADCx_SC2[REFSEL] bit selects the voltage reference sources for ADC.
//   VREFH/VREFL - connected as the primary reference option
//   1.2 V VREF_OUT - connected as the VALT reference option

#define DEFAULT         0
#define INTERNAL        2
#define INTERNAL1V2     2
#define INTERNAL1V1     2
#define EXTERNAL        0

void analogReference(uint8_t type) {
  if (type) {
    // internal reference requested
    if (!analog_reference_internal) {
      analog_reference_internal = 1;
      if (calibrating)
        ADC0_SC3 = 0; // cancel cal
      analog_init();
    }
  } else {
    // vcc or external reference requested
    if (analog_reference_internal) {
      analog_reference_internal = 0;
      if (calibrating)
        ADC0_SC3 = 0; // cancel cal
      analog_init();
    }
  }
}

void analogReadRes(unsigned int bits) {
  unsigned int config;

  if (bits >= 13) {
    if (bits > 16) bits = 16;
    config = 16;
  } else if (bits >= 11) {
    config = 12;
  } else if (bits >= 9) {
    config = 10;
  } else {
    config = 8;
  }
  analog_right_shift = config - bits;
  if (config != analog_config_bits) {
    analog_config_bits = config;
    if (calibrating) ADC0_SC3 = 0; // cancel cal
    analog_init();
  }
}

void analogReadAveraging(unsigned int num) {

  if (calibrating)
    wait_for_cal();

  if (num <= 1) {
    num = 0;
    ADC0_SC3 = 0;
  } else if (num <= 4) {
    num = 4;
    ADC0_SC3 = ADC_SC3_AVGE + ADC_SC3_AVGS(0);
  } else if (num <= 8) {
    num = 8;
    ADC0_SC3 = ADC_SC3_AVGE + ADC_SC3_AVGS(1);
  } else if (num <= 16) {
    num = 16;
    ADC0_SC3 = ADC_SC3_AVGE + ADC_SC3_AVGS(2);
  } else {
    num = 32;
    ADC0_SC3 = ADC_SC3_AVGE + ADC_SC3_AVGS(3);
  }
  analog_num_average = num;
}

// The SC1A register is used for both software and hardware trigger modes of operation.

static const uint8_t channel2sc1a[] = {
  5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
  0, 19, 3, 21, 26, 22
};

int analogRead(uint8_t pin)
{
  int result;

  if (pin >= 14) {
    if (pin <= 23) {
      pin -= 14;  // 14-23 are A0-A9
    } else if (pin >= 34 && pin <= 39) {
      pin -= 24;  // 34-37 are A10-A13, 38 is temp sensor, 39 is vref
    } else {
      return 0;   // all others are invalid
    }
  }

  if (calibrating)
    wait_for_cal();

  ADC0_SC1A = channel2sc1a[pin];
  while ((ADC0_SC1A & ADC_SC1_COCO) == 0) {
    yield();
    // wait
  }

  result = ADC0_RA >> analog_right_shift;

  return result;
}

#endif /* __ARMEL__ */

