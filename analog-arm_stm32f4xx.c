
/** \file
  \brief Analog subsystem, ARM specific part.
  STM32F4 goes a different way. The ADC don't have a register for
  each channel. We are using DMA instead.
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__

#include "cmsis-stm32f4xx.h"
#include "arduino.h"
#include "delay.h"
#include "temp.h"
#include <string.h>

// DMA ADC-buffer
#define OVERSAMPLE 6
static uint16_t BSS adc_buffer[NUM_TEMP_SENSORS * OVERSAMPLE];

// Private functions
void init_analog(void);
void init_dma(void);

/** Initialize the analog subsystem.

  Initialize the ADC and start hardware scan for all sensors.
*/
void analog_init() {

  if (NUM_TEMP_SENSORS) {                       // At least one channel in use.
    init_dma();
    init_analog();
  }

}

/** Initialize all analog pins from config
 
  Initialize the pins to analog mode, no pullup/no pulldown, highspeed
*/
void init_analog() {

  #undef DEFINE_TEMP_SENSOR
  /*
   config analog pins
   1. analog mode
   2. no pullup
   3. high speed
  */
  #define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
    pin ## _PORT->MODER |= (GPIO_MODER_MODER0 << ((pin ## _PIN) << 1)); \
    pin ## _PORT->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((pin ## _PIN) << 1)); \
    pin ## _PORT->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << ((pin ## _PIN) << 1));
  #include "config_wrapper.h"
  #undef DEFINE_TEMP_SENSOR

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //Enable clock

  /* Set ADC parameters */
  /* Set the ADC clock prescaler */
  ADC->CCR |= ADC_CCR_ADCPRE;

  ADC1->CR1 &=  ~(ADC_CR1_RES);
  ADC1->CR1 |=  ADC_CR1_RES_0;

  ADC1->CR1 |= ADC_CR1_SCAN | ADC_CR1_OVRIE;
  ADC1->CR2 |= ADC_CR2_DMA;

  /* Set ADC number of conversion */
  // ((NUM_TEMP_SENSORS) - 1) << 20
  ADC1->SQR1 &= ~(ADC_SQR1_L);
  ADC1->SQR1 |= (NUM_TEMP_SENSORS - 1) << 20;

  // for loop over each channel (0..15) for sequence
  #undef DEFINE_TEMP_SENSOR
  // for PIO ## ADC >= 10 SRPR1 and ADC -10, else SMPR 2
  // 0x06 = 144 cycles
  // subt line is to keep compiler happy
  #define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  if (NUM_TEMP_SENSORS) { \
    uint8_t subt = (pin ## _ADC >= 10) ? 10 : 0; \
    if (pin ## _ADC >= 10) { \
      ADC1->SMPR1 |= (uint32_t)0x06 << (3 * ((pin ## _ADC) - subt)); \
    } else { \
      ADC1->SMPR2 |= (uint32_t)0x06 << (3 * ((pin ## _ADC) - subt)); \
    } \
    subt = (TEMP_SENSOR_ ## name <= 5) ? 0 : (TEMP_SENSOR_ ## name <= 11) ? 6 : 12; \
    if (TEMP_SENSOR_ ## name <= 5) { \
      ADC1->SQR3 |= pin ## _ADC << (5 * TEMP_SENSOR_ ## name - subt); \
    } else \
    if (TEMP_SENSOR_ ## name <= 11) { \
      ADC1->SQR2 |= pin ## _ADC << (5 * (TEMP_SENSOR_ ## name - subt)); \
    } else { \
      ADC1->SQR1 |= pin ## _ADC << (5 * (TEMP_SENSOR_ ## name - subt)); \
    } \
  }
  #include "config_wrapper.h"
  #undef DEFINE_TEMP_SENSOR
  ADC1->CR2 |= ADC_CR2_CONT;
  ADC1->CR2 |= ADC_CR2_ADON;         // A/D Converter ON / OFF
  ADC1->CR2 |= ADC_CR2_SWSTART;
}

/** Init the DMA for ADC
*/

void init_dma() {

  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // Enable clock

  /**
    We have two DMA streams for ADC1. (DMA2_Stream0 and DMA2_Stream4)
    We take DMA2 Stream4.
    See reference manual 9.3.3 channel selection (p. 166)
  */
  // 1. Disable DMA-Stream
  DMA2_Stream4->CR &= ~DMA_SxCR_EN;
  while(DMA2_Stream4->CR & DMA_SxCR_EN);  // we wait until it is disabled.
  uint32_t tmp_CR = 0; //DMA2_Stream4->CR;

  // 2. perihperal port register address
  DMA2_Stream4->PAR = (uint32_t)&ADC1->DR;

  // 3. memory address
  DMA2_Stream4->M0AR = (uint32_t)&adc_buffer;

  // 4. total number of data items
  DMA2_Stream4->NDTR = NUM_TEMP_SENSORS * OVERSAMPLE;

  // 5. DMA channel
  // channel 0
  tmp_CR &= ~(DMA_SxCR_CHSEL);

  // 6.
  // 7. priority
  // Very high
  tmp_CR |= DMA_SxCR_PL;

  // 8. FIFO
  DMA2_Stream4->FCR &= ~(DMA_SxFCR_DMDIS);

  // 9. config the rest
  /*
   * halfword for memory and periphal: the 12bit ADC is 16bit right aligned
   * memory inc.: we read any adc and doing a step of 16bits after each conversion
   * circular mode: repeat until inf
   * double buffer mode: we write to one adress and read the other
   */
  tmp_CR &= ~(DMA_SxCR_DIR);
  tmp_CR |= DMA_SxCR_MSIZE_0 |
            DMA_SxCR_PSIZE_0 |
            DMA_SxCR_MINC |
            DMA_SxCR_CIRC |
            DMA_SxCR_TCIE;

  DMA2_Stream4->CR = tmp_CR;

  // 10. Enable DMA-Stream
  DMA2_Stream4->CR |= DMA_SxCR_EN;
  while(!(DMA2_Stream4->CR & DMA_SxCR_EN));

  NVIC_SetPriority(DMA2_Stream4_IRQn,
  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 1));
  NVIC_EnableIRQ(DMA2_Stream4_IRQn);                // Enable interrupt generally.

}

/**
  DMA2 Stream4 interrupt.

  Happens every time the complete stream is written.
  In that case, we can turn of the ADC until we read it.

  Must have the same name as in cmsis-startup_stm32f411xe.s.
*/
void DMA2_Stream4_IRQHandler(void) {
  DMA2->HIFCR = DMA_HIFCR_CTCIF4;
  ADC1->CR2 &= ~(ADC_CR2_DMA);     // clear DMA flag; we restart it in start_adc()
}

/** Read analog value.

  \param channel Channel to be read.

  \return Analog reading, 10-bit right aligned.

*/

uint16_t analog_read(uint8_t index) {
  if (NUM_TEMP_SENSORS > 0) {
    uint16_t r = 0;
    uint16_t temp;
    uint16_t max_temp = 0;
    uint16_t min_temp = 0xffff;

    for (uint8_t i = 0; i < OVERSAMPLE; i++) {
      temp = adc_buffer[index + NUM_TEMP_SENSORS * i];
      max_temp = max_temp > temp ? max_temp : temp;
      min_temp = min_temp < temp ? min_temp : temp;
      r += temp;
    }

    r = (r - max_temp - min_temp) / (OVERSAMPLE - 2);
    return r;
  } else {
    return 0;
  }
  // memset(adc_buffer[!(DMA2_Stream4->CR & DMA_SxCR_CT)], 0, sizeof(adc_buffer[0]));
}

/**
  Start a new ADC conversion.
*/
void start_adc() {
  ADC1->CR2 |= ADC_CR2_DMA;
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
