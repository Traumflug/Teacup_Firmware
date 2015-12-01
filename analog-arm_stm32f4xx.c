
/** \file
  \brief Analog subsystem, ARM specific part.

*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__

#include "cmsis-stm32f4xx.h"
#include "arduino.h"
#include "temp.h"


/** Inititalise the analog subsystem.

  Initialise the ADC and start hardware scan loop for all used sensors.
*/
void analog_init() {

  if (NUM_TEMP_SENSORS) {                       // At least one channel in use.

    PIOC_2_PORT->MODER |= (GPIO_MODER_MODER0 << ((PIOC_2_PIN) << 1));   // analog mode
    PIOC_2_PORT->PUPDR &= ~(3 << ((PIOC_2_PIN) << 1));                  // no pullup
    PIOC_2_PORT->OSPEEDR |= (3 << ((PIOC_2_PIN) << 1));                 // high speed

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //Enable clock

    /* Set ADC parameters */
    /* Set the ADC clock prescaler */
    /*  Datasheet says max. ADC-clock is 36MHz
        We don't need that much. 12MHz is slowest possible.
    */
    ADC->CCR |= (ADC_CCR_ADCPRE);
    
    /* Set ADC scan mode */
    // scan mode disabled
    // reset resolution
    // discontinous mode disabled
    ADC1->CR1 &= ~( ADC_CR1_SCAN | 
                    ADC_CR1_RES |
                    ADC_CR1_DISCEN);
      
    /* Set ADC resolution */
    // resoltion 10bit
    ADC1->CR1 |=  ADC_CR1_RES_0;
    
    /* Set ADC data alignment */
    // reset = right align
    // reset external trigger
    //
    // disable continous conversion mode
    // disable ADC DMA continuous request
    // disable ADC end of conversion selection
    ADC1->CR2 &= ~( ADC_CR2_ALIGN |
                    ADC_CR2_EXTSEL |
                    ADC_CR2_EXTEN |
                    ADC_CR2_CONT |
                    ADC_CR2_DDS |
                    ADC_CR2_EOCS);
    
    /* Set ADC number of conversion */
    // 1 conversion
    ADC1->SQR1 &= ~(ADC_SQR1_L);

  }
}

/** Read analog value.

  \param channel Channel to be read.

  \return Analog reading, 10-bit right aligned.

  STM32F4 goes a different way. The ADC don't have a register for 
  each channel. We need a DMA soon to convert and hold all datas.
*/
//#include "delay.h"
uint16_t analog_read(uint8_t index) {
  // 11.8.2 Managing a sequence of conversions without using the DMA
  // page 220

  ADC1->SMPR1 &= ~(ADC_SMPR1_SMP12);  // PIOC_2_ADC 12
                                      // 3 CYCLES

  ADC1->SQR3 &= ~(ADC_SQR3_SQ1);      // rank 1
  ADC1->SQR3 |= PIOC_2_ADC;           // << (5 * (rank - 1))

  ADC1->CR2 |=  ADC_CR2_ADON          // A/D Converter ON / OFF
              | ADC_CR2_SWSTART;      // Start Conversion of regular channels

  while (!(ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC);

  return ADC1->DR;
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
