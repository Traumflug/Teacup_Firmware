
/** \file
  \brief Analog subsystem, ARM specific part.

  Nice feature of the LPC11xx is, it features a hardware scan mode, which scans
  a given set of ADC pins over and over again, without need for an interrupt
  to switch between channels. Our code becomes pretty simple: set this
  mechanism up at init time and be done for the time being.

  Unlike ATmegas, LPC1114 features no analog voltage reference selector, it
  always compares against Vdd. This is the equivalent of REFERENCE_AVCC on AVR.
*/

#if defined TEACUP_C_INCLUDE && defined __ARMEL__

#include "cmsis-lpc11xx.h"
#include "arduino.h"


/** Inititalise the analog subsystem.

  Initialise the ADC and start hardware scan loop for all used sensors.

  LPC111x User Manual recommends an ADC clock of 4.5 MHz ("typically"), still
  we reduce this to 1 MHz, because we're not nearly as much in a hurry. A
  conversion needs 11 ADC clocks per channel, so we still get many thousand
  conversions/second; we need just 100/s.
*/
void analog_init() {

  if (analog_mask) {                            // At least one channel in use.

    LPC_SYSCON->PDRUNCFG &= ~(1 << 4);            // Turn on ADC clock.
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 13);       // Turn on ADC power.

    /**
      Register name mapping from LPC111x User Manual to CMSIS headers:

        chap. 25.5     cmsis-lpc11xx.h   description

        AD0CR          LPC_ADC->CR       A/D control register.
        AD0GDR         LPC_ADC->GDR      A/D global data register.
        AD0INTEN       LPC_ADC->INTEN    A/D interrupt enable register.
        AD0DRn         LPC_ADC->DR[n]    A/D channel n data register.
        AD0STAT        LPC_ADC->STAT     A/D status register.
    */
    LPC_ADC->CR = ((uint32_t)analog_mask << 0)    // Mask of used pins.
                | ((F_CPU / 1000000) << 8)        // 1 MHz ADC clock.
                | (1 << 16)                       // Hardware scan mode.
                | (0x0 << 17)                     // Maximum accuracy.
                | (0x0 << 24);                    // Clear START.

    LPC_ADC->INTEN = 0;                           // No interrupt generation.

    // Auto-generate pin setup.
    #undef DEFINE_TEMP_SENSOR
    #define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
      LPC_IOCON->pin ## _CMSIS = \
        ((type == TT_THERMISTOR) || (type == TT_AD595)) ? ( \
                                 (0x2 << 0)       /* Function ADx.       */ \
                               | (0 << 3)         /* Pullup inactive.    */ \
                               | (0 << 7))        /* Analog input mode.  */ \
          : LPC_IOCON->pin ## _CMSIS;
    #include "config_wrapper.h"
    #undef DEFINE_TEMP_SENSOR
  } /* analog_mask */
}

/** Read analog value.

  \param channel Channel to be read. Channel numbering starts at zero.

  \return Analog reading, 10-bit right aligned.

  Hardware scan mode nicely saves all the converted values separately, so
  there's no need to hold them in a copy.
*/
uint16_t analog_read(uint8_t index) {
  uint16_t result = 0;

  if (index < sizeof(adc_channel) && adc_channel[index] < 8) {
    result = (LPC_ADC->DR[adc_channel[index]] & 0xFFC0) >> 6;
  }

  return result;
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
