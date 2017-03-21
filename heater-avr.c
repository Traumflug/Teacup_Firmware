
/** \file
  \brief Manage heaters, including PID and PWM, AVR specific part.

  For test cases see the intro comment in heater.c.
*/

#if defined TEACUP_C_INCLUDE && defined __AVR__

#include	<stdlib.h>
#include "pinio.h"
#include	"crc.h"
#include "sersendf.h"
#include "debug.h"

/// \struct heater_definition_t
/// \brief simply holds pinout data- port, pin, pwm channel if used

typedef struct {
  union {
    volatile uint8_t *heater_port;  ///< pointer to port. DDR is inferred from this pointer too
    volatile uint8_t *heater_pwm;   ///< pointer to 8-bit PWM register, eg OCR0A (8-bit) or ORC3L (low byte, 16-bit)
  };
  uint8_t     masked_pin;    ///< heater pin, masked. eg for PB3 enter '1 << 3' here, or 1 << PB3_PIN or similar

  uint16_t    max_value;     ///< max value for the heater, for PWM in percent * 256
  pwm_type_t  pwm_type;      ///< saves the pwm-type: NO_PWM, SOFTWARE_PWM, HARDWARE_PWM
  uint8_t     invert;        ///< Wether the heater pin signal needs to be inverted.
} heater_definition_t;

// When pwm >= 2 it's hardware pwm, if the pin has hardware pwm.
// When pwm == 1 it's software pwm.
// pwm == 0 is no pwm at all.
// Use this macro only in DEFINE_HEATER_ACTUAL-macros.
#define PWM_TYPE(pwm, pin) (((pwm) >= HARDWARE_PWM_START) ? ((pin ## _PWM) ? HARDWARE_PWM : SOFTWARE_PWM) : pwm)

#undef DEFINE_HEATER_ACTUAL
/// \brief helper macro to fill heater definition struct from config.h
#define DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, max_value) { \
  {(PWM_TYPE(pwm, pin) == HARDWARE_PWM) ?                         \
    (pin ## _PWM) :                                               \
    &(pin ## _WPORT),                                             \
  },                                                              \
  MASK(pin ## _PIN),                                              \
  (PWM_TYPE(pwm, pin) != SOFTWARE_PWM) ?                          \
    (((max_value) * 64 + 12) / 25) :                              \
    (uint16_t)(255UL * 100 / (max_value)),                        \
  PWM_TYPE(pwm, pin),                                             \
  invert ? 1 : 0                                                  \
  },
static const heater_definition_t heaters[NUM_HEATERS] =
{
	#include	"config_wrapper.h"
};
#undef DEFINE_HEATER_ACTUAL

// We test any heater if we need software-pwm
#define DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, ...) \
  | (PWM_TYPE(pwm, pin) == SOFTWARE_PWM)
static const uint8_t software_pwm_needed = 0
  #include "config_wrapper.h"
;
#undef DEFINE_HEATER_ACTUAL


/// \brief initialise heater subsystem
/// Set directions, initialise PWM timers, read PID factors from eeprom, etc
void heater_init() {

	// setup PWM timers: fast PWM
	// Warning 2012-01-11: these are not consistent across all AVRs
	TCCR0A = MASK(WGM01) | MASK(WGM00);
	// PWM frequencies in TCCR0B, see page 108 of the ATmega644 reference.
	TCCR0B = MASK(CS00); // F_CPU / 256 (about 78(62.5) kHz on a 20(16) MHz chip)
	#ifndef FAST_PWM
		TCCR0B = MASK(CS00) | MASK(CS02); // F_CPU / 256 / 1024  (about 76(61) Hz)
	#endif
	TIMSK0 = 0;
	OCR0A = 0;
	OCR0B = 0;

	// timer 1 is used for stepping

  #ifdef TCCR2A
    TCCR2A = MASK(WGM21) | MASK(WGM20);
    // PWM frequencies in TCCR2B, see page 156 of the ATmega644 reference.
    TCCR2B = MASK(CS20); // F_CPU / 256  (about 78(62.5) kHz on a 20(16) MHz chip)
    #ifndef FAST_PWM
      TCCR2B = MASK(CS20) | MASK(CS21) | MASK(CS22); // F_CPU / 256 / 1024
    #endif
    TIMSK2 = 0;
    OCR2A = 0;
    OCR2B = 0;
  #endif

	#ifdef	TCCR3A
		TCCR3A = MASK(WGM30);
		TCCR3B = MASK(WGM32) | MASK(CS30);
		TIMSK3 = 0;
		OCR3A = 0;
		OCR3B = 0;
	#endif

	#ifdef	TCCR4A
		#ifdef TIMER4_IS_10_BIT
			// ATmega16/32U4 fourth timer is a special 10 bit timer
			TCCR4A = MASK(PWM4A) | MASK(PWM4B) ; // enable A and B
			TCCR4C = MASK(PWM4D); // and D
			TCCR4D = MASK(WGM40); // Phase correct
			TCCR4B = MASK(CS40);  // no prescaler
			#ifndef FAST_PWM
				TCCR4B = MASK(CS40) | MASK(CS42) | MASK(CS43); // 16 MHz / 1024 / 256
				//TCCR4B = MASK(CS40) | MASK(CS41) | MASK(CS43); // 16 MHz / 4096 / 256
			#endif
			TC4H   = 0;           // clear high bits
			OCR4C  = 0xff;        // 8 bit max count at top before reset
		#else
			TCCR4A = MASK(WGM40);
			TCCR4B = MASK(WGM42) | MASK(CS40);
		#endif
		TIMSK4 = 0;
		OCR4A = 0;
		OCR4B = 0;
		#ifdef OCR4D
			OCR4D = 0;
		#endif
	#endif

	#ifdef	TCCR5A
		TCCR5A = MASK(WGM50);
		TCCR5B = MASK(WGM52) | MASK(CS50);
		TIMSK5 = 0;
		OCR5A = 0;
		OCR5B = 0;
	#endif

	// setup pins
  #undef DEFINE_HEATER_ACTUAL
  #define DEFINE_HEATER_ACTUAL(name, pin, invert, pwm, ...) \
    if (PWM_TYPE(pwm, pin) == HARDWARE_PWM) {               \
      *pin ## _PWM = (invert) ? 255 : 0;                    \
      pin ## _TCCR |= MASK(pin ## _COM);                    \
    }
  #include "config_wrapper.h"
  #undef DEFINE_HEATER_ACTUAL

  // set all heater pins to output
  #define DEFINE_HEATER_ACTUAL(name, pin, invert, ...)  \
    SET_OUTPUT(pin);                                    \
    WRITE(pin, invert ? 1 : 0);
  #include "config_wrapper.h"
  #undef DEFINE_HEATER_ACTUAL

  pid_init();
}

/** \brief manually set PWM output
	\param index the heater we're setting the output for
	\param value the PWM value to write

	anything done by this function is overwritten by heater_tick above if the heater has an associated temp sensor
*/
void do_heater(heater_t index, uint8_t value) {
  if (index < NUM_HEATERS) {

    if (heaters[index].pwm_type == HARDWARE_PWM) {
      uint8_t pwm_value;

      // Remember, we scale, and the timer inverts already.
      pwm_value = (uint8_t)((heaters[index].max_value * value) / 256);

      *(heaters[index].heater_pwm) = heaters[index].invert ?
        (255 - pwm_value) : pwm_value;

      if (DEBUG_PID && (debug_flags & DEBUG_PID))
        sersendf_P(PSTR("PWM{%u = %u}\n"), index, *heaters[index].heater_pwm);
    }
    else {
      if ((value >= HEATER_THRESHOLD && ! heaters[index].invert) ||
          (value < HEATER_THRESHOLD && heaters[index].invert))
        *(heaters[index].heater_port) |= heaters[index].masked_pin;
      else
        *(heaters[index].heater_port) &= ~heaters[index].masked_pin;
    }

    if (value)
      power_on();
  }
}

#endif /* defined TEACUP_C_INCLUDE && defined __AVR__ */
