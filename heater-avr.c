
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
	volatile uint8_t *heater_port; ///< pointer to port. DDR is inferred from this pointer too
	uint8_t						heater_pin;  ///< heater pin, not masked. eg for PB3 enter '3' here, or PB3_PIN or similar
  /// Wether the heater pin signal needs to be inverted.
  uint8_t          invert;
	volatile uint8_t *heater_pwm;  ///< pointer to 8-bit PWM register, eg OCR0A (8-bit) or ORC3L (low byte, 16-bit)
} heater_definition_t;

#undef DEFINE_HEATER
/// \brief helper macro to fill heater definition struct from config.h
#define	DEFINE_HEATER(name, pin, invert, pwm) { \
  &(pin ## _WPORT), pin ## _PIN, invert ? 1 : 0, pwm ? (pin ## _PWM) : NULL},
static const heater_definition_t heaters[NUM_HEATERS] =
{
	#include	"config_wrapper.h"
};
#undef DEFINE_HEATER


/// \brief initialise heater subsystem
/// Set directions, initialise PWM timers, read PID factors from eeprom, etc
void heater_init() {
	heater_t i;

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

  /**
    - - TODO - - - TODO - - - TODO - - - TODO - -

    This produces a lot of initialisation code for setting up just 2 or 3
    pins. Can't be optimized out, because heaters[] could have been changed
    since initialisation.

    A much better strategy for this is the one found in heaters-arm.c: use
    the config_wrapper.h magic to initialise only what's needed. The needed
    values are likely already in arduino_xxx.h.
  */

	// setup pins
	for (i = 0; i < NUM_HEATERS; i++) {
		if (heaters[i].heater_pwm) {
			*heaters[i].heater_pwm = heaters[i].invert ? 255 : 0;
			// this is somewhat ugly too, but switch() won't accept pointers for reasons unknown
			switch((uint16_t) heaters[i].heater_pwm) {
				case (uint16_t) &OCR0A:
					TCCR0A |= MASK(COM0A1);
					break;
				case (uint16_t) &OCR0B:
					TCCR0A |= MASK(COM0B1);
					break;
        #ifdef TCCR2A
				case (uint16_t) &OCR2A:
					TCCR2A |= MASK(COM2A1);
					break;
				case (uint16_t) &OCR2B:
					TCCR2A |= MASK(COM2B1);
					break;
        #endif
				#ifdef TCCR3A
				case (uint16_t) &OCR3AL:
					TCCR3A |= MASK(COM3A1);
					break;
				case (uint16_t) &OCR3BL:
					TCCR3A |= MASK(COM3B1);
					break;
				#ifdef COM3C1
				case (uint16_t) &OCR3CL:
					TCCR3A |= MASK(COM3C1);
					break;
				#endif
				#endif
				#ifdef	TCCR4A
					#if defined (OCR4AL)
					case (uint16_t) &OCR4AL:
						TCCR4A |= MASK(COM4A1);
						break;
					case (uint16_t) &OCR4BL:
						TCCR4A |= MASK(COM4B1);
						break;
					case (uint16_t) &OCR4CL:
						TCCR4A |= MASK(COM4C1);
						break;
					#else
					// 10 bit timer
					case (uint16_t) &OCR4A:
						TCCR4A |= MASK(COM4A1);
						break;
					case (uint16_t) &OCR4B:
						TCCR4A |= MASK(COM4B1);
						break;
					#ifdef OCR4D
						case (uint16_t) &OCR4D:
							TCCR4C |= MASK(COM4D1);
							break;
					#endif
					#endif
				#endif
				#ifdef	TCCR5A
				case (uint16_t) &OCR5AL:
					TCCR5A |= MASK(COM5A1);
					break;
				case (uint16_t) &OCR5BL:
					TCCR5A |= MASK(COM5B1);
					break;
				case (uint16_t) &OCR5CL:
					TCCR5A |= MASK(COM5C1);
					break;
				#endif
			}
		}

	}

	// set all heater pins to output
  #undef DEFINE_HEATER
  #define DEFINE_HEATER(name, pin, invert, pwm) \
    SET_OUTPUT(pin);                            \
    WRITE(pin, invert ? 1 : 0);
  #include "config_wrapper.h"
  #undef DEFINE_HEATER

  pid_init();
}

/** \brief manually set PWM output
	\param index the heater we're setting the output for
	\param value the PWM value to write

	anything done by this function is overwritten by heater_tick above if the heater has an associated temp sensor
*/
void heater_set(heater_t index, uint8_t value) {
	if (index >= NUM_HEATERS)
		return;

	heaters_runtime[index].heater_output = value;

	if (heaters[index].heater_pwm) {
    *(heaters[index].heater_pwm) = heaters[index].invert ?
      (255 - value) : value;

		if (DEBUG_PID && (debug_flags & DEBUG_PID))
			sersendf_P(PSTR("PWM{%u = %u}\n"), index, *heaters[index].heater_pwm);
	}
	else {
    if ((value >= HEATER_THRESHOLD && ! heaters[index].invert) ||
        (value < HEATER_THRESHOLD && heaters[index].invert))
			*(heaters[index].heater_port) |= MASK(heaters[index].heater_pin);
		else
			*(heaters[index].heater_port) &= ~MASK(heaters[index].heater_pin);
	}

  if (value)
    power_on();
}

#endif /* defined TEACUP_C_INCLUDE && defined __AVR__ */
