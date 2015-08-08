
/** \file
  \brief Manage heaters, including PID and PWM, ARM specific part.
*/

#if defined TEACUP_C_INCLUDE && defined __ARMEL__

/**
  Test configuration.
*/
#ifdef EECONFIG
  #error EEPROM handling (EECONFIG) not yet supported on ARM.
#endif

#ifdef BANG_BANG
  #error BANG_BANG not supported on ARM. You may set PWM frequency of one \
         or all heater(s) to zero, which gives similar, better behaviour.
#endif

/** \def PWM_SCALE

  G-code standard (if such a thing exists at all) gives a heater setting
  range between 0 (off) and 255 (full on), so we let the PWM timers count up
  to 255. Doing so allows to set the prescaler for these frequencies (all on
  a 48 MHz CPU clock):

    prescaler     frequency         prescaler     frequency

            0     188.2 kHz                 4      37.6 kHz
            1      91.1 kHz                 5      31.4 kHz
            2      62.7 kHz               ...       ...
            3      47.0 kHz             65535      2.87 Hz

  As one can see, frequency steps are rather coarse on the high end and
  become finer grained the lower it gets.

  If this range is generally too high for your purposes, you can set PWM_SCALE
  to multiples of 255 to lower the range. Doubling it to 510 moves the
  frequency range to 1.4 Hz...91.1 kHz, quadrupling it to 1020 moves the range
  to 0.7 Hz...46.9 kHz and so on. The highest allowed number is 65535.

  That said, code below calculates the best prescaler value for a configured
  frequency, so you should bother about PWM_SCALE only of you need frequencies
  below 3 Hz.
*/
#define PWM_SCALE 255


/** Initialise heater subsystem.

  Initialise PWM timers, etc. Inspired by pwm.c in LPC1343CodeBase:

    https://github.com/microbuilder/LPC1343CodeBase

  Note that PWM is inversed, pins start at Low by chip design. When the pin's
  counter is reached, they're set to High. Reaching the timer reset counter is
  programmed to reset everything and start over. Thus, having both counter
  matches similar gives a low duty, having the pin counter match zero gives
  full on.

  For simplicity we reset all timer counters always on Match 3 and always
  at PWM_SCALE (255 per default), so setting a pin match to PWM_SCALE / 2
  gives 50% duty, setting it to PWM_SCALE gives full off. This choice disallows
  using a pin connected to a Match 3, working around this would make code much
  more complicated (and still not allow to use more than 3 pins per timer).

  On ARM we can define a PWM frequency pretty fine grained, so we take the
  'pwm' value of DEFINE_HEATER() not only wether to use PWM at all, but also
  to define the PWM frequency. Float values are allowed.

  If there's more than one pin on a timer, they share the same PWM frequency;
  the frequency choosen is the one of the pin defined last.
*/
void heater_init() {
  /**
    Pins on the LPC1114 are usable as following, capture pins not listed:

      pin      timer/match     func for PWM   other uses

      PIO0_1   CT32B0_MAT2     0x2            ---
      PIO0_8   CT16B0_MAT0     0x2            MISO0
      PIO0_9   CT16B0_MAT1     0x2            MOSI0
      PIO0_10  CT16B0_MAT2     0x3            SCK0 (also on PIO0_6)
      PIO0_11  CT32B0_MAT3     0x3            AD0, Step timer
      PIO1_1   CT32B1_MAT0     0x3            AD2
      PIO1_2   CT32B1_MAT1     0x3            AD3
      PIO1_3   CT32B1_MAT2     0x3            AD4
      PIO1_4   CT32B1_MAT3     0x2            AD5, PWM reset
      PIO1_6   CT32B0_MAT0     0x2            RXD, Step timer
      PIO1_7   CT32B0_MAT1     0x2            TXD, Step timer
      PIO1_9   CT16B1_MAT0     0x1            ---
  */
  if (NUM_HEATERS) {                            // At least one channel in use.

    // For simplicity, turn on all timers, not only the used ones.
    // TODO: turn on only the used ones. we know pin ## TIMER.
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7);        // Turn on CT16B0 power.
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);        // Turn on CT16B1 power.
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 10);       // Turn on CT32B1 power.

    // Auto-generate pin setup.
    #undef DEFINE_HEATER
    #define DEFINE_HEATER(name, pin, pwm) \
      LPC_IOCON->pin ## _CMSIS = pin ## _PWM;     /* Connect to timer.   */ \
      /*pin ## _TIMER->IR  = 0; ( = reset value)     No interrupts.      */ \
      pin ## _TIMER->TCR   = (1 << 0);            /* Enable counter.     */ \
      /* TODO: set PWM frequency here according to configuration. */        \
      /* TODO: check wether configured freq. is within range (1..65536). */ \
      pin ## _TIMER->PR    =                      /* Prescaler to        */ \
          F_CPU / PWM_SCALE / 1000 - 1;           /*   1000 Hz.          */ \
      pin ## _TIMER->MCR   = (1 << 10);           /* Reset on Match 3.   */ \
      /* PWM_SCALE - 1, so match = 255 is full off. */                      \
      pin ## _TIMER->MR[3] = PWM_SCALE - 1;       /* Match 3 at 254.     */ \
      /* TODO: Heaters should start at zero, of course. 10% is as demo. */  \
      pin ## _TIMER->MR[pin ## _MATCH] =          /* Match pin = duty.   */ \
          (PWM_SCALE * 0.9);                                                \
      /*pin ## _TIMER->CCR = 0; ( = reset value)     No pin capture.     */ \
      pin ## _TIMER->EMR |= ((1 << pin ## _MATCH) /* Connect to pin.     */ \
          | (0x03 << ((pin ## _MATCH * 2) + 4))); /* Toggle pin on match.*/ \
      /*pin ## _TIMER->CTCR = 0; ( = reset value)    Timer mode.         */ \
      pin ## _TIMER->PWMC |= ((1 << 3)            /* 3 to PWM mode.      */ \
          | (1 << pin ## _MATCH));                /* Pin to PWM mode.    */
    #include "config_wrapper.h"
    #undef DEFINE_HEATER
  } /* NUM_HEATERS */

#if 0
  pid_init(i);
#endif /* 0 */
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
