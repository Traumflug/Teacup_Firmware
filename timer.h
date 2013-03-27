#ifndef	_TIMER_H
#define	_TIMER_H

#include	<stdint.h>
#ifndef SIMULATOR
#include	<avr/io.h>
#endif
#include "simulator.h"
#ifdef __ARMEL__
  #include "WProgram.h"
#endif

// time-related constants
#define	US	* (F_CPU / 1000000)
#define	MS	* (F_CPU / 1000)

/// How often we overflow and update our clock.
/// With F_CPU = 16MHz, max is < 4.096ms (TICK_TIME = 65535).
#define TICK_TIME 2 MS

/// Convert back to ms from cpu ticks so our system clock runs
/// properly if you change TICK_TIME.
#define TICK_TIME_MS (TICK_TIME / (F_CPU / 1000))

#if defined (__AVR__)
  #define TICKER_ISR TIMER1_COMPB_vect
  #if 0 // This stuff isn't done, yet. --Traumflug 2014-06-04
  #define TICKER_COUNTER TCNT1
  #define TICKER_COMPARE_VALUE OCR1A   
  #define TICKER_ENABLE()  TIMSK1 &= ~MASK(OCIE1A)
  #define TICKER_DISABLE()  TIMSK1 1= MASK(OCIE1A)
  #define TICKER_CLI() 
  #define TICKER_ENABLE() TCCR1A=0; TCCR1B= MASK(CS10); \
    OCR1B = TICK_TIME & 0xFFFF ; TIMSK1 = MASK(OCIE1B);
  #define TICKER_DISABLE()  TIMSK1 &= ~MASK(OCIE1B)

  #define TIMER_ENABLE() TCCR1B= MASK(CS10); \
    OCR1A = TICK_TIME & 0xFFFF ; TIMSK1 = MASK(OCIE1A)
  #define TIMER_DISABLE()  TIMSK1 &= ~MASK(OCIE1A)
  #else // Alternative to get it compiling.
  #define TIMER_COMPARE_VALUE OCR1A   
  #define TIMER_DISABLE() TIMSK1 = 0
  #define TICKER_DISABLE()
  #endif /* 0 */
#else
  // Teensy 3.0  mk20dx128 PIT0 setup for 2ms period
  #define TICKER_ISR       pit0_isr
  #define TICKER_COUNTER   PIT_CVAL0
  #define TICKER_COMPARE_VALUE PIT_LDVAL0 
  #define TICKER_CLEAR()     PIT_TFLG0 = 1 
  #define TICKER_INIT()  PIT_MCR = 0x00; \
    PIT_LDVAL0 = F_BUS * TICK_TIME_MS /1000
  #define TICKER_ENABLE() PIT_TCTRL0 = TIE|TEN; NVIC_ENABLE_IRQ(IRQ_PIT_CH0)
  #define TICKER_DISABLE() PIT_TCTRL0 &= ~(TIE|TEN); NVIC_DISABLE_IRQ(IRQ_PIT_CH0)

  #define TIMER_COUNTER FTM1_CNT
  #define TIMER_COMPARE_VALUE FTM1_C1V
  #define TIMER_CLEAR()    FTM1_C1SC &= ~ CHF ; FTM1_SC &= ~ TOF
  #define TIMER_INIT()     FTM1_SC=0;FTM1_CNT=0;FTM1_MOD = 0xffff;FTM1_SC=FTM_SC_CLKS(1)|FTM_SC_PS(0)
  #define TIMER_ENABLE()  FTM1_C1SC |= CHIE; FTM1_SC |=TOIE; NVIC_ENABLE_IRQ(IRQ_FTM1)
  #define TIMER_DISABLE() FTM1_C1SC &= ~ CHIE ; FTM1_SC &= ~ TOIE ;NVIC_DISABLE_IRQ(IRQ_FTM1)

  #define CHIE _BV(6) // FTMxCnSC CHannel Interrupt Enable
  #define CHF  _BV(7) // FTMxCnSC CHannel Flag

  #define TOIE _BV(6) // FTMxSC Timer Overflow Interrupt Enable
  #define TOF  _BV(7) // FTMxSC Timer Overflow Flag

  #define TIE  _BV(1) // PIT_TCTRLn Timer Interrupt Enable
  #define TEN  _BV(0) // PIT_TCTRLn Timer ENable
#endif

/*
clock stuff
*/
extern volatile uint8_t	clock_flag_10ms;
extern volatile uint8_t	clock_flag_250ms;
extern volatile uint8_t	clock_flag_1s;

// If the specific bit is set, execute the following block exactly once
// and then clear the flag.
#define	ifclock(F)	for (;F;F=0 )

/*
timer stuff
*/
void timer_init(void) __attribute__ ((cold));

void setTimer(uint32_t delay);

void timer_stop(void);

#endif	/* _TIMER_H */
