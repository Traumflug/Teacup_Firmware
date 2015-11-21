
/** \file
  \brief Timer management, ARM specific part.

  To be included from timer.c.
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__

#include "cmsis-core_cm4.h"
#include "clock.h"
#include "pinio.h"
#include "dda_queue.h"

/** Timer initialisation.

  Initialise timer and enable system clock interrupt. Step interrupt is
  enabled later, when we start using it.

  For the system clock, we use SysTickTimer. This timer is made for exactly
  such purposes.

  Other than AVRs, Cortex-M doesn't allow reentry of interupts. To deal with
  this we use another interrupt, PendSV, with slightly lower priority and
  without an interrupt source. The possibly lengthy operation in dda_clock()
  goes into there and at the end of the SysTick interrupt we simply set PendSV
  pending. This way PendSV is executed right after SysTick or, if PendSV is
  already running, ignored.
*/
void timer_init() {

/**
  Initialise the system tick timer

  We enable the system tick timer with interrupts. A similar function is
  SysTick_Config(uint32_t ticks) in cmsis-core_cm4.h

  Register name mapping from STM32F4xx

    SYST_CSR       SysTick->CTRL     System Timer Control and status register.
    SYST_RVR       SysTick->LOAD     System Timer Reload value register.
    SYST_CVR       SysTick->VAL      System Timer Current value register.
    SYST_CALIB     SysTick->CALIB    System Timer Calibration value register.
*/

  NVIC_SetPriorityGrouping(1);
  NVIC_SetPriority(SysTick_IRQn,
  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));

  //NVIC_SetPriority(SysTick_IRQn, 2);              // Highest priority.

  // SysTick defined in cmsis-core_cm4.h.
  SysTick->LOAD = TICK_TIME - 1;                  // set reload register
  SysTick->VAL  = 0;                              // Load the SysTick Counter Value

  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk        // Enable the ticker.
                 | SysTick_CTRL_TICKINT_Msk       // Enable interrupt.
                 | SysTick_CTRL_CLKSOURCE_Msk;    // Run at full CPU clock.

  /**
    Initialise PendSV for dda_clock().
  */
  NVIC_SetPriority(PendSV_IRQn,
  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));               // Almost highest priority.

  /**
    Initialise the stepper timer. On ARM we have the comfort of hardware
    32-bit timers, so we don't have to artifically extend the timer to this
    size. We use match register 1 of the first 32-bit timer, TIM5.

    We run the timer all the time, just turn the interrupt on and off, to
    allow interrupts equally spaced, independent from processing time. See
    also description of timer_set().
  */
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;             // Turn on TIM5 power.

  TIM5->CR1          &= ~(0x03FF);                // clear register
  TIM5->CR1          |= TIM_CR1_CEN;              // Enable counter.

  NVIC_SetPriority(TIM5_IRQn,
  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));           // Also highest priority.
  NVIC_EnableIRQ(TIM5_IRQn);                // Enable interrupt generally.
}

/** System clock interrupt.

  Happens every TICK_TIME. Must have the same name as in
  cmsis-startup_stm32f411xe.s
*/
void SysTick_Handler(void) {

  clock_tick();

  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;             // Trigger PendSV_Handler().

}

/** System clock interrupt, slow part.

  Here we do potentially lengthy calculations. Must have the same name as in
  cmsis-startup_stm32f411xe.s
*/
void PendSV_Handler(void) {
  dda_clock();
}

/** Step interrupt.

  Happens for every stepper motor step. Must have the same name as in
  cmsis-startup_stm32f411xe.s

  TIM2 and TIM5 are 32bit timers. We use TIM5, because depending pins have
  an alternative timer for pwm.
*/
void TIM5_IRQHandler(void) {

  #ifdef DEBUG_LED_PIN
    WRITE(DEBUG_LED_PIN, 1);
  #endif

  /**
    Turn off interrupt generation, timer counter continues. As this interrupt
    is the only use of this timer, there's no need for a read-modify-write.
  */
  TIM5->DIER = 0;

  /**
    We have to "reset" this interrupt, else it'll be triggered over and over
    again.
  */
  TIM5->SR = 0;

  queue_step();

  #ifdef DEBUG_LED_PIN
    WRITE(DEBUG_LED_PIN, 0);
  #endif
}

/** Specify how long until the step timer should fire.

  \param delay Delay for the next step interrupt, in CPU ticks.

  \param check_short Tell whether to check for impossibly short requests. This
         should be set to 1 for calls from the step interrupt. Short requests
         then return 1 and do not schedule a timer interrupt. The calling code
         usually wants to handle this case.

         Calls from elsewhere should set it to 0. In this case a timer
         interrupt is always scheduled.

  \return A flag whether the requested time was too short to allow scheduling
          an interrupt. This is meaningful for ACCELERATION_TEMPORAL, where
          requested delays can be zero or even negative. In this case, the
          calling code should repeat the stepping code immediately and also
          assume the timer to not change his idea of when the last step
          happened.

  Strategy of this timer is to schedule timer interrupts not starting at the
  time of the call, but starting at the time of the previous timer interrupt
  fired. This ignores the processing time taken in the step interrupt so far,
  offering smooth and even step distribution. Flipside of this coin is,
  schedules issued at an arbitrary time can result in drastically wrong delays.
  See also discussion of parameter check_short and the return value.

  This enables the step interrupt, but also disables interrupts globally.
  So, if you use it from inside the step interrupt, make sure to do so
  as late as possible. If you use it from outside the step interrupt,
  do a sei() after it to make the interrupt actually fire.

  On ARM we have the comfort of hardware 32-bit timers, so we don't have to
  artifically extend the timer to this size. We use match register 1 of the
  first 32-bit timer, TIM5.
*/
uint8_t timer_set(int32_t delay, uint8_t check_short) {
  #ifdef ACCELERATION_TEMPORAL
    if (check_short) {
      /**
        100 = safe number of cpu cycles after current_time to allow a new
        interrupt happening. This is mostly the time needed to complete the
        current interrupt.

        TIM5->CNT         = timer counter = current time.
        TIM5->CCR1        = last capture compare = time of the last step.
      */
      if ((TIM5->CNT - TIM5->CCR1) + 100 > delay)
        return 1;
    }
  #endif /* ACCELERATION_TEMPORAL */

  /**
    Still here? Then we can schedule the next step. Off of the previous step.
    If there is no previous step, CNT and CCR1 should have been reset to zero
    by calling timer_reset() shortly before we arrive here.
  */
  TIM5->CCR1 += delay;

  /**
    Turn on the stepper interrupt. As this interrupt is the only use of this
    timer, there's no need for a read-modify-write.
  */
  TIM5->DIER = TIM_DIER_CC1IE;                    // Interrupt on MR0 match.

  return 0;
}

/** Timer reset.

  Reset the timer, so step interrupts scheduled at an arbitrary point in time
  don't lead to a full round through the timer counter.

  On ARM we actually do something, such a full round through the timer is
  2^32 / F_CPU = 22 to 44 seconds.
*/
void timer_reset() {
  TIM5->CNT = 0;
  TIM5->CCR1 = 0;
}

/** Stop timers.

  This means to be an emergency stop.
*/
void timer_stop() {
  SysTick->CTRL = 0;
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
