
/** \file
  \brief Timer management, ARM specific part.

  To be included from timer.c.
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__


/** Timer initialisation.

  Initialise timer and enable system clock interrupt. Step interrupt is
  enabled later, when we start using it.

  For the system clock, we use SysTickTimer. This timer is made for exactly
  such purposes.
*/
void timer_init() {

/**
  Initialise the system tick timer

  We enable the system tick timer with interrupts. A similar function is
  SysTick_Config(uint32_t ticks) in cmsis-core_cm4.h

  Register name mapping from STM32F4xx

*/
  NVIC_SetPriority(SysTick_IRQn, 0);              // Highest priority.

  // SysTick defined in cmsis-core_cm4.h.
  SysTick->LOAD = TICK_TIME - 1;                  // set reload register
  SysTick->VAL  = 0;                              // Load the SysTick Counter Value

  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk        // Enable the ticker.
                 | SysTick_CTRL_TICKINT_Msk       // Enable interrupt.
                 | SysTick_CTRL_CLKSOURCE_Msk;    // Run at full CPU clock.
}

/** System clock interrupt.

  Happens every TICK_TIME. Must have the same name as in
  cmsis-startup_lpc11xx.s
*/
void SysTick_Handler(void) {

  #ifndef __ARMEL_NOTYET__
  clock_tick();
  dda_clock();
  #endif /* __ARMEL_NOTYET__ */
}

/** Specify how long until the step timer should fire.

  \param delay Delay for the next step interrupt, in CPU ticks.

  \param check_short Tell whether to check for impossibly short requests. This
         should be set to 1 for calls from the step interrupt. Short requests
         then return 1 and do not schedule a timer interrupt. The calling code
         usually wants to handle this case.

         Calls from elsewhere should set it to 0. In this case a timer
         interrupt is always scheduled. At the risk that this scheduling
         doesn't delay the requested time, but up to a full timer counter
         overflow ( = 65536 / F_CPU = 3 to 4 milliseconds).

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
*/
uint8_t timer_set(int32_t delay, uint8_t check_short) {
  return 0;
}

/** Stop timers.

  This means to be an emergency stop.
*/
void timer_stop() {
  SysTick->CTRL = 0;
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
