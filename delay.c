#include  "delay.h"

/** \file
  \brief Delay routines
*/

#include  <stdint.h>
#include  <util/delay_basic.h>
#include  "watchdog.h"

#if F_CPU < 4000000UL
#error Delay functions only work with F_CPU >= 4000000UL
#endif

/** Delay in microseconds
  \param delay time to wait in microseconds

  Calibrated in SimulAVR.

  Accuracy on 20 MHz CPU clock: -1/+3 clock cycles over the whole range(!).
  Accuracy on 16 MHz CPU clock: delay is about 0.8% too short.

  Exceptions are delays of 0..2 on 20 MHz, which are all 0.75 us and delays
  of 0..3 on 16 MHz, which are all 0.93us.
*/
void delay_us(uint16_t delay) {
  wd_reset();

  // Compensate call overhead, as close as possible.
  #define OVERHEAD_CALL_CLOCKS 39 // clock cycles
  #define OVERHEAD_CALL_DIV ((OVERHEAD_CALL_CLOCKS / (F_CPU / 1000000)) + 1)
  #define OVERHEAD_CALL_REM ((OVERHEAD_CALL_DIV * (F_CPU / 1000000)) - \
                             OVERHEAD_CALL_CLOCKS)

  if (delay > OVERHEAD_CALL_DIV) {
    delay -= OVERHEAD_CALL_DIV;
    if (OVERHEAD_CALL_REM >= 2)
      _delay_loop_2((OVERHEAD_CALL_REM + 2) / 4);
  }
  else {
    return;
  }

  while (delay > (65536L / (F_CPU / 4000000L))) {
    #define OVERHEAD_LOOP_CLOCKS 13

    _delay_loop_2(65536 - (OVERHEAD_LOOP_CLOCKS + 2) / 4);
    delay -= (65536L / (F_CPU / 4000000L));
    wd_reset();
  }
  if (delay)
    _delay_loop_2(delay * (F_CPU / 4000000L));
  wd_reset();
}

/** Delay in microseconds
  \param delay time to wait in milliseconds

  Accuracy on 20 MHz: delay < 0.04% too long over the whole range.
  Accuracy on 16 MHz: delay < 0.8% too short over the whole range.
*/
void delay_ms(uint32_t delay) {
  wd_reset();
  while (delay > 65) {
    delay_us(64999);
    delay -= 65;
    wd_reset();
  }
  delay_us(delay * 1000 - 2);
  wd_reset();
}
