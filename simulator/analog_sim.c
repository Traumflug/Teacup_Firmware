#include "analog.h"
#include "simulator.h"

static bool analog_initialised = false;

void analog_init(void) {
  sim_info("analog_init: not implemented in simulator");
  analog_initialised = true;
}

uint16_t analog_read(uint8_t channel) {
  sim_assert(analog_initialised, "analog_init() was not called before analog_read()");
  sim_assert(sim_interrupts, "interrupts disabled");
  return 0;
}
