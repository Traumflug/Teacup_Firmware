#include "intercom.h"

/* Intercom stubs here to appease compiler for build testing.
 * Functionally moot.
 */

void intercom_init(void) {
  sim_error("simulated intercom is not supported");
}

void send_temperature(uint8_t index, uint16_t temperature) {
}

uint16_t read_temperature(uint8_t index) {
  return 0;
}

#ifdef MOTHERBOARD
void set_dio(uint8_t index, uint8_t value) {
}
#else
uint8_t get_dio(uint8_t index) {
  return 0;
}
#endif

void set_err(uint8_t err) {
}

uint8_t get_err() {
  return 0;
}

void start_send(void) {
}
