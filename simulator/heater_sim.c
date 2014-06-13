
#include "heater.h"

void heater_init(void) {
}

void heater_set(heater_t index, uint8_t value) {
  sim_info("heater not implemented in simulator.");
}

void heater_tick(heater_t h, temp_type_t type, uint16_t current_temp, uint16_t target_temp) {
}

uint8_t heaters_all_zero(void) {
  return 1;
}

#ifdef EECONFIG
void pid_set_p(heater_t index, int32_t p) {}
void pid_set_i(heater_t index, int32_t i) {}
void pid_set_d(heater_t index, int32_t d) {}
void pid_set_i_limit(heater_t index, int32_t i_limit) {}
void heater_save_settings(void) {
  sim_info("heater not implemented in simulator.");
}
#endif /* EECONFIG */

void heater_print(uint16_t i) {
  sim_info("heater not implemented in simulator.");
}
