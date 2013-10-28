#include <unistd.h>

#include "delay.h"
#include "simulator.h"

void delay_ms(uint32_t ms) {
  usleep(ms * 1000);
}

void delay_us(uint16_t us) {
  usleep(us);
}
