#ifdef SIMULATOR

#include <unistd.h>

#include "delay.h"
#include "simulator.h"

void delay_us(uint16_t us) {
  usleep(us);
}

#endif
