
#ifndef _PARALLEL_4BIT_H
#define _PARALLEL_4BIT_H

#include "config_wrapper.h"


enum rs_e {
  parallel_4bit_instruction = 0,
  parallel_4bit_data
};

void parallel_4bit_init(void);
uint8_t parallel_4bit_busy(void);
void parallel_4bit_write(uint8_t data, enum rs_e rs);


#endif /* _PARALLEL_4BIT_H */
