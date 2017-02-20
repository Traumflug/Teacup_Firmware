#include "temp.h"
#include "analog.h"
#include "simulator.h"
#include <stdio.h>

static bool analog_initialised = false;

void analog_init(void) {
  analog_initialised = true;
}

static uint16_t analog_read_value = 0;
uint16_t analog_read(uint8_t channel) {
  sim_assert(analog_initialised, "analog_init() was not called before analog_read()");
  sim_assert(sim_interrupts, "interrupts disabled");
  return analog_read_value;
}

void sim_report_temptables(int sensor) {
  int i ;
  temp_sensor_t s, first = sensor, last = sensor+1 ;

  // sensor is a specific sensor or -1 for "all sensors"
  if (sensor == -1) {
    first = 0;
    last = NUM_TEMP_SENSORS;
  }

  sei();
  analog_init();
  printf("; Temperature sensor test %d\n", sensor);
  for (s = first; s < last; s++ ) {
    printf("; Sensor %d\n", s);
    for (i = 0 ; i < 1024 ; i++ ) {
      analog_read_value = i ;
      temp_sensor_tick() ;
      uint16_t temp = temp_get(s);
      printf("%d %.2f\n", i, ((float)temp)/4 ) ;
    }
  }
}
