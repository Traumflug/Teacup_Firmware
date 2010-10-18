#include <unistd.h>

#include "delay.h"
#include "simulation.h"

void delay(uint32_t us)
{
	usleep(us);
}

void delay_ms(uint32_t ms)
{
	usleep(ms * 1000);
}

