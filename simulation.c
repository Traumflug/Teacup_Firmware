#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "simulation.h"

uint8_t ACSR;
uint8_t TIMSK1;


/* -- debugging ------------------------------------------------------------ */

void sim_info(const char fmt[], ...)
{
	va_list ap;
	fputs("\033[0;32m" , stdout);
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	fputs("\033[m\n", stdout);
}

void sim_error(const char msg[])
{
	printf("\033[0;31mERROR: %s\033[m\n", msg);
	exit(-1);
}

void sim_assert(bool cond, const char msg[])
{
	if (!cond)
	{
		sim_error(msg);
	}
}


/* -- interrupts ----------------------------------------------------------- */

volatile bool sim_interrupts = false;
void sei(void)
{
	sim_interrupts = true;
}


/* -- PIN I/O ------------------------------------------------------------ */

#define out true
#define in  false

static int x = 0, y = 0, z = 0, e = 0;

static bool direction[PIN_NB];
static bool state[PIN_NB];

static void print_pos(void)
{
	sim_info("x:%5d       y:%5d       z:%5d       e:%5d", x, y, z, e);
}

void WRITE(pin_t pin, bool s)
{
	bool old_state = state[pin];

	if (direction[pin] == out)
	{
		state[pin] = s;
	}
	if (s && !old_state) /* rising edge */
	{
		switch (pin)
		{
		case X_STEP_PIN:
			x += state[X_DIR_PIN] ? 1 : -1;
			print_pos();
			break;
		case Y_STEP_PIN:
			y += state[Y_DIR_PIN] ? 1 : -1;
			print_pos();
			break;
		case Z_STEP_PIN:
			z += state[Z_DIR_PIN] ? 1 : -1;
			print_pos();
			break;
		case E_STEP_PIN:
			e += state[E_DIR_PIN] ? 1 : -1;
			print_pos();
			break;
		default:
			break;
		}
	}
}

void SET_OUTPUT(pin_t pin)
{
	direction[pin] = out;	
}

void SET_INPUT(pin_t pin)
{
	direction[pin] = in;
}

