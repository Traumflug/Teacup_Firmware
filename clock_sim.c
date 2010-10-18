#include <signal.h>
#include <stdlib.h>
#include <sys/time.h>

#include "clock.h"
#include "simulation.h"

static uint8_t clock_counter_250ms = 0;
static uint8_t clock_counter_1s    = 0;
volatile uint8_t clock_flag = 0;

static bool clock_initialised = false;
#define SIM_CLOCK_SLOWDOWN 50

static void timer2_isr(int cause, siginfo_t *HowCome, void *ucontext)
{
	if (!sim_interrupts) return;

	sim_interrupts = false;

	// 1/4 second tick
	if (++clock_counter_250ms == 250 / SIM_CLOCK_SLOWDOWN) {
		clock_flag |= CLOCK_FLAG_250MS;
		clock_counter_250ms = 0;
		if (++clock_counter_1s == 4) {
			clock_flag |= CLOCK_FLAG_1S;
			clock_counter_1s = 0;
		}
	}

	sim_interrupts = true;
}

void clock_setup(void)
{
	struct itimerval itimer;
        struct sigaction sa;
	long unsigned int usec = 1000 * SIM_CLOCK_SLOWDOWN;

	sim_info("clock_setup: simulate timer 2 ISR at %luus", usec);
        sa.sa_sigaction = timer2_isr;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_SIGINFO;
        if (sigaction(SIGPROF, &sa, 0)) {
		sim_error("sigaction");
        }
	
	itimer.it_interval.tv_sec = 0;
	itimer.it_interval.tv_usec = 1000 * SIM_CLOCK_SLOWDOWN;
	itimer.it_value.tv_sec = 0;
	itimer.it_value.tv_usec = 1000 * SIM_CLOCK_SLOWDOWN;
	setitimer(ITIMER_PROF, &itimer, NULL); 

	clock_initialised = true;
}

