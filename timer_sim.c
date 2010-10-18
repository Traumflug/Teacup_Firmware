#include <signal.h>
#include <stdlib.h>
#include <sys/time.h>

#include "dda_queue.h"
#include "timer.h"
#include "simulation.h"

static bool timer_initialised = false;

void setupTimerInterrupt(void)
{
	disableTimerInterrupt();
	sim_info("setupTimerInterrupt");
	timer_initialised = true;
}

static void timer1_isr(int cause, siginfo_t *HowCome, void *ucontext)
{
	if (!sim_interrupts || !timerInterruptIsEnabled()) return;

	sim_interrupts = false;

	WRITE(SCK, 1);
	queue_step();
	WRITE(SCK, 0);

	sim_interrupts = true;
}

void setTimer(uint32_t delay)
{
	struct itimerval itimer;
        struct sigaction sa;
	
	sim_assert(timer_initialised, "timer not initialised");

        sa.sa_sigaction = timer1_isr;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_SIGINFO;
        if (sigaction(SIGPROF, &sa, 0)) {
		sim_error("sigaction");
        }
	itimer.it_interval.tv_sec = 0;
	itimer.it_interval.tv_usec = (long)delay * 8000000 / F_CPU;
	itimer.it_value.tv_sec = 0;
	itimer.it_value.tv_usec = itimer.it_interval.tv_usec;
	setitimer(ITIMER_PROF, &itimer, NULL); 
}

