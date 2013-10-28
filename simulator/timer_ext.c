#include <signal.h>
#include <stdlib.h>
#include <sys/time.h>

#include "dda_queue.h"
#include "timer.h"
#include "simulator.h"
#include <time.h>
#include <stdio.h> // printf
#include <unistd.h> // usleep

#define TIME_SLOW_FACTOR 1

static void schedule_timer(uint32_t useconds);

static bool timer_initialised = false;

enum {
  // One bit per timer
  TIMER_OCR1A = 0x1 ,
  TIMER_OCR1B = 0x2 ,
};

static volatile uint8_t timer_reason;  // Who scheduled this timer

static uint64_t now_us(void) {
  struct timespec tv;

  int n = clock_gettime(CLOCK_MONOTONIC, &tv);
  sim_assert(n == 0, "clock_gettime failed");

  // Convert tv to nanoseconds.
  uint64_t nsec = tv.tv_sec;
  nsec *= 1000 * 1000 * 1000;
  nsec += tv.tv_nsec;

  // nanoseconds to microseconds
  return nsec / 1000;
}

uint16_t sim_tick_counter(void) {
  // microseconds to 16-bit clock ticks
  return (now_us() / TIME_SLOW_FACTOR) US;
}

extern uint8_t clock_counter_10ms, clock_counter_250ms, clock_counter_1s;
static uint64_t begin;
static uint64_t then;
void sim_timer_init(void) {
  then = begin = now_us();
  sim_info("timer_init");
  timer_initialised = true;

  sim_setTimer();
}

void sim_timer_stop(void) {
  sim_info("timer_stop");
  timer_reason = 0;  // Cancel pending timer;
}

static void timer1_isr(int cause, siginfo_t *HowCome, void *ucontext) {
  if ( ! sim_interrupts) {
    // Interrupts disabled. Schedule another callback in 10us.
    schedule_timer(10);
    return;
  }

  sim_interrupts = false;

  #ifdef SIM_DEBUG
    uint64_t now = now_us();
    static unsigned int cc_1s = 0, prev_1s = 0;

    if ( ! clock_counter_1s && prev_1s) ++cc_1s;
    prev_1s = clock_counter_1s;

    //uint16_t now = sim_tick_counter();
    uint64_t real = now-begin;
    uint64_t avr = cc_1s * 4 + clock_counter_1s;
    avr *= 250;
    avr += clock_counter_250ms * 10;
    avr += clock_counter_10ms;
    avr *= 1000 ;
    printf("test: Real: %us %u.%03ums   AVR: %us %u.%03ums    Real/AVR: %u\n",
           real / 1000000 , (real % 1000000)/1000 , real % 1000 ,
           avr / 1000000  , (avr  % 1000000)/1000 , avr  % 1000 ,
           real / (avr?avr:1) );
    printf("test: 10ms=%u 250ms=%u 1s=%u  total=%lu actual=%lu\n",
           clock_counter_10ms, clock_counter_250ms, clock_counter_1s,
           now - begin , now - then);
    //printf("          timer1_isr    tick_time=%04X  now=%04X  delta=%u  total=%u\n",
    //       TICK_TIME , now, now_us() - then, (now_us() - begin)/1000000 ) ;
    then = now;
  #endif

  if (timer_reason & TIMER_OCR1A) TIMER1_COMPA_vect();
  if (timer_reason & TIMER_OCR1B) TIMER1_COMPB_vect();
  timer_reason = 0;

  sim_interrupts = true;

  // Setup next timer
  sim_setTimer();
}

// TODO: Remove 'delay' value and use AVR regs instead
void sim_setTimer() {
  // Set callbacks for COMPA and COMPB timers
  uint32_t nextA = 0, nextB = 0;
  uint16_t now = sim_tick_counter();

  sim_assert(timer_initialised, "timer not initialised");

  //-- Calculate time in clock ticks until next timer events
  if (TIMSK1 & MASK(OCIE1A)) {
    sim_debug("Timer1 Interrupt A: Enabled");
    nextA = OCR1A - now;
    // 0 = No timer;  1-0x10000 = time until next occurrence
    if ( ! nextA) nextA = 0x10000;
  }
  if (TIMSK1 & MASK(OCIE1B)) {
    sim_debug("Timer1 Interrupt B: Enabled");
    nextB = OCR1B - now;
    // 0 = No timer;  1-0x10000 = time until next occurrence
    if ( ! nextB) nextB = 0x10000;
  }

  //-- Find the nearest event
  uint32_t next = nextA;
  if (nextB && ( ! next || (nextB < next))) {
    next = nextB;
    timer_reason = TIMER_OCR1B;
  }

  //-- Flag the reasons for the next event
  timer_reason = 0;
  if (next == nextA) timer_reason |= TIMER_OCR1A;
  if (next == nextB) timer_reason |= TIMER_OCR1B;

  // FIXME: We will miss events if they occur like this:
  //    nextA = 0x1000
  //    nextB = 0x1001
  //    timer_reason = TIMER_OCR1A
  //    ISR is triggered and finishes at 0x1002
  //      => Next trigger for B will not occur until NEXT 0x1001 comes around
  //    Need some way to notice a missed trigger.
  //    Maybe store 32-bit tick value for next trigger time for each timer.

  //-- Convert ticks to microseconds
  long actual = ((unsigned long)next) * TIME_SLOW_FACTOR / (1 US);
  if (next) {
    sim_debug("OCR1A:%04X   OCR1B:%04X    now=%04X", OCR1A, OCR1B, now );
    sim_debug("              next=%u   real=%u", next, actual);
  }

  //-- Schedule the event
  schedule_timer(actual);
}

// Schedule Timer1 callback useconds from now.
// Zero cancels any pending timer.
static void schedule_timer(uint32_t useconds) {
  struct itimerval itimer;
  struct sigaction sa;

  sa.sa_sigaction = timer1_isr;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_SIGINFO;
  if (sigaction(SIGALRM, &sa, 0)) {
    sim_error("sigaction");
  }
  itimer.it_interval.tv_sec = 0;
  itimer.it_interval.tv_usec = 0;  // If signal occurs , trigger again in 10us
  itimer.it_value.tv_sec = useconds / 1000000;
  itimer.it_value.tv_usec = useconds % 1000000;
  setitimer(ITIMER_REAL, &itimer, NULL);
}
