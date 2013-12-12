#include <signal.h>
#include <stdlib.h>
#include <sys/time.h>

#include "dda_queue.h"
#include "timer.h"
#include "simulator.h"
#ifdef __MACH__
  #include <mach/mach_time.h>
  #define CLOCK_REALTIME 0
  #define CLOCK_MONOTONIC 0
#else
  #include <time.h>
#endif
#include <stdio.h> // printf
#include <unistd.h> // usleep

#define TICKS_TO_US(t) (t / (F_CPU / 1000000))

static uint16_t time_scale = 1;

static void schedule_timer(uint32_t useconds);
static void timer1_isr(void);

static bool timer_initialised = false;

enum {
  // One bit per timer
  TIMER_OCR1A = 0x1 ,
  TIMER_OCR1B = 0x2 ,
};

static volatile uint8_t timer_reason;  // Who scheduled this timer
static uint64_t ticks;
static uint32_t warpTarget;

#ifdef __MACH__
int clock_gettime(int clk_id, struct timespec *t) {
  mach_timebase_info_data_t timebase;
  mach_timebase_info(&timebase);
  uint64_t time;

  time = mach_absolute_time();
  double nseconds = ((double)time * (double)timebase.numer)/((double)timebase.denom);
  double seconds = ((double)time * (double)timebase.numer)/((double)timebase.denom * 1e9);
  t->tv_sec = seconds;
  t->tv_nsec = nseconds;
  return 0;
}
#endif

static uint64_t now_ns(void) {
  struct timespec tv;

  int n = clock_gettime(CLOCK_MONOTONIC, &tv);
  sim_assert(n == 0, "clock_gettime failed");

  // Convert tv to nanoseconds.
  uint64_t nsec = tv.tv_sec;
  nsec *= 1000 * 1000 * 1000;
  nsec += tv.tv_nsec;
  return nsec;
}

static uint64_t now_us(void) {
  // nanoseconds to microseconds
  return now_ns() / 1000;
}

void sim_time_warp(void) {
  if (time_scale || timer_reason == 0)
    return;

  ticks += warpTarget;
  warpTarget = 0;

  timer1_isr();
}

uint16_t sim_tick_counter(void) {
  if (time_scale) {
    // microseconds to 16-bit clock ticks
    return (now_us() / time_scale) US;
  }
  return (uint16_t)(ticks % 0xFFFF);
}

#ifdef SIM_DEBUG
extern uint8_t clock_counter_10ms, clock_counter_250ms, clock_counter_1s;
#endif

static uint64_t begin;
static uint64_t then;
void sim_timer_init(uint8_t scale) {
  time_scale = scale;
  then = begin = now_ns();
  if (scale==0)
    sim_info("timer_init: warp-speed");
  else if (scale==1)
    sim_info("timer_init: real-time");
  else
    sim_info("timer_init: 1/%u time", scale);
  timer_initialised = true;
}

void sim_timer_stop(void) {
  sim_info("timer_stop");
  timer_reason = 0;  // Cancel pending timer;
}

uint64_t sim_runtime_ns(void) {
  if (time_scale)
    return (now_ns() - begin) / time_scale;
  return TICKS_TO_US(ticks) * 1000 ;
}

static void timer1_callback(int cause, siginfo_t *HowCome, void *ucontext) {
  timer1_isr();
}

static void timer1_isr(void) {
  const uint8_t tr = timer_reason;
  if ( ! sim_interrupts) {
    // Interrupts disabled. Schedule another callback in 10us.
    schedule_timer(10);
    return;
  }
  timer_reason = 0;

  cli();

  #ifdef SIM_DEBUG
    uint64_t now = now_ns();
    static unsigned int cc_1s = 0, prev_1s = 0;

    if ( ! clock_counter_1s && prev_1s) ++cc_1s;
    prev_1s = clock_counter_1s;

    //uint16_t now = sim_tick_counter();
    uint64_t real = (now-begin) / 1000;
    uint64_t avr = cc_1s * 4 + clock_counter_1s;
    avr *= 250;
    avr += clock_counter_250ms * 10;
    avr += clock_counter_10ms;
    avr *= 1000 ;
    printf("test: Real: %us %u.%03ums   AVR: %us %u.%03ums    Real/AVR: %u\n",
           real / 1000000 , (real % 1000000)/1000 , real % 1000 ,
           avr / 1000000  , (avr  % 1000000)/1000 , avr  % 1000 ,
           real / (avr?avr:1) );
    printf("test: 10ms=%u 250ms=%u 1s=%u  total=%luns actual=%luns\n",
           clock_counter_10ms, clock_counter_250ms, clock_counter_1s,
           now - begin, now - then, sim_runtime_ns());
    //printf("          timer1_isr    tick_time=%04X  now=%04X  delta=%u  total=%u\n",
    //       TICK_TIME , now, now_us() - then, (now_us() - begin)/1000000 ) ;
    then = now;
  #endif

  if (tr & TIMER_OCR1A) TIMER1_COMPA_vect();
  if (tr & TIMER_OCR1B) TIMER1_COMPB_vect();

  sei();

  // Setup next timer
  sim_setTimer();
}

void sim_setTimer() {
  // Set callbacks for COMPA and COMPB timers
  uint32_t nextA = 0, nextB = 0;
  uint16_t now = sim_tick_counter();

  sim_assert(timer_initialised, "timer not initialised");

  //-- Calculate time in clock ticks until next timer events
  if (TIMSK1 & MASK(OCIE1A)) {
    sim_debug("Timer1 Interrupt A: Enabled");
    nextA = (OCR1A - now) & 0xFFFF ;
    // 0 = No timer;  1-0x10000 = time until next occurrence
    if ( ! nextA) nextA = 0x10000;
  }

  if (TIMSK1 & MASK(OCIE1B)) {
    sim_debug("Timer1 Interrupt B: Enabled");
    nextB = (OCR1B - now) & 0xFFFF;
    // 0 = No timer;  1-0x10000 = time until next occurrence
    if ( ! nextB) nextB = 0x10000;
  }

  //-- Find the nearest event
  uint32_t next = nextA;
  if (nextB && ( ! next || (nextB < next)))
    next = nextB;

  //-- Flag the reasons for the next event
  timer_reason = 0;
  if (next && next == nextA) timer_reason |= TIMER_OCR1A;
  if (next && next == nextB) timer_reason |= TIMER_OCR1B;

  warpTarget = next ;

  if (time_scale) {
    // FIXME: We will miss events if they occur like this:
    //    nextA = 0x1000
    //    nextB = 0x1001
    //    timer_reason = TIMER_OCR1A
    //    ISR is triggered and finishes at 0x1002
    //      => Next trigger for B will not occur until NEXT 0x1001 comes around
    //    Need some way to notice a missed trigger.
    //    Maybe store 32-bit tick value for next trigger time for each timer.

    //-- Convert ticks to microseconds
    long actual = ((unsigned long)next) * time_scale / (1 US);
    if ( next && !actual)
      actual++;


    if (next) {
      sim_debug("OCR1A:%04X   OCR1B:%04X    now=%04X", OCR1A, OCR1B, now );
      sim_debug("              next=%u   real=%u", next, actual);
    }

    //-- Schedule the event
    schedule_timer(actual);
  }
}

// Schedule Timer1 callback useconds from now.
// Zero cancels any pending timer.
static void schedule_timer(uint32_t useconds) {
  struct itimerval itimer;
  struct sigaction sa;

  if (time_scale) {
    sa.sa_sigaction = timer1_callback;
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
}
