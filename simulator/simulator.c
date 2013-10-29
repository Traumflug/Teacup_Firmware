#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "simulator.h"
#include "data_recorder.h"

uint8_t ACSR;
uint8_t TIMSK1;
uint16_t
  TCCR0A,
  TCCR0B,
  TCCR1A,
  TCCR1B,
  TCCR2A,
  TCCR2B,
  OCR0B,
  OCR1A,
  OCR1B,
  OCR2A,
  OCR2B,
  TIMSK0,
  TIMSK2;

volatile uint8_t
  DIO1_WPORT,
  DIO2_WPORT,
  DIO3_WPORT,
  DIO4_WPORT;

int g_argc;
char** g_argv;
void sim_start(int argc, char** argv) {
  // TODO: Parse args here and open the serial port instead of saving them
  //       for serial_init.
  // Save these for the serial_init code
  g_argc = argc;
  g_argv = argv;

  recorder_init("datalog.out");

  // Record pin names in datalog
#define NAME_PIN(x) add_trace_var(#x , x);
  NAME_PIN(X_STEP_PIN);
  NAME_PIN(X_DIR_PIN);
  NAME_PIN(X_MIN_PIN);
  NAME_PIN(X_ENABLE_PIN);
  NAME_PIN(Y_STEP_PIN);
  NAME_PIN(Y_DIR_PIN);
  NAME_PIN(Y_MIN_PIN);
  NAME_PIN(Y_ENABLE_PIN);
  NAME_PIN(Z_STEP_PIN);
  NAME_PIN(Z_DIR_PIN);
  NAME_PIN(Z_MIN_PIN);
  NAME_PIN(Z_ENABLE_PIN);
  NAME_PIN(E_STEP_PIN);
  NAME_PIN(E_DIR_PIN);
  NAME_PIN(E_ENABLE_PIN);

  NAME_PIN(STEPPER_ENABLE_PIN);

  NAME_PIN(SCK);
  NAME_PIN(MOSI);
  NAME_PIN(MISO);
  NAME_PIN(SS);
}

/* -- debugging ------------------------------------------------------------ */

static void fgreen(void) { fputs("\033[0;32m" , stdout); }
static void fred(void)   { fputs("\033[0;31m" , stdout); }
static void fcyan(void)  { fputs("\033[0;36m" , stdout); }
static void fbreset(void) { fputs("\033[m" , stdout); }

static void bred(void)   { fputs("\033[0;41m" , stdout); }

void sim_info(const char fmt[], ...) {
  va_list ap;
  fgreen();
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  fputc('\n', stdout);
  fbreset();
}

void sim_debug(const char fmt[], ...) {
#ifdef SIM_DEBUG
  va_list ap;
  fcyan();
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  fputc('\n', stdout);
  fbreset();
#endif
}

void sim_tick(char ch) {
  fcyan();
  fprintf(stdout, "%c", ch);
  fbreset();
  fflush(stdout);
}

void sim_error(const char msg[]) {
  fred();
  printf("ERROR: %s\n", msg);
  fputc('\n', stdout);
  fbreset();
  exit(-1);
}

void sim_assert(bool cond, const char msg[]) {
  if (!cond) {
    sim_error(msg);
  }
}


/* -- interrupts ----------------------------------------------------------- */

volatile bool sim_interrupts = false;

void sei(void) {
  sim_interrupts = true;
}

void cli(void) {
  sim_interrupts = false;
}


/* -- PIN I/O ------------------------------------------------------------ */

#define out true
#define in  false

static int x = 0, y = 0, z = 0, e = 0;

static bool direction[PIN_NB];
static bool state[PIN_NB];

static void print_pos(void) {
  printf("print_pos: %d, %d, %d, %d\n", x, y, z, e);
  sim_info("x:%5d    y:%5d    z:%5d    e:%5d", x, y, z, e);
}

bool READ(pin_t pin) {
  sim_assert(pin < PIN_NB, "READ: Pin number out of range");
  // Add any necessary reactive pin-handlers here.
  return state[pin];
}

void WRITE(pin_t pin, bool s) {
  bool old_state = state[pin];
  sim_assert(pin < PIN_NB, "WRITE: Pin number out of range");

  if (direction[pin] == out) {
    state[pin] = s;
  }

  if (old_state != s) {
    uint32_t useconds = (uint32_t)sim_runtime_ns();
    record_pin(pin, s, useconds);
    #ifdef TRACE_ALL_PINS
      fgreen();
      for (int i = 0; i < PIN_NB; i++) {
        if (state[i]) bred(); else bblack();
        fputc('A' + i, stdout);
      }
      fbreset();
      printf("\n");
    #else
      bred();
      if (s)
        sim_tick('A' + pin);
      else
        sim_tick('a' + pin);
      fbreset();
    #endif
  }

  if (s && !old_state) { /* rising edge */
    switch (pin) {
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

void SET_OUTPUT(pin_t pin) {
  sim_assert(pin < PIN_NB, "Pin number out of range");
  direction[pin] = out;
}

void SET_INPUT(pin_t pin) {
  sim_assert(pin < PIN_NB, "Pin number out of range");
  direction[pin] = in;
}
