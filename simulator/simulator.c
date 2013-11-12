#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <getopt.h>

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

#define AXES 4
enum {
  TRACE_POS    = 0,  /* 0..AXES-1 */
  TRACE_PINS   = AXES,
};

int verbose = 1;                ///< 0=quiet, 1=normal, 2=noisy, 3=debug, etc.
int trace_gcode = 0;            ///< show gcode on the console
int trace_pos = 0;              ///< show print head position on the console

const char * shortopts = "qvo::";
struct option opts[] = {
  { "quiet", no_argument, &verbose , 0 },
  { "verbose", no_argument, NULL, 'v' },
  { "gcode", no_argument, NULL, 'g' },
  { "pos", no_argument, NULL, 'p' },
  { "tracefile", optional_argument, NULL, 'o' }
};

static void usage(const char *name) {
  printf("Usage:  %s [-q || --quiet] [-v [-v] || --verbose[=n]] [-g || --gcode]  [-p || --pos] [-o[outfile] || --tracefile[=outfile]] gcode-file || UART-tty\n", name);
  printf("\n\n");
  printf("   -g || --gcode        show gcode on console as it is processed\n");
  printf("   -p || --pos          show head position on console\n");
  printf("   -o || --tracefile    write simulator pin trace to 'outfile' (default filename=datalog.out)\n");
  printf("\n");
  exit(1);
}

int g_argc;
char** g_argv;
void sim_start(int argc, char** argv) {
  int c;
  int index;

  while ((c = getopt_long (argc, argv, "qgpvo::", opts, &index)) != -1)
    switch (c) {
    case 'q':
      verbose = 0;
      break;
    case 'g':
      trace_gcode = 1;
      break;
    case 'p':
      trace_pos = 1;
      break;
    case 'v':
      verbose += 1;
      break;
    case 'o':
      recorder_init(optarg ? optarg : "datalog.out");
      break;
    default:
      sim_error("Unexpected result in getopt_long handler");
    }

  // Store remaining arguments list for serial sim
  g_argc = argc - optind + 1;
  g_argv = argv + optind - 1;

  if (argc < 2) usage(argv[0]);

  // Record pin names in datalog
#define NAME_PIN_AXES(x) \
  add_trace_var(#x "_X" , TRACE_##x + 0); \
  add_trace_var(#x "_Y" , TRACE_##x + 1); \
  add_trace_var(#x "_Z" , TRACE_##x + 2); \
  add_trace_var(#x "_E" , TRACE_##x + 3);
  NAME_PIN_AXES(POS);

#define NAME_PIN(x) add_trace_var(#x , TRACE_PINS + x);
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
}

/* -- debugging ------------------------------------------------------------ */

static void fgreen(void) { fputs("\033[0;32m" , stdout); }
static void fred(void)   { fputs("\033[0;31m" , stdout); }
static void fcyan(void)  { fputs("\033[0;36m" , stdout); }
static void fyellow(void){ fputs("\033[0;33;1m" , stdout); }
static void fbreset(void) { fputs("\033[m" , stdout); }

static void bred(void)   { fputs("\033[0;41m" , stdout); }

static void vsim_info_cont(const char fmt[], va_list ap) {
  if (verbose < 1) return;
  fgreen();
  vprintf(fmt, ap);
  fbreset();
}

static int sameline = 0;
static void clearline(void) {
  if (sameline)
    fputc('\n', stdout);
  sameline = 0;
}

static void sim_info_cont(const char fmt[], ...) {
  va_list ap;
  va_start(ap, fmt);
  vsim_info_cont(fmt, ap);
  va_end(ap);
  sameline = 1;
}

void sim_info(const char fmt[], ...) {
  va_list ap;
  clearline();
  va_start(ap, fmt);
  vsim_info_cont(fmt, ap);
  va_end(ap);
  if (verbose < 1) return;
  fputc('\n', stdout);
}

void sim_debug(const char fmt[], ...) {
  va_list ap;
  if (verbose < 3) return;
  clearline();
  fcyan();
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  fputc('\n', stdout);
  fbreset();
}

void sim_tick(char ch) {
  if (verbose < 2) return;
  fcyan();
  fprintf(stdout, "%c", ch);
  fbreset();
  fflush(stdout);
}

static char gcode_buffer[300]; 
static int gcode_buffer_index;
void sim_gcode_ch(char ch) {
  // Got CR, LF or buffer full
  if ( gcode_buffer_index == sizeof(gcode_buffer)-1 ||
       ch == '\r' || ch == '\n' || ch == 0 ) {

    // Terminate string, reset buffer, emit gcode
    if (gcode_buffer_index) {
      gcode_buffer[gcode_buffer_index] = 0;
      gcode_buffer_index = 0;

      if (trace_gcode) {
        clearline();
        fyellow();
        printf("%s\n", gcode_buffer);
        fbreset();
        fflush(stdout);
      }

      // Send gcode to data_recorder
      record_comment(gcode_buffer);
    }

    if (ch == '\r' || ch == '\n' || ch == 0)
      return;
  }

  // Acumulate char from stream
  gcode_buffer[gcode_buffer_index++] = ch;
}

void sim_gcode(const char msg[]) {
  for ( ; *msg ; msg++ ) sim_gcode_ch(*msg);
  sim_gcode_ch(0);
}

void sim_error(const char msg[]) {
  clearline();
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

/** Maximum time (ns) between steps which we still consider "movement"
 *  Must be less than 0x20000000, MAXINT/2 */
#define MAX_IDLE_TIME_NS (2*1000*1000*1000)
#define NS_PER_SEC       (1000*1000*1000)  // Token for "1 billion"

/* -- PIN I/O ------------------------------------------------------------ */

#define out true
#define in  false

enum { X_AXIS, Y_AXIS, Z_AXIS, E_AXIS , AXIS_MAX , AXIS_NONE };
static int pos[AXIS_MAX];            ///< Current position in steps

static bool direction[PIN_NB];
static bool state[PIN_NB];

static void print_pos(void) {
  char * axis = "xyze";
  int i;
  if (trace_pos) {
    for ( i = X_AXIS ; i < AXIS_MAX ; i++ ) {
      sim_info_cont("%c:%5d   ", axis[i], pos[i]);
    }
    if (verbose > 1)
      clearline();
    else
      sim_info_cont("               \r");
  }
}

bool READ(pin_t pin) {
  sim_assert(pin < PIN_NB, "READ: Pin number out of range");
  // Add any necessary reactive pin-handlers here.
  return state[pin];
}

void WRITE(pin_t pin, bool s) {
  bool old_state = state[pin];
  uint64_t nseconds = sim_runtime_ns();
  sim_assert(pin < PIN_NB, "WRITE: Pin number out of range");

  if (direction[pin] == out) {
    state[pin] = s;
  }

  if (old_state != s) {
    record_pin(TRACE_PINS + pin, s, nseconds);
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
    int axis = AXIS_NONE;
    int dir;
    switch (pin) {
    case X_STEP_PIN:
      dir = state[X_DIR_PIN] ? 1 : -1;
      axis = X_AXIS;
      break;
    case Y_STEP_PIN:
      dir = state[Y_DIR_PIN] ? 1 : -1;
      axis = Y_AXIS;
      break;
    case Z_STEP_PIN:
      dir = state[Z_DIR_PIN] ? 1 : -1;
      axis = Z_AXIS;
      break;
    case E_STEP_PIN:
      dir = state[E_DIR_PIN] ? 1 : -1;
      axis = E_AXIS;
      break;
    default:
      break;
    }
    if ( axis != AXIS_NONE ) {
      pos[axis] += dir;
      record_pin(TRACE_POS + axis, pos[axis], nseconds);
      print_pos();
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
