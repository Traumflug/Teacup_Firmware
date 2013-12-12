/*
 * data_recorder.c
 *
 * Record simulator data to a file in real-time.
 *
 */

#include "simulator.h"
#include "data_recorder.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static FILE *file;

#define MAX_PINS 100
static int32_t values[MAX_PINS];       ///< Pin and value states
static int pin_count;                  ///< Number of pins we emit
static void emit_log_data(void);

static void recorder_close(void) {
  if (!file) return;
  // force last line to emit
  emit_log_data();
  fflush(file);
  fclose(file);
  file = NULL;
}

void recorder_init(const char* filename) {
  sim_assert( ! file, "Recorder already initialized");

  file = fopen(filename, "w");
  sim_assert(file, "record_init: failed to create file");

  time_t t = time(NULL);
  fprintf(file, "# Teacup_Firmware simulator v1.0\n");
  fprintf(file, "# Recorded %s\n", asctime(localtime(&t)));
  fprintf(file, "#\n# Tips:\n");
  fprintf(file, "# * Plot the actual points traveled with gnuplot\n");
  fprintf(file, "#   gnuplot --persist -e \"set pointsize 0.01; plot '%s' u 2:3:1 with points\"\n", filename);
  fprintf(file, "#\n");

  fflush(file);
  atexit((void*)recorder_close);
}

void add_trace_var(const char* name, int pin) {
  sim_assert(pin < MAX_PINS, "pin number invalid");

  if (file) {
    fprintf(file, "# %d - %s\n", pin+1, name);
    fflush(file);
  }

  if (pin >= pin_count)
    pin_count = pin + 1;
}

static uint64_t prev_t;
static void emit_log_data(void) {
  if (!file) return;
  // Naive format: each line contains all values, beginning with the time
  fprintf(file, "%lu", prev_t/1000); // microseconds
  for (int i = 0; i < pin_count; i++)
    fprintf(file, " %u", values[i]);
  fprintf(file, "\n");
  fflush(file);
}

// Record a value signal change
void record_pin(int pin, int32_t state, uint64_t t) {
  if (state == values[pin] && t == prev_t ) return;

  sim_assert(pin < MAX_PINS, "pin number invalid");

  emit_log_data();
  prev_t = t;
  values[pin] = state;
}

void record_comment(const char * msg) {
  if (!file) return;
  fprintf(file, "# %s\n", msg);
  fflush(file);
}

void record_raw(const char * msg) {
  if (!file) return;
  fprintf(file, "%s", msg);
  fflush(file);
}
