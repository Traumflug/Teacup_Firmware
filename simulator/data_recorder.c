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

static void recorder_close(int code, void*x ) {
  // force last line to emit
  emit_log_data();
  fflush(file);
  fclose(file);
}

void recorder_init(const char* filename) {
  sim_assert( ! file, "Recorder already initialized");

  file = fopen(filename, "w");
  sim_assert(file, "record_init: failed to create file");

  time_t t = time(NULL);
  fprintf(file, "# Teacup_Firmware simulator v1.0\n");
  fprintf(file, "# Recorded %s\n", asctime(localtime(&t)));
  fflush(file);
  on_exit(recorder_close, NULL);
}

void add_trace_var(const char* name, int pin) {
  sim_assert(file, "add_trace_var: Recorder not initialized");
  sim_assert(pin < MAX_PINS, "pin number invalid");

  fprintf(file, "# %d - %s\n", pin, name);
  if (pin >= pin_count)
    pin_count = pin + 1;
  fflush(file);
}

static uint64_t prev_t;
static void emit_log_data(void) {
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

  sim_assert(file , "record_pin: Recorder not initialized");
  sim_assert(pin < MAX_PINS, "pin number invalid");

  // Record previous state when new state value appears
  if ( t != prev_t ) {
    emit_log_data();
  }
  prev_t = t;
  values[pin] = state;
}

static char comment_buffer[300];
static int comment_buffer_index;
void record_comment_stream(char ch) {
  // Got CR, LF or buffer full
  if ( comment_buffer_index == sizeof(comment_buffer)-1 ||
       ch == '\r' || ch == '\n' || ch == 0 ) {

    // Terminate string, reset buffer, emit comment
    comment_buffer[comment_buffer_index] = 0;
    comment_buffer_index = 0;
    record_comment(comment_buffer);
    if (ch == '\r' || ch == '\n')
      return;
  }

  // Acumulate char from stream
  comment_buffer[comment_buffer_index++] = ch;
}

void record_comment(const char * msg) {
  fprintf(file, "# %s\n", msg);
  fflush(file);
}
