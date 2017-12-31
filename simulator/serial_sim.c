#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>

#include "dda_queue.h"
#include "serial.h"
#include "simulator.h"

static int serial_fd;
static bool serial_initialised = false;

extern int g_argc;
extern char ** g_argv;
static int gcode_fd;

static void open_tty(const char *devname);
static void open_file(void);

void serial_init(void) {
  sim_assert(g_argc >= 2, "please specify a serial port device name or gcode file to process");

  open_file();
  serial_initialised = true;
}

static void open_file() {
  struct stat st;
  static int i=1;
  const char * filename = g_argv[i++];

  // Close previous file
  if (gcode_fd) close(gcode_fd);
  gcode_fd = 0;

  // No more files
  if (i > g_argc) return;

  sim_info("Opening G-code source %s.", filename);
  sim_assert(!stat(filename, &st), "Could not stat G-code source.");

  if (!st.st_rdev) {
    // Normal file
    gcode_fd = open(filename, O_RDONLY );
    sim_assert(gcode_fd, "Could not open G-code file.");
  } else {
    // Some kind of device (treat as TTY)
    open_tty(filename);
  }
}

static void open_tty(const char *devname)
{
  struct termios options;
  serial_fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);
  sim_assert(serial_fd != -1, "couldn't open serial port");
  sim_assert(isatty(serial_fd), "not a TTY");

  // Get the current options for the port
  if (tcgetattr(serial_fd, &options) != 0) {
    sim_error("tcgetattr");
  }

  // Set the baud rates
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // Enable the receiver and set local mode
  options.c_cflag |= (CLOCAL | CREAD);

  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // Set the new options for the port
  if (tcsetattr(serial_fd, TCSANOW, &options) != 0) {
    sim_error("tcsetattr");
  }

  // flush tx and rx buffers
  tcflush(serial_fd, TCIOFLUSH);
}

// return number of characters in the receive buffer
uint8_t serial_rxchars(void) {
  sim_assert(serial_initialised, "serial interface not initialised");
  if (serial_fd) {
    int rx_chars_nb;

    ioctl(serial_fd, FIONREAD, &rx_chars_nb);
    return rx_chars_nb;
  }

  // An open file always has more data
  if (gcode_fd) return 1;

  // No more gcode data; wait for DDA queue to drain
  if (mb_tail_dda == NULL) {
    sim_info("Gcode processing completed.");
    exit(0);
  }

  // Nothing to read from
  return 0;
}

// read one character
uint8_t serial_popchar(void) {
  uint8_t c = 0;
  ssize_t count = 0;
  int fd = serial_fd ? serial_fd : gcode_fd;

  sim_assert(serial_initialised, "serial interface not initialised");
  sim_assert(serial_rxchars() > 0, "no chars to read");
  while (fd && !count) {
    count = read(fd, &c, 1);
    if (!gcode_fd || count)
      break;
    // EOF: try to open next file
    open_file();
    fd = serial_fd ? serial_fd : gcode_fd;
  }
  return c;
}

// send one character
void serial_writechar(uint8_t data) {
  sim_assert(serial_initialised, "serial interface not initialised");
  record_comment_stream((char)data);
  if (serial_fd) {
    ssize_t count;
    count = write(serial_fd, &data, 1);
    sim_assert(count == 1, "could not write to serial port");
  }
}

// read/write many characters
void serial_writestr(uint8_t *data) {
  const char *str = (char *)data;
  sim_assert(serial_initialised, "serial interface not initialised");
  record_comment(str);
  if (serial_fd) {
    ssize_t count;
    count = write(serial_fd, str, strlen(str));
    sim_assert(count == strlen(str), "could not write to serial port");
  }
}

// write from flash
void serial_writestr_P(PGM_P data) {
  serial_writestr((uint8_t *)data);
}
