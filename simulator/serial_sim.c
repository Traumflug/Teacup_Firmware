#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "serial.h"
#include "simulator.h"

static int serial_fd;
static bool serial_initialised = false;

extern int g_argc;
extern char ** g_argv;

void serial_init(void) {
  struct termios options;

  int argc = g_argc;
  char **argv = g_argv;

  sim_assert(argc >= 2, "please specify a serial port device name");

  sim_info("opening serial port %s", argv[1]);
  serial_fd = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
  sim_assert(serial_fd != -1, "couldn't open serial port");
  sim_assert(isatty(serial_fd), "not a TTY");

  sim_info("configuring port");
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

  serial_initialised = true;
}

// return number of characters in the receive buffer
uint8_t serial_rxchars(void) {
  int rx_chars_nb;

  sim_assert(serial_initialised, "serial interface not initialised");
  ioctl(serial_fd, FIONREAD, &rx_chars_nb);
  return rx_chars_nb;
}

// read one character
uint8_t serial_popchar(void) {
  uint8_t c;
  ssize_t count;

  sim_assert(serial_initialised, "serial interface not initialised");
  sim_assert(serial_rxchars() > 0, "no chars to read");
  count = read(serial_fd, &c, 1);
  sim_assert(count == 1, "no character in serial RX buffer");
  return c;
}

// send one character
void serial_writechar(uint8_t data) {
  ssize_t count;
  sim_assert(serial_initialised, "serial interface not initialised");
  putchar(data);
  count = write(serial_fd, &data, 1);
  sim_assert(count == 1, "could not write to serial port");
}

// read/write many characters
void serial_writestr(uint8_t *data) {
  ssize_t count;
  const char *str = (char *)data;
  sim_assert(serial_initialised, "serial interface not initialised");
  puts(str);
  count = write(serial_fd, str, strlen(str));
  sim_assert(count == strlen(str), "could not write to serial port");
}

// write from flash
void serial_writestr_P(PGM_P data) {
  serial_writestr((uint8_t *)data);
}
