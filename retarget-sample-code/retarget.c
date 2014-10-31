
#include <serial_api.h>

serial_t serial_line;


void retarget_init() {
  serial_init(&serial_line, USBTX, USBRX);
  serial_baud(&serial_line, 115200);
  serial_format(&serial_line, 8, ParityNone, 1);
}

int _write (int fd, char *ptr, int len) {
  /* Write "len" of char from "ptr" to file id "fd"
   * Return number of char written. */
  int chars_to_go = len;

  while (chars_to_go) {
    serial_putc(&serial_line, *ptr);
    ptr++;
    chars_to_go--;
  }

  return len;
}

int _read (int fd, char *ptr, int len) {
  /* Read "len" of char to "ptr" from file id "fd"
   * Return number of char read. */
  int chars_got = 0;

  while (chars_got < len) {
    *ptr = (char)serial_getc(&serial_line);
    chars_got++;
    /* Reading only up to a newline makes more
     * sense when dealing with human input. */
    if (*ptr == '\n' || *ptr == '\r') {
      break;
    }
    ptr++;
  }

  return chars_got;
}

void _ttywrch(int ch) {
  /* Write one char "ch" to the default console. */
  serial_putc(&serial_line, ch);
}
