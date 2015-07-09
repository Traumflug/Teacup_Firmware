
/** \file SD card simulator
*/

#include "sd.h"

#ifdef SD

#include "delay.h"
#include "serial.h"
#include "sersendf.h"
#include "gcode_parse.h"

#include <sys/types.h>
#include <dirent.h>
#include <stdio.h>
#include <errno.h>

/** Initialize SPI for SD card reading.
*/
void sd_init(void) {
  sim_info("SD card mounted (simulated).");
}

void sd_mount(void) {}

void sd_unmount(void) {}

/** List a given directory.

  \param path The path to list. Toplevel path is "/".

  A slash is added to directory names, to make it easier for users to
  recognize them.
*/
void sd_list(const char* path) {
  DIR *dir;

  dir = opendir(path);
  serial_writechar('\n');
  if (dir) {
    for (;;) {
      struct dirent *de = readdir(dir);
      if (!de)
        break;
      serial_writestr((uint8_t *)de->d_name);
      #ifdef _DIRENT_HAVE_D_TYPE
        if (de->d_type & DT_DIR)
          serial_writechar('/');
      #endif
      serial_writechar('\n');
    }
  }
  else {
    sim_info("E: failed to open dir. (%s, %u)", path, errno);
  }
}

/** Open a file for reading.

  \param filename Name of the file to open and to read G-code from.
*/
static FILE *f;
void sd_open(const char* filename) {
  if (f) fclose(f);
  f = fopen(filename, "rb");
  if (!f) {
    sim_info("E: failed to open file. (%s, %u)", filename, errno);
  }
}

uint8_t gcode_parse_char_sd(uint8_t c) {
  return gcode_parse_char(c);
}

/** Read a line of G-code from a file.

  \param A pointer to the parser function. This function should accept an
         uint8_t with the character to parse and return an uint8_t whether
         end of line (EOL) was reached.

  \return Whether end of line (EOF) was reached or an error happened.
*/
uint8_t sd_read_gcode_line(void) {
  char str[1024] ;
  if (!f) {
    sim_info("E: file not open.");
    return 1;
  }
  char *s = fgets(str, sizeof(str), f);
  if (!s) {
    sim_info("SD card: reached EOF.");
    return 1;
  }

  while (*s)
    gcode_parse_char_sd((uint8_t)*s++);
  return 0;
}

#endif /* SD */
