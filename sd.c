
/** \file Coordinating reading and writing of SD cards.
*/

#include "sd.h"

#ifdef SD

#include "delay.h"
#include "serial.h"
#include "sersendf.h"
#include "gcode_parse.h"


static FATFS sdfile;
static FRESULT result;

/** Initialize SPI for SD card reading.
*/
void sd_init(void) {
  WRITE(SD_CARD_SELECT_PIN, 1);
  SET_OUTPUT(SD_CARD_SELECT_PIN);
}

/** Mount the SD card.
*/
void sd_mount(void) {
  result = pf_mount(&sdfile);
  if (result != FR_OK)
    sersendf_P(PSTR("E: SD init failed. (%su)"), result);
}

/** Unmount the SD card.

  This makes just sure subsequent reads to the card do nothing, instead of
  trying and failing. Not mandatory, just removing the card is fine, as well
  as inserting and mounting another one without previous unmounting.
*/
void sd_unmount(void) {
  pf_unmount(&sdfile);
}

/** List a given directory.

  \param path The path to list. Toplevel path is "/".

  A slash is added to directory names, to make it easier for users to
  recognize them.
*/
void sd_list(const char* path) {
  FILINFO fno;
  DIR dir;

  serial_writechar('\n');
  result = pf_opendir(&dir, path);
  if (result == FR_OK) {
    for (;;) {
      result = pf_readdir(&dir, &fno);
      if (result != FR_OK || fno.fname[0] == 0)
        break;
      serial_writestr((uint8_t *)fno.fname);
      if (fno.fattrib & AM_DIR)
        serial_writechar('/');
      serial_writechar('\n');
      delay_ms(2); // Time for sending the characters.
    }
  }
  else {
    sersendf_P(PSTR("E: failed to open dir. (%su)"), result);
  }
}

/** Open a file for reading.

  \param filename Name of the file to open and to read G-code from.

  Before too long this will cause the printer to read G-code from this file
  until done or until stopped by G-code coming in over the serial line.
*/
void sd_open(const char* filename) {
  result = pf_open(filename);
  if (result != FR_OK) {
    sersendf_P(PSTR("E: failed to open file. (%su)"), result);
  }
}

/** Read a line of G-code from a file.

  \param A pointer to the parser function. This function should accept an
         uint8_t with the character to parse and return an uint8_t wether
         end of line (EOL) was reached.

  \return Wether end of line (EOF) was reached or an error happened.

  Juggling with a buffer smaller than 512 bytes means that the underlying
  SD card handling code reads a full sector (512 bytes) in each operation,
  but throws away everything not fitting into this buffer. Next read operation
  reads the very same sector, but keeps a different part. That's ineffective.

  Much better is to parse straight as it comes from the card. This way there
  is no need for a buffer at all. Sectors are still read multiple times, but
  at least one line is read in one chunk (unless it crosses a sector boundary).
*/
uint8_t sd_read_gcode_line(void) {

  result = pf_parse_line(&gcode_parse_char);
  if (result == FR_END_OF_FILE) {
    return 1;
  }
  else if (result != FR_OK) {
    sersendf_P(PSTR("E: failed to parse from file. (%su)"), result);
    return 1;
  }

  return 0;
}

#endif /* SD */
