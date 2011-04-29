
/** \file Coordinating reading and writing of SD cards.
*/

#include "sd.h"

#ifdef SD

#include "delay.h"
#include "serial.h"
#include "sersendf.h"


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

#endif /* SD */
