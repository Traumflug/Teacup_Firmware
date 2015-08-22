
/** \file Coordinating reading and writing of SD cards.
*/

#include "sd.h"

#ifdef SD

#include "delay.h"
#include "serial.h"
#include "sersendf.h"
#include "SimpleLCD.h"

#define SD_BUFFER_SIZE 16


static FATFS sdfile;
static FRESULT result;

static uint8_t sd_buffer[SD_BUFFER_SIZE];
static uint8_t sd_buffer_ptr = SD_BUFFER_SIZE;

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
  if (result != FR_OK){
    sersendf_P(PSTR("E: SD init failed. (%su)"), result);
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"SD init Failed      ");

    }
    else
    {
		lcdGoToAddr(0x54); 
		lcdWriteText((uint8_t *)"SD init Success     ");

    sersendf_P(PSTR("E: SD init OK. (%su)"), result);
    }


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

#ifdef LCD
/** List to LCD with offset.

  \param path    The path to list. Toplevel path is "/".
  \param offset  Offset in the file list.

  We display three file names, all indented by two spaces, and a '>' in front
  of the current file. To scroll through a long file list, change the offset
  incrementally depending on up/down buttons or an turning encoder and call
  this function on each such event.

  A slash is added to directory names, to make it easier for users to
  recognize them.
*/



sd_list_offset(const char* path,uint16_t count) {
  FILINFO fno;
  DIR dir;
  uint16_t listpos=0;
  result = pf_opendir(&dir, path);
  if (result == FR_OK) {
    for (;;) {
	
      result = pf_readdir(&dir, &fno);
      if (result != FR_OK || fno.fname[0] == 0)
	break;


       
      
      if (listpos==count){
	lcdGoToAddr(0x54); 
	lcdWriteText((uint8_t *)fno.fname);
	break;
	}
	listpos++;

}

  }

}
#endif

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

/** Read a character from a file.

  \return The character read, or zero if there is no such character (e.g. EOF).

  In principle it'd be possible to read the file character by character.
  However, Before too long this will cause the printer to read G-code from this file
  until done or until stopped by G-code coming in over the serial line.
*/
uint8_t sd_read_char(void) {
  UINT read;
  uint8_t this_char;

  if (sd_buffer_ptr == SD_BUFFER_SIZE) {
    result = pf_read(sd_buffer, SD_BUFFER_SIZE, &read);
    if (result != FR_OK) {
      sersendf_P(PSTR("E: failed to read from file. (%su)"), result);
      return 0;
    }
    if (read < SD_BUFFER_SIZE) {
      sd_buffer[read] = 0;           // A zero marks EOF.
    }

    sd_buffer_ptr = 0;
  }

  this_char = sd_buffer[sd_buffer_ptr];
  if (this_char == 0)
    sd_buffer_ptr = SD_BUFFER_SIZE;  // Start over, perhaps with next file.
  else
    sd_buffer_ptr++;

  return this_char;
}

#endif /* SD */
