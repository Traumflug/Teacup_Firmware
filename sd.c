
/** \file Coordinating reading and writing of SD cards.
*/

#include "sd.h"

#ifdef SD


void sd_init(void) {
  WRITE(SD_CARD_SELECT_PIN, 1);
  SET_OUTPUT(SD_CARD_SELECT_PIN);
}

#endif /* SD */
