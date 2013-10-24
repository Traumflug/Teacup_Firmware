#include	"crc.h"

/** \file
	\brief crc16 routine
*/

#ifndef SIMULATOR
#include	<util/crc16.h>
#else

// Equivalent to avr-libc's _crc16_update.
uint16_t _crc16_update(uint16_t crc, uint8_t a) {
  int i;
  crc ^= a;
  for (i = 0; i < 8; ++i) {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
  return crc;
}
#endif

/** block-at-once CRC16 calculator
	\param *data data to find crc16 for
	\param len length of data
	\return uint16 crc16 of passed data

	uses avr-libc's optimised crc16 routine
*/
uint16_t	crc_block(void *data, uint16_t len) {
	uint16_t	crc = 0xfeed;
	for (; len; data++, len--) {
		crc = _crc16_update(crc, *((uint8_t *) data));
	}
	return crc;
}
