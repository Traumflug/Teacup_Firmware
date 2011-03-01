#include	"crc.h"

#include	<util/crc16.h>

// 	uint16_t _crc16_update(uint16_t crc, uint8_t a) {
// 		int i;
// 		crc ^= a;
// 		for (i = 0; i < 8; ++i)
// 		{
// 			if (crc & 1)
// 				crc = (crc >> 1) ^ 0xA001;
// 			else
// 				crc = (crc >> 1);
// 		}
// 		return crc;
// 	}

uint16_t	crc_block(void *data, uint16_t len) {
	uint16_t	crc = 0;
	for (; len; data++, len--) {
		crc = _crc16_update(crc, *((uint8_t *) data));
	}
	return crc;
}
