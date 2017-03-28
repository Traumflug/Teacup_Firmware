#ifndef _ROT_ENC_H
#define _ROT_ENC_H


#include "config_wrapper.h"
#include <stdint.h>

void rot_enc_init(void);
// Read Encoder state
int8_t rot_enc_read(uint8_t rot_enc_clear);
// Report button state
uint8_t rot_enc_but(uint8_t rot_enc_bclr);

#endif /* _ROT_ENC_H */
