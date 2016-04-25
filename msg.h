#ifndef	_SERMSG_H
#define	_SERMSG_H

#include	<stdint.h>

// functions for sending hexadecimal
void write_hex4(void (*writechar)(uint8_t), uint8_t v);
void write_hex8(void (*writechar)(uint8_t), uint8_t v);
void write_hex16(void (*writechar)(uint8_t), uint16_t v);
void write_hex32(void (*writechar)(uint8_t), uint32_t v);

// functions for sending decimal
#define write_uint8(v, w)  write_uint32(v, w)
#define write_int8(v, w)   write_int32(v, w)
#define write_uint16(v, w) write_uint32(v, w)
#define write_int16(v, w)  write_int32(v, w)

void write_uint32(void (*writechar)(uint8_t), uint32_t v);
void write_int32(void (*writechar)(uint8_t), int32_t v);

void write_uint32_vf(void (*writechar)(uint8_t), uint32_t v, uint8_t fp);
void write_int32_vf(void (*writechar)(uint8_t), int32_t v, uint8_t fp);

#endif	/* _SERMSG_H */
