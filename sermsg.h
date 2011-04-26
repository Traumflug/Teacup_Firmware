#ifndef	_SERMSG_H
#define	_SERMSG_H

#include	<stdint.h>

// functions for sending hexadecimal
void serwrite_hex4(uint8_t v);
void serwrite_hex8(uint8_t v);
void serwrite_hex16(uint16_t v);
void serwrite_hex32(uint32_t v);

// functions for sending decimal
#define	serwrite_uint8(v)		serwrite_uint32(v)
#define	serwrite_int8(v)		serwrite_int32(v)
#define	serwrite_uint16(v)	serwrite_uint32(v)
#define	serwrite_int16(v)		serwrite_int32(v)

void serwrite_uint32(uint32_t v);
void serwrite_int32(int32_t v);

void serwrite_uint32_vf(uint32_t v, uint8_t fp);
void serwrite_int32_vf(int32_t v, uint8_t fp);

#endif	/* _SERMSG_H */