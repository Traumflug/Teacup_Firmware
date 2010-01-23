#ifndef	_SERMSG_H
#define	_SERMSG_H

#include	<stdint.h>

// void serwrite_uint8(uint8_t v);
// void serwrite_int8(int8_t v);
//
// void serwrite_uint16(uint16_t v);
// void serwrite_int16(int16_t v);

#define	serwrite_uint8(v)		serwrite_uint32(v)
#define	serwrite_int8(v)		serwrite_int32(v)
#define	serwrite_uint16(v)	serwrite_uint32(v)
#define	serwrite_int16(v)		serwrite_int32(v)

void serwrite_uint32(uint32_t v);
void serwrite_int32(int32_t v);

#endif	/* _SERMSG_H */