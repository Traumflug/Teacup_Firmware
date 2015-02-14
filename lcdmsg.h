#ifndef	_LCDMSG_H
#define	_LCDMSG_H

#include	<stdint.h>

// functions for sending hexadecimal
void lcdwrite_hex4(uint8_t v);
void lcdwrite_hex8(uint8_t v);
void lcdwrite_hex16(uint16_t v);
void lcdwrite_hex32(uint32_t v);

// functions for sending decimal
#define	lcdwrite_uint8(v)		lcdwrite_uint32(v)
#define	lcdwrite_int8(v)		lcdwrite_int32(v)
#define	lcdwrite_uint16(v)	lcdwrite_uint32(v)
#define	lcdwrite_int16(v)		lcdwrite_int32(v)

void lcdwrite_uint32(uint32_t v);
void lcdwrite_int32(int32_t v);

void lcdwrite_uint32_vf(uint32_t v, uint8_t fp);
void lcdwrite_int32_vf(int32_t v, uint8_t fp);

#endif	/* _LCDMSG_H */