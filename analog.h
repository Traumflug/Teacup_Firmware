#ifndef	_ANALOG_H
#define	_ANALOG_H

#include	<stdint.h>

#define	REFERENCE_AREF	0
#define	REFERENCE_AVCC	64
#define	REFERENCE_1V1		192

#include	"config.h"

#ifndef	REFERENCE
#warning	define REFERENCE as one of
#warning	REFERENCE_AREF, REFERENCE_AVCC or REFERENCE_1V1
#warning	in your config.h
#error REFERENCE undefined
#endif

void 			analog_init(void);
uint16_t	analog_read(uint8_t channel);

#endif	/* _ANALOG_H */
