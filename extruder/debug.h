#ifndef	_DEBUG_H
#define	_DEBUG_H

#include	<stdint.h>

// by setting these to zero, the compiler should optimise the relevant code out
#define		DEBUG_PID				0
#define		DEBUG_DDA				0
#define		DEBUG_POSITION	0

#define		DEBUG_ECHO			0

extern volatile uint8_t	debug_flags;

#endif	/* _DEBUG_H */
