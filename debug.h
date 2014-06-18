#ifndef	_DEBUG_H
#define	_DEBUG_H

#include	<stdint.h>

#ifdef	DEBUG
  #define DEBUG_ECHO       1
  #define DEBUG_INFO       2
  #define DEBUG_ERRORS     4
  #define DEBUG_DRYRUN     8
  #define DEBUG_PID       16
  #define DEBUG_DDA       32
  #define DEBUG_POSITION  64
#else
	// by setting these to zero, the compiler should optimise the relevant code out
	#define		DEBUG_PID				0
	#define		DEBUG_DDA				0
	#define		DEBUG_POSITION	0
	#define		DEBUG_ECHO			0
  #define DEBUG_INFO       0
  #define DEBUG_DRYRUN     0
#endif


extern volatile uint8_t	debug_flags;

#endif	/* _DEBUG_H */
