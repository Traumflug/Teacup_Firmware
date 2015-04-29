#ifndef	_DEBUG_H
#define	_DEBUG_H

#include	<stdint.h>

/** \def DEBUG
  Enable debug output generally and some additional M-codes.

  Individual flags are enabled at runtime using M111 (see gcode_process.c) or
  by modifying the presets here.

  WARNING: enabling some or all of these is done can produce /heaps/ of extra
  output. This extra output will break most host-side talkers that expect
  particular responses from firmware such as RepRap Host or ReplicatorG. Use
  with serial terminal or other suitable talker only.
*/
//#define DEBUG

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
