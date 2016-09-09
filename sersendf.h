#ifndef	_SERSENDF_H
#define	_SERSENDF_H

#include "sendf.h"
#include "serial.h"

/**
  Before we had display support, all messages went to the serial link,
  so this destination was hardcoded. This macro avoids changing a whole lot
  of older code.

  Deprecated macro? Convenience macro? Dunno.
*/
#define sersendf_F(...) sendf_F(serial_writechar, __VA_ARGS__)

#endif	/* _SERSENDF_H */
