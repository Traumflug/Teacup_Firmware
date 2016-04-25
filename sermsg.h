
#ifndef _MSG_H
#define _MSG_H

#include "msg.h"
#include "serial.h"

/**
  Before we had display support, all messages went to the serial link,
  so this destination was hardcoded. These macros avoid changing some
  older code.

  Deprecated macros? Convenience macros? Dunno.
*/
#define serwrite_uint8(v)  write_uint8(serial_writechar, v)
#define serwrite_int8(v)   write_int8(serial_writechar, v)
#define serwrite_uint16(v) write_uint32(serial_writechar, v)
#define serwrite_int16(v)  write_int32(serial_writechar, v)
#define serwrite_uint32(v) write_uint32(serial_writechar, v);
#define serwrite_int32(v)  write_int32(serial_writechar, v);

#endif /* _MSG_H */
