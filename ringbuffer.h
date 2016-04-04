/** \file
  \brief Header for implementing a ringbuffer.

  This header implements a macro-level ringbuffer. It's meant to be included
  only in source files and only where needed. Such a sequence implements a
  ringbuffer:

    #define BUFSIZE     64
    volatile uint8_t rxhead = 0;
    volatile uint8_t rxtail = 0;
    volatile uint8_t rxbuf[BUFSIZE];
    #include "ringbuffer.h"

  Requirements:

    - The name 'BUFSIZE' has to be kept exactly. One can use two or more
      ringbuffers in the same source file, but they all have to be the same
      size.

    - BUFSIZE has to be a power of two: 2, 4, 8, 16, 32, 64, 128 or 256.

    - In the example above, 'rx' is the name of the buffer. <name>head,
      <name>tail and <name>buf have to be named exactly.

  Ringbuffer logic:

  head = written data pointer.
  tail = read data pointer.

  When head == tail, buffer is empty.
  When head + 1 == tail, buffer is full.
  Thus, number of available spaces in buffer is (tail - head) & bufsize.

  Can write:
  (tail - head - 1) & (BUFSIZE - 1)

  Write to buffer:
  buf[head++] = data; head &= (BUFSIZE - 1);

  Can read:
  (head - tail) & (BUFSIZE - 1)

  Read from buffer:
  data = buf[tail++]; tail &= (BUFSIZE - 1);
*/

/** \def buf_canread()

  Check if we can read from this buffer. Buffer pointers are left unchanged.

  Example:

    if (buf_canread(rx)) {
      ...
    }

*/
#define buf_canread(buffer)     ((buffer ## head - buffer ## tail ) & \
                                 (BUFSIZE - 1))

/** \def buf_pop()

  Actually read a byte from this buffer. This also forwards the buffer pointer.
  It's imperative to check for availabilty of a byte before. Pop'ing an
  empty buffer means to go through the whle buffer again.

  Example:

    uint8_t next_byte;

    if (buf_canread(rx)) {
      buf_pop(rx, next_byte);
    }

*/
#define buf_pop(buffer, data)   do { \
                                  data = buffer ## buf[buffer ## tail]; \
                                  buffer ## tail = (buffer ## tail + 1) & \
                                    (BUFSIZE - 1); \
                                } while (0)

/** \def buf_canwrite()

  Check if we can write to this buffer. Buffer pointers are left unchanged.

  Example:

    uint8_t data;

    if (buf_canwrite(tx)) {
      printf("There's room in the buffer, please send data.\n");
    }

*/
#define buf_canwrite(buffer)    ((buffer ## tail - buffer ## head - 1) & \
                                 (BUFSIZE - 1))

/** \def buf_push()

  Actually write to this buffer. This also forwards the buffer pointers. Like
  with buf_pop() it's mandatory to test for available room before actually
  writing, else valid data is overwritten.

  Example:

    void write_char_to_buffer(uint8_t data) {
      if (buf_canwrite(tx))
        buf_push(tx, data);
    }
*/
#define buf_push(buffer, data)  do { \
                                  buffer ## buf[buffer ## head] = data; \
                                  buffer ## head = (buffer ## head + 1) & \
                                    (BUFSIZE - 1); \
                                } while (0)
