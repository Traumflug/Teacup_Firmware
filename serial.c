#include	"serial.h"

/** \file
	\brief Serial subsystem

	Teacup's serial subsystem is a powerful, thoroughly tested and highly modular serial management system.

	It uses ringbuffers for both transmit and receive, and intelligently decides whether to wait or drop transmitted characters if the buffer is full.

	It also supports XON/XOFF flow control of the receive buffer, to help avoid overruns.
*/

#define TEACUP_C_INCLUDE
#include "serial-avr.c"
#include "serial-arm_lpc11xx.c"
#include "serial-arm_stm32f4xx.c"
#undef TEACUP_C_INCLUDE


/// send a string- look for null byte instead of expecting a length
void serial_writestr(uint8_t *data)
{
	uint8_t i = 0, r;
	// yes, this is *supposed* to be assignment rather than comparison, so we break when r is assigned zero
	while ((r = data[i++]))
		serial_writechar(r);
}

/**
  Write string from FLASH.

  Extensions to output flash memory pointers. This prevents the data to
  become part of the .data segment instead of the .code segment. That means
  less memory is consumed for multi-character writes.

  For single character writes (i.e. '\n' instead of "\n"), using
  serial_writechar() directly is the better choice.
*/
void serial_writestr_P(PGM_P data_P)
{
	uint8_t r, i = 0;
	// yes, this is *supposed* to be assignment rather than comparison, so we break when r is assigned zero
	while ((r = pgm_read_byte(&data_P[i++])))
		serial_writechar(r);
}
