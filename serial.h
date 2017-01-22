#ifndef	_SERIAL_H
#define	_SERIAL_H

#include "config_wrapper.h"
#include	<stdint.h>


#ifdef USB_SERIAL
  #include "usb_serial.h"
  #define serial_init() usb_init()
  #define serial_rxchars() usb_serial_available()
  #define serial_popchar() usb_serial_getchar()
#else
  // initialise serial subsystem
  void serial_init(void);

  // return number of characters in the receive buffer,
  // and number of spaces in the send buffer
  uint8_t serial_rxchars(void);
  // uint8_t serial_txchars(void);

  // read one character
  uint8_t serial_popchar(void);
  // send one character
  void serial_writechar(uint8_t data);

  #ifdef SERIAL_DEBUG
    // uint8_t debug_rxchars(void);
    // uint8_t debug_txchars(void);
    // uint8_t debug_popchar(void);
    void debug_writechar(uint8_t data);
  #else
    #define debug_writechar(data) serial_writechar(data)
  #endif

#endif /* USB_SERIAL */

void writestr(void (*writechar)(uint8_t), uint8_t *data, ...);

// write from flash
void writestr_P(void (*writechar)(uint8_t), PGM_P data_P, ...);

#define serial_writestr(...) writestr(serial_writechar, __VA_ARGS__)
#define serial_writestr_P(...) writestr_P(serial_writechar, __VA_ARGS__)

#ifdef SERIAL_DEBUG
  #define debug_writestr(...) writestr(debug_writechar, __VA_ARGS__)
  #define debug_writestr_P(...) writestr_P(debug_writechar, __VA_ARGS__)
#else
  #define debug_writestr(...) writestr(serial_writechar, __VA_ARGS__)
  #define debug_writestr_P(...) writestr_P(serial_writechar, __VA_ARGS__)
#endif

#endif	/* _SERIAL_H */
