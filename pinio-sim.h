/*
  pinio functions for simulator
*/

#if defined SIMULATOR

  #include "simulator.h"

/*
  We are not concerned with speed, so our read/write macros are defined as
  functions which can produce different types of traceability.
 */
  bool _READ(pin_t pin);
  void _WRITE(pin_t pin, bool on);
  void _SET_OUTPUT(pin_t pin);
  void _SET_INPUT(pin_t pin);
  #define _PULLUP_ON(IO)   _WRITE(IO, 1)
  #define _PULLUP_OFF(IO)  _WRITE(IO, 0)

#endif /* SIMULATOR */
