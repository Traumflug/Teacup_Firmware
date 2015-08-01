
/**
  Some stuff common to all config.h files. Put it here to allow files like
  board.*.config.h or printer.*.h to be moved to about everywhere in the
  file system. Else we'd have to enforce the Configtool user to put these
  files where we need them.
*/
#include "arduino.h"

/**
  This wrapper config header is used to allow makefiles and test scripts to
  replace or augment the user's 'config.h' file in a controlled manner. A
  makefile may add CFLAGS+=-DUSER_CONFIG=alternate_config.h to cause Teacup
  to build with a different config header.
*/
#ifndef USER_CONFIG
#define USER_CONFIG "config.h"
#endif

#include USER_CONFIG

/**
  Give users a hint in case they obviously forgot to read instructions.
*/
#ifndef STEPS_PER_M_X
  #error config.h missing. Please follow instructions at \
    reprap.org/wiki/Teacup_Firmware#Simple_Installation
#endif

/**
  Additional tests to protect against misconfiguration.
*/
#ifdef USB_SERIAL
  #undef BAUD
#endif

/**
  ACCELERATION_TEMPORAL doesn't support lookahead, yet.
*/
#if defined ACCELERATION_TEMPORAL && defined LOOKAHEAD
  #warning Acceleration temporal doesnt support lookahead, yet. \
           Lookahead disabled.
  #undef LOOKAHEAD
#endif

#ifdef __ARMEL_NOTYET__
  #undef LOOKAHEAD
#endif
