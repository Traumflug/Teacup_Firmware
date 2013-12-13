/*
 * This wrapper config header is used to allow makefiles and test scripts to
 * replace or augment the user's 'config.h' file in a controlled manner. A
 * makefile may add CFLAGS+=-DUSER_CONFIG=alternate_config.h to cause Teacup
 * to build with a different config header.
 */

#ifndef USER_CONFIG
#define USER_CONFIG "config.h"
#endif

#include USER_CONFIG
