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

/**
  Give users a hint in case they obviously forgot to read instructions.
*/
#ifndef STEPS_PER_M_X
  #error config.h missing. Please follow instructions at \
    reprap.org/wiki/Teacup_Firmware#Simple_Installation
#endif

/*
	For Scara-style printers (e.g. like the RepRap Morgan), X- and Y-axis movements need a different calculation from
	Delta-type printers (e.g. like the RepRap Mendel).
	Activate SCARA_PRINTER to use Scara-calculations, instead of the default for Delta-printers.
	Using SCARA_PRINTER will enable Scara-specific constants and code in several places in the firmware.
	It will also override some definitions of the user's config-file (see scara.h for details).

	Comment out the following definition of SCARA_PRINTER to compile for Delta-printers.
*/
#define	SCARA_PRINTER

#ifdef	SCARA_PRINTER
	#include	"scara.h"
#endif

