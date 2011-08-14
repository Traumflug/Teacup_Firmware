
#include <stdio.h>
#include <stdlib.h>

#define __AVR_ATmega1280__	1
#define	SKIP_ARDUINO_INC	1

#include "config.h"
#include "dda.h"

#define PRINT_INT_XYZ( macro, units) printf( "   %-25s  %10d     %10d     %10d            n/a          %s\n", \
		# macro, macro ## X, macro ## Y, macro ## Z, units);
#define PRINT_INT_XYZE( macro, units) printf( "   %-25s  %10d     %10d     %10d     %10d          %s\n", \
		# macro, macro ## X, macro ## Y, macro ## Z, macro ## E, units);
#define PRINT_LONG_XYZE( macro, units) printf( "   %-25s  %10ld     %10ld     %10ld     %10ld          %s\n", \
		# macro, macro ## X, macro ## Y, macro ## Z, macro ## E, units);
#define PRINT_FLOAT_XYZE( macro, units) printf( "   %-25s      %10.3f     %10.3f     %10.3f     %10.3f      %s\n", \
		# macro, macro ## X, macro ## Y, macro ## Z, macro ## E, units);

void main( void)
{
	printf( "\nThese are the actual values as defined in config.h and other header files:\n\n");
	printf( "                                   X-axis         Y-axis         Z-axis         E-axis           units\n");
	printf( "config.h:\n");
	PRINT_INT_XYZE( STEPS_PER_REV_, "[steps/rev]");
	PRINT_FLOAT_XYZE( FEED_PER_REV_, "[mm/dev]");
	PRINT_FLOAT_XYZE( STEPS_PER_MM_, "[steps/mm]");
	PRINT_FLOAT_XYZE( MAX_REV_SPEED_, "[rev/sec]");
	PRINT_INT_XYZE( MAX_STEP_FREQ_, "[steps/sec]");
	PRINT_INT_XYZE( MAXIMUM_FEEDRATE_, "[mm/min]");
	PRINT_INT_XYZ( SEARCH_FEEDRATE_, "[mm/min]");

	printf( "dda.h:\n");
	PRINT_LONG_XYZE( UM_PER_STEP_, "[um/step]");

	printf( "\n");
	exit( 0);
}


