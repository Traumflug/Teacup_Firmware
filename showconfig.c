
#include <stdio.h>
#include <stdlib.h>

#define __AVR_ATmega1280__	1
#define	SKIP_ARDUINO_INC	1

#include "config.h"

typedef int uint32_t;

void main( void)
{
	printf( "\nThese are the actual values as defined in config.h:\n\n");
	printf( "                       X-axis              Y-axis              Z-axis              E-axis            units\n");
	printf( "STEPS_PER_REV_    %10d          %10d          %10d          %10d           [steps/rev]\n",
		STEPS_PER_REV_X, STEPS_PER_REV_Y, STEPS_PER_REV_Z, STEPS_PER_REV_E);
	printf( "FEED_PER_REV_         %10.3f          %10.3f          %10.3f          %10.3f       [mm/rev]\n",
		FEED_PER_REV_X, FEED_PER_REV_Y, FEED_PER_REV_Z, FEED_PER_REV_E);
	printf( "STEPS_PER_MM_         %10.3f          %10.3f          %10.3f          %10.3f       [steps/mm]\n",
		STEPS_PER_MM_X, STEPS_PER_MM_Y, STEPS_PER_MM_Z, STEPS_PER_MM_E);
	printf( "MAX_REV_SPEED_        %10.3f          %10.3f          %10.3f          %10.3f       [rev/sec]\n",
		MAX_REV_SPEED_X, MAX_REV_SPEED_Y, MAX_REV_SPEED_Z, MAX_REV_SPEED_E);
	printf( "MAX_STEP_FREQ_    %10d          %10d          %10d          %10d           [steps/sec]\n",
		(int)MAX_STEP_FREQ_X, (int)MAX_STEP_FREQ_Y, (int)MAX_STEP_FREQ_Z, (int)MAX_STEP_FREQ_E);
	printf( "MAXIMUM_FEEDRATE_ %10d          %10d          %10d          %10d           [mm/min]\n",
		MAXIMUM_FEEDRATE_X, MAXIMUM_FEEDRATE_Y, MAXIMUM_FEEDRATE_Z, MAXIMUM_FEEDRATE_E);
	printf( "SEARCH_FEEDRATE_  %10d          %10d          %10d                 N/A           [mm/min]\n\n",
		SEARCH_FEEDRATE_X, SEARCH_FEEDRATE_Y, SEARCH_FEEDRATE_Z);
	exit( 0);
}


