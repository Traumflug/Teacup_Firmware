#ifndef _CONFIG_MACROS_H
#define _CONFIG_MACROS_H

/**********************************************************************
 *  These macros are for conversion of the configuration definitions  *
 *  and are independent from any particular configuration.            *
 *********************************************************************/

/// The number of step pulses needed for one complete motor axis revolution [pulses / rev].
#define STEPS_PER_REV_X				(MOTOR_S_P_R_X * MICROSTEPPING_X)
#define STEPS_PER_REV_Y				(MOTOR_S_P_R_Y * MICROSTEPPING_Y)
#define STEPS_PER_REV_Z				(MOTOR_S_P_R_Z * MICROSTEPPING_Z)
#define STEPS_PER_REV_E				(MOTOR_S_P_R_E * MICROSTEPPING_E)

/// determine the number of step pulses needed for each mm feed [pulses / mm].
#define STEPS_PER_MM_X				((double)STEPS_PER_REV_X / FEED_PER_REV_X)
#define STEPS_PER_MM_Y				((double)STEPS_PER_REV_Y / FEED_PER_REV_Y)
#define STEPS_PER_MM_Z				((double)STEPS_PER_REV_Z / FEED_PER_REV_Z)
#define STEPS_PER_MM_E				((double)STEPS_PER_REV_E / FEED_PER_REV_E)

/// Upper limit of step frequency [pulses / sec]
#define MAX_STEP_FREQ_X				(uint32_t)(MAX_REV_SPEED_X * STEPS_PER_REV_X)
#define MAX_STEP_FREQ_Y				(uint32_t)(MAX_REV_SPEED_Y * STEPS_PER_REV_Y)
#define MAX_STEP_FREQ_Z				(uint32_t)(MAX_REV_SPEED_Z * STEPS_PER_REV_Z)
#define MAX_STEP_FREQ_E				(uint32_t)(MAX_REV_SPEED_E * STEPS_PER_REV_E)

/// used for G0 rapid moves and as a cap for all other feedrates [mm / min].
#define MAXIMUM_FEEDRATE_X			(uint32_t)(60 * MAX_REV_SPEED_X * FEED_PER_REV_X)
#define MAXIMUM_FEEDRATE_Y			(uint32_t)(60 * MAX_REV_SPEED_Y * FEED_PER_REV_Y)
#define MAXIMUM_FEEDRATE_Z			(uint32_t)(60 * MAX_REV_SPEED_Z * FEED_PER_REV_Z)
#define MAXIMUM_FEEDRATE_E			(uint32_t)(60 * MAX_REV_SPEED_E * FEED_PER_REV_E)

/// used when searching endstops and as default feedrate [mm / min].
#define SEARCH_FEEDRATE_X			(uint32_t)((SEARCH_FEED_FRACTION_X) * MAXIMUM_FEEDRATE_X)
#define SEARCH_FEEDRATE_Y			(uint32_t)((SEARCH_FEED_FRACTION_Y) * MAXIMUM_FEEDRATE_Y)
#define SEARCH_FEEDRATE_Z			(uint32_t)((SEARCH_FEED_FRACTION_Z) * MAXIMUM_FEEDRATE_Z)
// no SEARCH_FEEDRATE_E, as E can't be searched

/// Step period definitions for homing code

// beware to use only partial axis speed when approaching the switch because
// we've got no acceleration implemented here!
#define HOME_FEED			600		/*[mm/min]*/
// use fixed feed of 120 [mm/min] to run from the switch (assume any
// axis can run at this speed)
#define RELEASE_FEED		 20		/*[mm/min]*/
#define RELEASE_DISTANCE	2.0 	/*[mm]*/

#define LIMIT_FEED( feed, limit)	( ((feed) < (limit)) ? (feed) : (limit) )

#define HOME_FEED_FAST_X			LIMIT_FEED( (HOME_FEED), SEARCH_FEEDRATE_X)
#define HOME_FEED_FAST_Y			LIMIT_FEED( (HOME_FEED), SEARCH_FEEDRATE_Y)
#define HOME_FEED_FAST_Z			LIMIT_FEED( (HOME_FEED), SEARCH_FEEDRATE_Z)

#define HOME_FEED_SLOW_X			LIMIT_FEED( (RELEASE_FEED), SEARCH_FEEDRATE_X)
#define HOME_FEED_SLOW_Y			LIMIT_FEED( (RELEASE_FEED), SEARCH_FEEDRATE_Y)
#define HOME_FEED_SLOW_Z			LIMIT_FEED( (RELEASE_FEED), SEARCH_FEEDRATE_Z)

#define HOME_FAST_STEP_PERIOD_X		FEED_TO_US_STEP( X, HOME_FEED_FAST_X)
#define HOME_FAST_STEP_PERIOD_Y		FEED_TO_US_STEP( Y, HOME_FEED_FAST_Y)
#define HOME_FAST_STEP_PERIOD_Z		FEED_TO_US_STEP( Z, HOME_FEED_FAST_Z)

#define HOME_SLOW_STEP_PERIOD_X		FEED_TO_US_STEP( X, HOME_FEED_SLOW_X)
#define HOME_SLOW_STEP_PERIOD_Y		FEED_TO_US_STEP( Y, HOME_FEED_SLOW_Y)
#define HOME_SLOW_STEP_PERIOD_Z		FEED_TO_US_STEP( Z, HOME_FEED_SLOW_Z)

// G.P. conversion macros
#define FEED_TO_US_STEP( axis, feed)	(uint32_t)(60000000 / ((feed) * (STEPS_PER_MM_ ## axis)))
#define US_STEP_TO_FEED( axis, period)	(uint32_t)(60000000 / ((period) * (STEPS_PER_MM_ ## axis)))
#define MIN_CLOCKS_PER_STEP( axis)		(uint32_t)(F_CPU / MAX_STEP_FREQ_ ## axis)
#define	STEPS_TO_UM( axis, steps)		(uint32_t)(1000L * (steps)) / ((uint32_t) STEPS_PER_MM_ ## axis)

// good old HP35 had no PI, so I'll never forget these numbers :-)
#define PI               			((float) 355 / 113)

#define MIN_CLOCKS_PER_STEP_X		MIN_CLOCKS_PER_STEP( X)
#define MIN_CLOCKS_PER_STEP_Y		MIN_CLOCKS_PER_STEP( Y)
#define MIN_CLOCKS_PER_STEP_Z		MIN_CLOCKS_PER_STEP( Z)
#define MIN_CLOCKS_PER_STEP_E		MIN_CLOCKS_PER_STEP( E)

#endif // _CONFIG_MACROS_H

