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
#define RELEASE_FEED		120		/*[mm/min]*/
#define RELEASE_DISTANCE	5.0 	/*[mm]*/

#define HOME_FAST_STEP_PERIOD_X			(uint32_t) FEED_STEP_IN_US( X, HOME_FEED)
#define HOME_SLOW_STEP_PERIOD_X			(uint32_t) FEED_STEP_IN_US( X, RELEASE_FEED)
#define HOME_FAST_STEP_PERIOD_Y			(uint32_t) FEED_STEP_IN_US( Y, HOME_FEED)
#define HOME_SLOW_STEP_PERIOD_Y			(uint32_t) FEED_STEP_IN_US( Y, RELEASE_FEED)
#define HOME_FAST_STEP_PERIOD_Z			(uint32_t) FEED_STEP_IN_US( Z, HOME_FEED)
#define HOME_SLOW_STEP_PERIOD_Z			(uint32_t) FEED_STEP_IN_US( Z, RELEASE_FEED)

// G.P. conversion macros
#define FEED_TO_US_STEP( axis, feed)	(60000000 / ((feed) * STEPS_PER_MM_ ## axis))
#define FEED_STEP_IN_US( axis, feed)	FEED_TO_US_STEP( axis, (feed < SEARCH_FEEDRATE_ ## axis) ? feed : SEARCH_FEEDRATE_ ## axis)

#define EXTRUSION_GAIN				((FILAMENT_DIAM_IN * FILAMENT_DIAM_IN) \
									 / (FILAMENT_DIAM_OUT * FILAMENT_DIAM_OUT))
// good old HP35 had no PI, so I'll never forget these numbers :-)
#define PI               			((float) 355 / 113)


#endif // _CONFIG_MACROS_H

