#ifndef _PRUSA_MECH_H
#define _PRUSA_MECH_H

/// Drive train characteristics for PRUSA:
#define TEETH_ON_PULLEY_X			8
#define TEETH_ON_PULLEY_Y			8
#define TIMING_BELT_PITCH			5.0   		/* distance between two teeth */
#define THREAD_PITCH_Z   			1.25  		/* pitch of M8 thread on threaded rod */

/// The feed for one motor axis revolution [mm / rev].
#define FEED_PER_REV_X				(double)(TEETH_ON_PULLEY_X * TIMING_BELT_PITCH)
#define FEED_PER_REV_Y				(double)(TEETH_ON_PULLEY_Y * TIMING_BELT_PITCH)
#define FEED_PER_REV_Z				(double)(THREAD_PITCH_Z)

/// Mechanical dimenstions [mm]
#define	AXIS_TRAVEL_X				200.0
#define	AXIS_TRAVEL_Y				200.0
#define	AXIS_TRAVEL_Z				140.0

#endif //  _PRUSA_MECH_H

