#ifndef _WADES_EXTRUDER_H
#define _WADES_EXTRUDER_H

/// Drive train characteristics for Wade's extruder:
#ifndef EXTRUDER_FEED_AXIS_DIAM
// Give reasonable default if not specified in config file.
#	define EXTRUDER_FEED_AXIS_DIAM	7.6   	/* effective diameter of hobbed axis (nom.) */
#endif

#define EXTRUDER_REDUCTION	((double)39 / 11)  	/* for geared Wade extruder */

/// The feed for one motor axis revolution [mm / rev].
/// http://blog.arcol.hu/?p=157 may help with this one
#define FEED_PER_REV_E		(double)(PI * EXTRUDER_FEED_AXIS_DIAM / EXTRUDER_REDUCTION)

#endif // _WADES_EXTRUDER

