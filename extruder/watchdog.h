#ifndef	_WATCHDOG_H
#define	_WATCHDOG_H

#include "config_wrapper.h"

#ifdef USE_WATCHDOG

// initialize
void wd_init(void) __attribute__ ((cold));

// reset timeout- must be called periodically or we reboot
void wd_reset(void);

// notable lack of disable function

#else /* USE_WATCHDOG */

#define wd_init()  /* empty */
#define wd_reset() /* empty */

#endif	/* USE_WATCHDOG */
#endif	/* _WATCHDOG_H */
