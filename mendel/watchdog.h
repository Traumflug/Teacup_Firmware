#ifndef	_WATCHDOG_H
#define	_WATCHDOG_H

// initialize
void wd_init(void);

// reset timeout- must be called periodically or we reboot
void wd_reset(void);

// notable lack of disable function

#endif	/* _WATCHDOG_H */
