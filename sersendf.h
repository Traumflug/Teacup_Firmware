#ifndef	_SERSENDF_H
#define	_SERSENDF_H

#include "arduino.h"


// No __attribute__ ((format (printf, 1, 2)) here because %q isn't supported.
void sersendf(char *format, ...);
void sersendf_P(PGM_P format_P, ...);

#endif	/* _SERSENDF_H */
