#ifndef	_SERSENDF_H
#define	_SERSENDF_H

#ifndef SIMULATOR
#include	<avr/pgmspace.h>
#endif
#include "simulator.h"

void sersendf(char *format, ...)		__attribute__ ((format (printf, 1, 2)));
void sersendf_P(PGM_P format, ...)	__attribute__ ((format (printf, 1, 2)));

#endif	/* _SERSENDF_H */
