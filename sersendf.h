#ifndef	_SERSENDF_H
#define	_SERSENDF_H

#ifndef SIMULATOR
#include	<avr/pgmspace.h>
#endif
#include "simulator.h"

#if !defined (DO_NOT_CHECK_FORMATS)
#define FORMAT_ATTRIBUTE __attribute__ ((format (printf, 1, 2)))
#else 
#define FORMAT_ATTRIBUTE 
#endif

void sersendf(char *format, ...)	FORMAT_ATTRIBUTE;
void sersendf_P(PGM_P format_P, ...)	FORMAT_ATTRIBUTE;

#endif	/* _SERSENDF_H */
