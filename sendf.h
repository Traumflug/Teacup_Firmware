
#ifndef _SENDF_H
#define _SENDF_H

#include "config_wrapper.h"


// No __attribute__ ((format (printf, 1, 2)) here because %q isn't supported.
void sendf_P(void (*writechar)(uint8_t), PGM_P format_P, ...);

#endif /* _SENDF_H */
