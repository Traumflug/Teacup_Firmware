
#ifndef _SENDF_H
#define _SENDF_H

#include "config_wrapper.h"


// No __attribute__ ((format (printf, 1, 2)) here because %q isn't supported.
void sendf_F(void (*writechar)(uint8_t), const __flash char *format_F, ...);

#endif /* _SENDF_H */
