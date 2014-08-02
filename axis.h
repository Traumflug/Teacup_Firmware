#ifndef _AXIS_H
#define _AXIS_H

#include <string.h>

#include "config_wrapper.h"

#include	"preprocessor_math.h"

#ifndef SIMULATOR
  #include <avr/pgmspace.h>
#else
  #define PROGMEM
#endif


// Enum to denote valid axises
enum axis_e {
#ifdef U_STEP_PIN
  U,
#endif
#ifdef V_STEP_PIN
  V,
#endif
#ifdef X_STEP_PIN
  X,
#endif
#ifdef Y_STEP_PIN
  Y,
#endif
#ifdef Z_STEP_PIN
  Z,
#endif
#ifdef E_STEP_PIN
  E,
#endif

  AXIS_COUNT,
  INVALID_AXIS
};

static const char axis_enum2char[] =
#ifdef U_STEP_PIN
  "U"
#endif
#ifdef V_STEP_PIN
  "V"
#endif
#ifdef X_STEP_PIN
  "X"
#endif
#ifdef Y_STEP_PIN
  "Y"
#endif
#ifdef Z_STEP_PIN
  "Z"
#endif
#ifdef E_STEP_PIN
  "E"
#endif
;

/**
  \typedef axes_uint32_t
  \brief n-dimensional vector used to describe uint32_t axis information.

  Stored value can be anything unsigned. Units should be specified when declared.
*/
typedef uint32_t axes_uint32_t[AXIS_COUNT];

/**
  \typedef axes_int32_t
  \brief n-dimensional vector used to describe int32_t axis information.

  Stored value can be anything unsigned. Units should be specified when declared.
*/
typedef int32_t axes_int32_t[AXIS_COUNT];

/// \var steps_per_m_P
/// \brief motor steps required to advance one meter on each axis
static const axes_uint32_t PROGMEM steps_per_m_P = {
#ifdef U_STEP_PIN
  STEPS_PER_M_U,
#endif
#ifdef V_STEP_PIN
  STEPS_PER_M_V,
#endif
#ifdef X_STEP_PIN
  STEPS_PER_M_X,
#endif
#ifdef Y_STEP_PIN
  STEPS_PER_M_Y,
#endif
#ifdef Z_STEP_PIN
  STEPS_PER_M_Z,
#endif
#ifdef E_STEP_PIN
  STEPS_PER_M_E
#endif
};

/// \var maximum_feedrate_P
/// \brief maximum allowed feedrate on each axis
static const axes_uint32_t PROGMEM maximum_feedrate_P = {
#ifdef U_STEP_PIN
  MAXIMUM_FEEDRATE_U,
#endif
#ifdef V_STEP_PIN
  MAXIMUM_FEEDRATE_V,
#endif
#ifdef X_STEP_PIN
  MAXIMUM_FEEDRATE_X,
#endif
#ifdef Y_STEP_PIN
  MAXIMUM_FEEDRATE_Y,
#endif
#ifdef Z_STEP_PIN
  MAXIMUM_FEEDRATE_Z,
#endif
#ifdef E_STEP_PIN
  MAXIMUM_FEEDRATE_E
#endif
};

/// \var c0_P
/// \brief Initialization constant for the ramping algorithm. Timer cycles for
///        first step interval.
static const axes_uint32_t PROGMEM c0_P = {
#ifdef U_STEP_PIN
  (uint32_t)((double)F_CPU / SQRT((double)(STEPS_PER_M_U * ACCELERATION / 2000.))),
#endif
#ifdef V_STEP_PIN
  (uint32_t)((double)F_CPU / SQRT((double)(STEPS_PER_M_V * ACCELERATION / 2000.))),
#endif
#ifdef X_STEP_PIN
  (uint32_t)((double)F_CPU / SQRT((double)(STEPS_PER_M_X * ACCELERATION / 2000.))),
#endif
#ifdef Y_STEP_PIN
  (uint32_t)((double)F_CPU / SQRT((double)(STEPS_PER_M_Y * ACCELERATION / 2000.))),
#endif
#ifdef Z_STEP_PIN
  (uint32_t)((double)F_CPU / SQRT((double)(STEPS_PER_M_Z * ACCELERATION / 2000.))),
#endif
#ifdef E_STEP_PIN
  (uint32_t)((double)F_CPU / SQRT((double)(STEPS_PER_M_E * ACCELERATION / 2000.)))
#endif
};


/// Convert a char to a axis_e or returns INVALID_AXIS if no axis named c exists
static inline int axis_char2enum(char c)
{
    char *cp = strchr(axis_enum2char, c);
    return cp ? cp - axis_enum2char : INVALID_AXIS;
}

#endif //_AXIS_H
