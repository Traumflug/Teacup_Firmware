/** \file
  \brief Dynamic Z-height compensation for out-of-level print bed.
*/

#include "bed_leveling.h"

#ifdef BED_LEVELING

#include <stdint.h>
#include <stdlib.h>

#include "dda_maths.h"
#include "debug.h"
#include "sersendf.h"
#include "config_wrapper.h"

struct point {
  int32_t x, y, z;
};

// Accept exactly three points for bed leveling
static uint8_t level_index = 0;
static struct point bed_level_map[3];

// Alias the three points
static struct point const * const P = &bed_level_map[0];
static struct point const * const Q = &bed_level_map[1];
static struct point const * const R = &bed_level_map[2];

int bed_plane_calculate(void);

// Reset the bed level values to "unknown"
void bed_level_reset() {
  level_index = 0;
}

// Scale x and y down to tenths of mm to keep math from overflowing
#define SCALE 100
static int32_t scale_to_dum(int32_t a) {
  if (a<0) return (a-SCALE/2)/SCALE;
  return (a+SCALE/2)/SCALE;
}

// Register a point on the bed plane; coordinates in um
void bed_level_register(int32_t x, int32_t y, int32_t z) {
  // Scale x and y to tenths of mm;  keep z in um
  x = scale_to_dum(x);
  y = scale_to_dum(y);

  // Find a previous registered point at the same location or use a new location
  struct point * p = bed_level_map;
  int i = 0;
  for (; i < level_index; i++, p++) {
    if (p->x == x && p->y == y)
      break;
  }

  // We can only handle three points
  if (i >= 3)
    return;

  p->x = x;
  p->y = y;
  p->z = z;

  // Bump the index if we just used a new location
  if (i == level_index)
    ++level_index;

  // Nothing more to do until we have three points
  if (level_index < 3)
    return;

  // We have three points. Try to calculate the plane of the bed.
  if (!bed_plane_calculate())
    --level_index;
}

// Pre-scaled coefficients of the planar equation,
//   Ax + By + Cz + K= 0
//
// When we have these coefficients, we're only going to use them relative to -1/C, so
//   Ac = -A/C;  Bc = -B/C;  Kc = 0 (because we translate a point to the origin)
//   f(x,y) = z = Ac*x + Bc*y + Kc
static uint32_t Aq, Ar, Bq, Br, C;
static int8_t Asign, Bsign;

int bed_leveling_active() {
  // No adjustment if not calibrated yet
  return level_index == 3;
}

void bed_level_report() {
  sersendf_P(PSTR("Bed leveling status:"));
  if (!bed_leveling_active()) {
    sersendf_P(PSTR(" not"));
  }
  sersendf_P(PSTR(" active (%d) positions registered\n"), level_index);
  for (int i = 0; i < level_index; i++) {
    sersendf_P(PSTR("  %d: G29 S1 X%lq Y%lq Z%lq\n"),
                    i+1, bed_level_map[i].x * SCALE, bed_level_map[i].y * SCALE, bed_level_map[i].z);
  }
}

int32_t bed_level_adjustment(int32_t x, int32_t y) {
  int32_t za, zb;

  // No adjustment if not calibrated yet
  if (!bed_leveling_active())
    return 0;

  x = scale_to_dum(x);
  y = scale_to_dum(y);

  x -= P->x;
  y -= P->y;

  za = muldivQR(x, Aq, Ar, C);
  if (Asign)
    za = -za;

  zb = muldivQR(y, Bq, Br, C);
  if (Bsign)
    zb = -zb;

  return P->z - za - zb;
}


int bed_plane_calculate() {
  // Coefficients of the planar equation,
  //   Ax + By + Cz + K = 0
  int32_t a, b, c;

  // Draw two vectors on the plane, u = B-A and v = C-A
  int32_t Ui, Uj, Uk, Vi, Vj, Vk;

  // U = vector(QP)
  Ui = Q->x - P->x;
  Uj = Q->y - P->y;
  Uk = Q->z - P->z;

  // V = vector(RP)
  Vi = R->x - P->x;
  Vj = R->y - P->y;
  Vk = R->z - P->z;

  // Find normal vector (a,b,c) = (U x V) and is perpendicular to the plane
  a = Uj*Vk - Uk*Vj;
  b = Uk*Vi - Ui*Vk;
  c = Ui*Vj - Uj*Vi;

  // Notes:
  //  * Ignore K (constant) by translating plane to pass through origin at P (0,0,0)
  //  * if a==0 and b==0, the bed is already level; z-offset is still important
  //  * if c==0 the bed is perpendicular or the points are colinear
  if (c == 0)
    return 0;

  if (DEBUG_DDA && (debug_flags & DEBUG_DDA))
    sersendf_P(PSTR("Coefficients: A:%ld B:%ld C:%ld\n"), a, b, c);

  // muldiv requires positive parameters
  // remember the signs separately
  Asign = a < 0;
  Bsign = b < 0;
  if (Asign) a = -a;
  if (Bsign) b = -b;

  // Combine A/C and B/C, so combine their signs, too
  if (c < 0) {
    c = -c;
    Asign = !Asign;
    Bsign = !Bsign;
  }

  // Pre-calc the coefficients A/C and B/C
  Aq = a / c;
  Ar = a % c;
  Bq = b / c;
  Br = b % c;
  C = c;

  int ret = 1;
  // Sanity check
  for (int i = 0; i < level_index; i++) {
    int32_t x=bed_level_map[i].x * SCALE, y=bed_level_map[i].y * SCALE;
    int32_t validation = bed_level_adjustment(x, y);
    if (labs(validation - bed_level_map[i].z) > 10) {
      sersendf_P(PSTR("!! Bed plane validation failed: Point %d: X:%lq Y:%lq  Expected Z:%lq  Calculated Z:%lq\n"),
                      i+1, x, y, bed_level_map[i].z, validation);
      ret = 0; // invalidate results on error
    }
  }

  return ret;
}

#endif /* BED_LEVELING */
