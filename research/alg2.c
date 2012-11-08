
/*
   This is evaluation code, implementing algorithms for
   moving bezier curves and look-ahead making use of these
   bezier curves.

   Bezier algorithms found in:
   A Rasterizing Algorithm for Drawing Curves - Alois Zingl 2012
   http://free.pages.at/easyfilter/bresenham.pdf
*/

#include <stdlib.h>
#include <stdio.h>

#define ACCELERATION 10 // mm/s^2


typedef struct {
	int32_t						X;
	int32_t						Y;
	int32_t						Z;
	int32_t						E;
	u_int32_t					F;
} TARGET;

TARGET start = {    0,     0,     0,   100};
TARGET medium = {    0,     0,     0,   100};

void
setPixel (int x, int y) {
  printf("%d %d\n", x, y);
}

void
plotLine(int x0, int y0, int x1, int y1) {
  int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, e2;

  for (;;) {
    setPixel(x0, y0);
    e2 = 2 * err;
    if (e2 >= dy) {
      if (x0 == x1) break;
      err += dy; x0 += sx;
    }
    if (e2 <= dx) {
      if (y0 == y1) break;
      err += dx; y0 += sy;
    }
  }
}

void
plotBasicQuadBezier(int x0, int y0, int x1, int y1, int x2, int y2) {
  printf("P0 = %d, %d;  P1 = %d, %d;  P2 = %d, %d\n", x0, y0, x1, y1, x2, y2);
  int sx = x0 < x2 ? 1 : -1, sy = y0 < y2 ? 1 : -1; /* step direction */
  double x = x0 - 2 * x1 + x2, y = y0 - 2 * y1 + y2;
  double xy = 2 * x * y * sx * sy;
  double cur = sx * sy * (x * (y2 - y0) - y * (x2 - x0)) / 2; /* curvature */
  /* compute error increments of P0 */
  double dx = (1 - 2 * abs(x0 - x1)) * y * y + abs(y0 - y1) * xy - 2 * cur * abs(y0 - y2);
  double dy = (1 - 2 * abs(y0 - y1)) * x * x + abs(x0 - x1) * xy + 2 * cur * abs(x0 - x2);
  /* compute error increments of P2 */
  double ex = (1 - 2 * abs(x2 - x1)) * y * y + abs(y2 - y1) * xy + 2 * cur * abs(y0 - y2);
  double ey = (1 - 2 * abs(y2 - y1)) * x * x + abs(x2 - x1) * xy - 2 * cur * abs(x0 - x2);
  /* sign of gradient must not change */
  //~ assert ((x0 - x1) * (x2 - x1) <= 0 && (y0 - y1) * (y2 - y1) <= 0) {
    //~ printf("ERROR: sign of gradient changes\n");
    //~ return;
  //~ }

  if (cur == 0) { plotLine(x0, y0, x2, y2); return; } /* straight line */
  
  x *= 2 * x; y *= 2 * y;
  if (cur < 0) {
    /* negated curvature */
    x = -x; dx = -dx; ex = -ex; xy = -xy;
    y = -y; dy = -dy; ey = -ey;
  }
  /* algorithm fails for almost straight line, check error values */
  if (dx >= -y || dy <= -x || ex <= -y || ey >= -x) {
    x1 = (x0 + 4 * x1 + x2) / 6; y1 = (y0 + 4 * y1 + y2) / 6;
    plotLine(x0, y0, x1, y1);
    plotLine(x1, y1, x2, y2);
    return;
  }
  dx -= xy; ex = dx + dy; dy -= xy; /* error of 1.step */

  for(;;) { /* plot curve */
    setPixel(x0, y0);
    ey = 2  * ex - dy; /* save value for test of y step */
    if (2 * ex >= dx) { /* x step */
      if (x0 == x2) break;
      x0 += sx; dy -= xy; ex += dx += y;
    }
    if (ey <= 0) { /* y step */
      if (y0 == y2) break;
      y0 += sy; dx -= xy; ex += dy += x;
    }
  }
}

void
move(TARGET to, char stop) {
  static char first = 1;
  
  if (! first) {
    /* Now we have two segments. What we do is:
       - calculate the possible junction speed
       - shorten both paths by this
       - accelerate on the first segment (1. sub-segment)
       - move linearly on the first segment (2. sub-segment)
       - decelerate on the first segment to reach the junction curve (3. sub-segment)
       - move the curve constant speed (4. sub-segment)
       At this point we're done, unless we stop after the second segment.
       Stopping after the second segment additionally means:
       - accelerate on the second segment
       - move linearly on the second segment
       - decelerate to zero on the second segment.
    */
    /* Possible curve radius, circle approxximated.
         a = v^2 / r  <=>  r = v^2 / a */
    
    
    TARGET startB, endB;

    startB.X = start.X + (medium.X - start.X) * 3 / 4;
    startB.Y = start.Y + (medium.Y - start.Y) * 3 / 4;
    endB.X = medium.X + (to.X - medium.X) * 1 / 4;
    endB.Y = medium.Y + (to.Y - medium.Y) * 1 / 4;
    plotLine(start.X, start.Y, startB.X, startB.Y);
    plotBasicQuadBezier(startB.X, startB.Y, medium.X, medium.Y, endB.X, endB.Y);
  }
  start = medium;
  medium = to;
  first = 0;
}

int
main() {
  TARGET t1 = { 5000,  5000,     0,   100};
  TARGET t2 = { 5000, 10000,     0,   100};
  TARGET t3 = {10000, 10000,     0,   100};

  move(t1, 0);
  move(t2, 0);
  move(t3, 1);

  return 0;
}