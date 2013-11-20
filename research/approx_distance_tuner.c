#include <stdlib.h>
#include <stdio.h>
#include "dda_maths.h"
#include <math.h>

/**
  This tool tries to improve the original selected sin/cos values by comparing the
  "real" distance formula with our fast integer version and examining the distribution
  of measured errors.  It wanders around the neighborhood of each value pair in a
  brute-force way looking for replacement values which give an improved clustering of
  errors (approaching zero).  The clustering is determined by comparison of standard
  deviations.  We arbitrarily use 2sd from the mean to determine our "max" and "min"
  error values, but this probably is unnecessary.

  We also report when we finds an improved "mean" but we do not use it because this
  turns out to be a lesser predicter.

  It is designed to be run multiple times in case one set of numbers leads to a new
  neigborhood we want to explore.  But in practice the 2nd iteration produced the same
  numbers as the first, so I stopped there.
*/

//================================================================================

// Original values suggested by approximate_distance.plot
//static const uint16_t orig_xscale[8] = { 1023, 1008, 979, 941, 898, 853, 807, 757 };
//static const uint16_t orig_yscale[8] = { 65, 187, 303, 406, 494, 568, 632, 691 };

// First iteration of tuning
static uint16_t orig_xscale[8] = { 1023, 1007, 979, 938, 893, 845, 794, 742 };
static uint16_t orig_yscale[8] = { 58, 189, 302, 412, 502, 579, 647, 706 };

// Second iteration of tuning
static uint16_t xscale[8] = { 1023, 1007, 979, 938, 893, 845, 794, 742 };
static uint16_t yscale[8] = { 58, 189, 302, 412, 502, 579, 647, 706 };

uint32_t approx_distance(uint32_t dx, uint32_t dy) {
  uint32_t min, max, approx;
  uint32_t minx, max_2, max_4, max_8;
  int n;

  if ( dx < dy ) {
    min = dx;
    max = dy;
  } else {
    min = dy;
    max = dx;
  }

  if ( min < 2) return max ;

  // Quickie divide: n = min(7 , 8 * ( min / max ) )
  n = 0;
  minx = min;
  max_2 = (  max + 1) >> 1;  /* max/2 */
  max_4 = (max_2 + 1) >> 1;  /* max/4 */
  max_8 = (max_4 + 1) >> 1;  /* max/8 */
  if (minx > max_2) {
    minx -= max_2 ;
    n += 4;
  }
  if (minx > max_4) {
    minx -= max_4 ;
    n += 2;
  }
  if (minx > max_8) {
    n += 1;
  }

  approx = ( max * xscale[n] ) + ( min * yscale[n] );

  // add 512 for proper rounding, then divide by 1024
  return (( approx + 512 ) >> 10 );
}

//================================================================================

double actual2d(uint64_t dx , uint64_t dy) {
  return sqrt(dx*dx + dy*dy);
}

double actual3d(uint64_t dx , uint64_t dy, uint64_t dz) {
  return sqrt(dx*dx + dy*dy + dz*dz);
}

double mean, sigma;
double acc, accsq;
int nSamples ;

void sd_init(void) {
	mean = 0;
	nSamples = 0;
	sigma = 0;
	acc = 0;
	accsq = 0;
}

void sd_accumulate(float d) {
	nSamples++;
	double acc1 = acc + (d - acc) / nSamples;
	accsq += (d - acc)*(d-acc1);
	acc = acc1;
}

void sd_final() {
	mean = acc ;
	sigma = sqrt(accsq / nSamples) ;
}

// Find the average, worst and SD of errors in a range of samples over the "nth eighth".
double sd(unsigned int n) {
  int dx, dy;

  sd_init();
  for ( dx = 5 ; dx <= 2000 ; dx+= 5 ) {
    int bottom = dx * n / 8 ;
    int top = bottom + dx / 8 - 1 ;
    for ( dy = bottom ; dy <= top ; dy+= 1 ) {
        uint32_t ad2 = approx_distance( dx, dy ) ;
        float ac2 = actual2d( dx, dy ) ;
        if (ac2) {
	  float err = ((float)ad2)/((float)ac2) ;
	  sd_accumulate(err);
	}
     }
  }
  sd_final();

  return fabs(1 - mean) + 2 * sigma ;
}

void show(int i) {
  float a = mean - 2 * sigma ;
  float b = mean + 2 * sigma ;
  printf("%d: %4u %4u  mean: %lf sigma: %lf 2*sigma: %lf..%lf\n", i, xscale[i], yscale[i], mean, sigma , a , b );
}

void main( void ) {
  int target=1;
  int xs, ys;
  double best_sigma, best_mean, best;
  int best_xs, best_ys;

  printf("--------------------\n");
  for (target=0; target < 8; target++ ) {
   best = sd(target);
   show(target);
  }
  printf("--------------------\n");

  for (target=0; target < 8; target++ ) {
   uint16_t sx = xscale[target], sy = yscale[target] ;
   best_xs = sx ;
   best_ys = sy ;

   if ( sx == orig_xscale[target] && sy == orig_yscale[target] ) {
     printf("Already tuned.\n");
     continue;
   }

   best = sd(target);
   best_sigma = sigma;
   best_mean = mean;

   printf("---[ %d ]---\n", target);
   show(target);

   for ( xs = -90; xs < 90 ; xs++ ) {
    xscale[target] = sx + xs ;
    for ( ys = -90; ys < 90 ; ys++ ) {
      float nx;
      yscale[target] = sy + ys ;
      nx = sd(target);
      if ( nx < best ) {
	printf(" %lf => %lf: ", best, nx); show(target);
        best = nx;
	best_xs = sx + xs;
	best_ys = sy + ys;
      }
      if ( sigma < best_sigma ) {
        best_sigma = sigma;
	//printf("sigma: "); show(target);
      }
      if ( fabs(1.0-mean) < fabs(1.0-best_mean) ) {
	best_mean = mean ;
	printf("mean:  "); show(target);
      }
    }
   }

   xscale[target] = best_xs;
   yscale[target] = best_ys;
  }


  printf("--------------------\n");
  for (target=0; target < 8; target++ ) {
   best = sd(target);
   show(target);
  }
  printf("--------------------\n");

  for (target=0; target < 8; target++ ) printf("%u, ", xscale[target]);
  printf("\n");
  for (target=0; target < 8; target++ ) printf("%u, ", yscale[target]);
  printf("\n");
}
