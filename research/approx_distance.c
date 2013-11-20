#include <stdlib.h>
#include <stdio.h>
#include "dda_maths.h"
#include <math.h>

double actual2d(uint64_t dx , uint64_t dy) {
  return sqrt(dx*dx + dy*dy);
}

double actual3d(uint64_t dx , uint64_t dy, uint64_t dz) {
  return sqrt(dx*dx + dy*dy + dz*dz);
}

void main( void ) {
  uint32_t dx, dy, dz;
  float min_err = 1.0, max_err = 1.0 ;
  uint32_t count = 0;

  printf("===[ 2D test ]===\n");
  for ( dx = 0 ; dx <= 20000 ; dx+= 5 )
    for ( dy = 0 ; dy <= 20000 ; dy+= 5 ) {
        uint32_t ad2 = approx_distance( dx, dy ) ;
        double ac2 = actual2d( dx, dy ) ;
        double err = ((double)ad2)/((double)ac2);
        ++count ;
        // Calculate max error which is not attributable to integer truncation
        if ((int)(ac2-ad2)) {
          if ( err < min_err ) min_err = err;
          if ( err > max_err ) max_err = err;
        //  if ( err > 1.0025 || err < 0.9975 )
        //    printf("dx=%d dy=%d  approx=%u / actual=%f  ratio=%lf  diff=%u\n", dx, dy, ad2, ac2, err, abs(ac2-ad2)) ;
        }
    }
  printf("Samples: %u\n", count );
  printf("Max error: %f\n", max_err );
  printf("Min error: %f\n", min_err );

  count=0;
  min_err = 1.0;
  max_err = 1.0;
  printf("===[ 3D test ]===\n");
  for ( dx = 0 ; dx <= 20000 ; dx+= 10 )
    for ( dy = 0 ; dy <= 20000 ; dy+= 7 ) {
      for ( dz = 0 ; dz <= 200 ; dz+= 1 ) {
        uint32_t ad3 = approx_distance_3( dx, dy, dz ) ;
        double ac3 = actual3d( dx, dy, dz) ;
        double err = ((double)ad3)/((double)ac3);
        ++count ;
        // Calculate max error which is not attributable to integer truncation
        if ((int)(ac3-ad3)) {
          if ( err < min_err ) min_err = err;
          if ( err > max_err ) max_err = err;
        //  if ( abs(ac3-ad3)>1 && (err > 1.005 || err < 0.995 ))
        //    printf("dx=%d dy=%d dz=%d  approx=%u / actual=%f  ratio=%lf  diff=%u\n", dx, dy, dz, ad3, ac3, err, abs(ac3-ad3)) ;
        }
      }
    }

  printf("Samples: %u\n", count );
  printf("Max error: %f\n", max_err );
  printf("Min error: %f\n", min_err );
}
