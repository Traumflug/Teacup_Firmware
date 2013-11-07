#include <stdlib.h>
#include <stdio.h>
#include "dda_maths.h"
#include <math.h>

double actual2d(uint64_t dx , uint64_t dy) {
  return sqrt(dx*dx + dy*dy);
}

void main( void ) {
  uint32_t dx, dy, dz;
  float min_err = 1.0, max_err = 1.0 ;
  uint32_t count = 0;

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
          if ( err > 1.0025 || err < 0.9975 )
            printf("dx=%d dy=%d  approx=%u / actual=%f  ratio=%lf  diff=%u\n", dx, dy, ad2, ac2, err, abs(ac2-ad2)) ;
        }
    }

  printf("Samples: %u\n", count );
  printf("Max error: %f\n", max_err );
  printf("Min error: %f\n", min_err );
}
