#!/usr/bin/awk -f
BEGIN { firstline = 1 }

# Calculates the first derivative of columns 2 and 3 wrt column 1
$0 !~ /^#/ {
  $1 /= 1000000.0
  if (firstline == 1) {
    for ( i=1; i<=NF; i++) {
      prev_t[i] = $1
      prev_x[i] = $i
      deriv[i] = 0
    }
    firstline = 0
  } else {
    printf $1
    for ( i=2; i<=NF; i++) {
      if ( $i != prev_x[i] ) {
        dx = $i - prev_x[i]
        dt = $1 - prev_t[i]
        deriv[i] = dx/dt
        prev_t[i] = $1
        prev_x[i] = $i
      }
      printf " %s %0f", $i, deriv[i]
    }
    print ""
  }
}
