#!/usr/bin/awk -f
BEGIN { firstline = 1 }

# Calculates the first derivative of columns 2 and 3 wrt column 1
$0 !~ /^#/ {
  t = $1
  x = $2
  y = $3
  if (firstline == 1) {
    old_tx = old_ty = t
    old_x = x
    old_y = y
    dy_dt = dx_dt = 0
    firstline = 0
  } else {
    dx = old_x - x
    dy = old_y - y

    if ( dx != 0) {
      dt = old_tx - t
      dx_dt = dx/dt
      old_tx = t
      old_x = x
    }
    if ( dy != 0) {
      dt = old_ty - t
      dy_dt = dy/dt
      old_ty = t
      old_y = y
    }
    if ( dx != 0 || dy != 0 ) {
      print t, " ", x, " ", y, " ", dx_dt, " ", dy_dt
    }
  }
}
