#!/usr/bin/awk -f
BEGIN { firstline = 1 }

$0 !~ /^#/ {
  # Adjust time from microseconds to minutes
  x = $1/1000000.0 / 60.0
  y = $2
  if (firstline == 1) {
    old_x = x
    old_y = y
    firstline = 0
  } else {
    n = old_y - y
    d = old_x - x
    if ( d != 0) {
      print (old_x + x)/2.0, "\t", (n/d)
      old_x = x
      old_y = y
    }
  }
}
