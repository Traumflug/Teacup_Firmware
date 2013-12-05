#!/bin/sh

TMPDIR=/tmp
TOOLSDIR=$(dirname $0)

# Set this to your steps-per-mm value, or use 1 to keep results in steps
MM_PER_STEP=500.0

die() {
	echo "$1"
	exit 1
}

test -n "$1" || die "Usage: $0 <tracefile-name>"
test -f "$1" || die "Error: $1 does not exist or is not a regular file"

FILE="$1"
VELOC="$TMPDIR/$(basename "$FILE").velocity"

"${TOOLSDIR}/deriv.awk" "$FILE" > "$VELOC"

gnuplot --persist -e "
  set ylabel 'Position';
  set y2label 'Velocity (mm/minute)'; set ytics nomirror ; set y2tics;
  set samples 10000 ;

  plot '${VELOC}' u (\$1/1000000):(\$2/$MM_PER_STEP) with lines t \"X-position\" , '' u (\$1/1000000):(\$3/$MM_PER_STEP) with lines t \"Y-position\"
     , '' u (\$1/1000000):(60000000.0*\$4/$MM_PER_STEP) with lines t \"X-velocity\" axes x1y2
     , '' u (\$1/1000000):(60000000.0*\$5/$MM_PER_STEP) with lines t \"Y-velocity\" axes x1y2
"
