#!/bin/sh

set -e

TMPDIR=/tmp

function die() {
  echo "$1" 1>&2
  exit 1
}

test -d "${TMPDIR}" || die "${TMPDIR} is not a valid directory"

for STEPS_PER_MM in 50 500 1500 ; do
  rm -f "${TMPDIR}"/test-F=*.plot

  #for LOOKAHEAD in false true ; do
  for LOOKAHEAD in false ; do
    for ACCELERATION in {10..410..200} ; do
      TEST="A=$ACCELERATION.L=$LOOKAHEAD"

      printf "=======[ %s ]======\n" $TEST

      (
        printf "#define ACCELERATION_RAMPING\n"
        printf "#define ACCELERATION %s.\n" ${ACCELERATION}
        if $LOOKAHEAD ; then
          printf "#define LOOKAHEAD\n"
        fi
        printf "\n#define STEPS_PER_MM %s\n" $STEPS_PER_MM
        printf "//__________________________________________\n"
        cat testcases/velocity-test.h
      ) > config.h

      make -f Makefile-SIM

      for FEEDRATE in {300..599..100}; do
        PLOTFILE=${TMPDIR}/test-F=${FEEDRATE}.L=${LOOKAHEAD}.SPMM=${STEPS_PER_MM}.plot
        PLOT=,
        if ! test -e "$PLOTFILE"; then
          PLOT=plot
          printf "set title 'FEEDRATE=%s  LOOKAHEAD=%s  STEPS_PER_MM=%s\n" $FEEDRATE $LOOKAHEAD $STEPS_PER_MM > ${PLOTFILE}
        fi

        TRACE=${TEST}.F=${FEEDRATE}
        test -e ${TMPDIR}/test.gcode && rm ${TMPDIR}/test.gcode
        for DISTANCE in {30..200..30}; do
          printf "G1 X%s F%s\n" $DISTANCE $FEEDRATE >> ${TMPDIR}/test.gcode
        done
        printf "M2\n" >> ${TMPDIR}/test.gcode
        ./sim ${TMPDIR}/test.gcode --tracefile=${TMPDIR}/${TRACE} --time-scale=0 --gcode
        testcases/deriv.awk ${TMPDIR}/${TRACE} > ${TMPDIR}/${TRACE}.velocity
        printf "%s '%s.velocity' u (\$1):(\$2/%s.0) with lines t 'A=%s'" "$PLOT" "$TRACE" "$STEPS_PER_MM" "$ACCELERATION" >> ${PLOTFILE}
      done
    done
  done

  (
    cd "${TMPDIR}"
    for plot in test-F=*.plot; do
      gnuplot --persist -e "load '$plot'"
    done
  )

done
