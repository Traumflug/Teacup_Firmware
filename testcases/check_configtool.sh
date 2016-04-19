#!/bin/bash

# Check that the output from configtool.py matches the input when no changes
# are made in the GUI. This requires that the distributed config files match
# the format of the configtool template files, but this is desired anyway.

# Stop on error
set -e

# Create an output directory for verification
OUTDIR=build/test
rm -rf ${OUTDIR}
mkdir -p ${OUTDIR}

EXITCODE=0

# Check board and printer configurations.
for IN in $(git ls-files config/*.h); do

  # Use configtool.py to regenerate headers for comparison
  OUT=${OUTDIR}/$(basename ${IN})
  ./configtool.py --load=${IN} --save=${OUT} --done

  # Strip the "help text" comments from the source and output files
  perl -p0i -e 's#/\*.*?\*/##sg' ${OUT}
  perl -p0 -e 's#/\*.*?\*/##sg' ${IN} > ${OUT}.cmp

  # Fail if the result is different except in whitespace
  if ! diff -qBbw ${OUT} ${OUT}.cmp ; then
    echo "Configtool integrity test failed on file ${IN}"
    diff -Bbw ${OUT} ${OUT}.cmp || :
    EXITCODE=1
  fi
done
exit ${EXITCODE}
