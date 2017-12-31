#!/bin/bash

# Check that the output from configtool.py matches the input when no changes
# are made in the GUI. This requires that the distributed config files match
# the format of the configtool template files, but this is desired anyway.

# Stop on error.
set -e

# Create an output directory for verification.
OUTDIR="build/test"
rm -rf "${OUTDIR}"
mkdir -p "${OUTDIR}"

# Check board and printer configurations.
git ls-files "config/*.h" | while read IN; do

  # Use configtool.py to regenerate headers for comparison.
  OUT="${OUTDIR}"/$(basename "${IN}")
  ./configtool.py --load="${IN}" --save="${OUT}" --quit

  # Strip the "help text" comments from the source and output files.
  #
  # This should go away one day, but currently it avoids failures on the only
  # partially handled CANNED_CYCLE #define.
  perl -p0i -e 's#/\*.*?\*/##sg' "${OUT}"
  perl -p0 -e 's#/\*.*?\*/##sg' "${IN}" > "${OUT}.cmp"

  # Fail if the result is different except in whitespace.
  if ! diff -qBb "${OUT}" "${OUT}.cmp" ; then
    echo "Configtool integrity test failed on file ${IN}"
    echo "  Executed: ./configtool.py --load=\"${IN}\" --save=\"${OUT}\" --quit"
    echo "  Expected resulting settings to match, but they do not."

    diff -uBb "${OUT}" "${OUT}.cmp" | sed -e 's/^/    /' || :
    exit 1
  fi
done

rm -rf "${OUTDIR}"
