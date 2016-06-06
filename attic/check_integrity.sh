#!/bin/bash

# Check all our config files for integrity. We often had the case that a
# configuration #define was added, which is entirely fine, but it was forgotten
# to distribute this over all the relevant files.
#
# Maybe it's not too exciting form the single-source-of-information-POV, but
# Teacup maintains configuration items redundant in all the configuration
# files. Point of doing so is to support manual config file editing as well as
# using Configtool.

# Make sure we're inside config/
cd config 2> /dev/null
if [ $(basename ${PWD}) != "config" ]; then
  echo "Execute this from within config/."
  exit 1
fi

# We can't abort out of subshells, so track our success with a temporary file.
EXITFILE=$(mktemp)
trap "rm -f ${EXITFILE}" 0
echo 0 > ${EXITFILE}

# Check both, board and printer configurations.
for T in "board" "printer"; do
  # Test #1: are all #defines in the generic config in all the individual
  #          files?
  awk '
    /^#define/ {
      print $2;
    }
    /^\/\/#define/ {
      print $2;
    }
  ' < ../configtool/${T}.generic.h | while read W; do
    for F in $(git ls-files ${T}.\*.h ../testcases/config.h.Profiling); do
      if ! grep "#define" ${F} | grep -q ${W}; then
        echo "Missing #define ${W} in ${F}."
        echo 1 > ${EXITFILE}
      fi
    done
  done

  # Test #2: the opposite, has the generic config all of the latest three
  #          individual configs?
  for F in $(ls -t ${T}.*.h | head -3); do
    awk '
      /^\/\/DEFINE_HEATERS_START/, /^\/\/DEFINE_HEATERS_END/ {
        # This section is created on the fly, so not in the generic file.
        next;
      }
      /^#define/ {
        print $2;
      }
      /^\/\/#define/ {
        print $2;
      }
    ' < ${F} | while read W; do
      if ! grep "#define" ../configtool/${T}.generic.h | grep -q ${W}; then
        echo "Missing #define ${W} in configtool/${T}.generic.h."
        echo 1 > ${EXITFILE}
      fi
    done
  done
done

EXIT=$(cat ${EXITFILE})

if [ ${EXIT} -ne 0 ]; then
  echo "Config integrity tests failed."
else
  echo "Config integrity tests succeeded."
fi

exit ${EXIT}
