#!/bin/bash

# This should point to a SIMINFO-enabled SimulAVR executable:
SIMULAVR="../../simulavr/src/simulavr"
if [ ! -x "${SIMULAVR}" ]; then
  echo "SimulAVR executable not found, please adjust"
  echo "variables at the top of ${0}."
  exit 1;
fi

if [ $# -eq 0 ]; then
  BASE=$(basename $0)
  echo "Usage: ${BASE} <gcode file> ..."
  echo
  echo "${BASE} runs each G-code file in the simulator, limited to 60 seconds"
  echo "simulation time (can take much more wall clock time) and processes the"
  echo "results into a PNG picture and a .vcd file with calculated velocities."
  exit 1
fi


# General preparation.
PIPE_IN_FILE=$(mktemp -u)
PIPE_OUT_FILE=$(mktemp -u)
TRACEIN_FILE=$(mktemp)
STATISTICS_FILE=$(mktemp)

trap 'cat '${STATISTICS_FILE}'; rm -f '${PIPE_IN_FILE}' '${PIPE_OUT_FILE}' \
     '${TRACEIN_FILE}' '${STATISTICS_FILE} 0

mkfifo ${PIPE_IN_FILE} || exit 1
mkfifo ${PIPE_OUT_FILE} || exit 1


# Respect USER_CONFIG.
if [ "${USER_CONFIG}" ]; then
  CONFIG="${USER_CONFIG}"
  TEACUP_ELF="${USER_CONFIG%.h}/teacup.elf"
  TEACUP_ELF="../build/${TEACUP_ELF#../}"
else
  CONFIG="../config.h"
  TEACUP_ELF="../build/teacup.elf"
fi
echo "Taking configuration in ${CONFIG}."
echo "Taking Teacup binary ${TEACUP_ELF}."

# Prepare statistics.
echo                             > ${STATISTICS_FILE}
(cd .. && echo make USER_CONFIG="${USER_CONFIG}" size) | \
  tail -4                       >> ${STATISTICS_FILE}


# Prepare a pin tracing file, assuming a Gen7-v1.4 configuration. See
# http://reprap.org/wiki/SimulAVR#Putting_things_together:_an_example
#
#   #define X_DIR_PIN        DIO28
#   #define X_STEP_PIN       DIO29
#   #define Y_DIR_PIN        DIO26
#   #define Y_STEP_PIN       DIO27
#   #define DEBUG_LED_PIN    DIO21
echo "# X Dir"              > ${TRACEIN_FILE}
echo "+ PORTA.A3-Out"      >> ${TRACEIN_FILE}
echo "# X Step"            >> ${TRACEIN_FILE}
echo "+ PORTA.A2-Out"      >> ${TRACEIN_FILE}
echo "# Y Dir"             >> ${TRACEIN_FILE}
echo "+ PORTA.A5-Out"      >> ${TRACEIN_FILE}
echo "# Y Step"            >> ${TRACEIN_FILE}
echo "+ PORTA.A4-Out"      >> ${TRACEIN_FILE}
echo "# DEBUG LED"         >> ${TRACEIN_FILE}
echo "+ PORTC.C5-Out"      >> ${TRACEIN_FILE}
echo "Assuming pin configuration for a Gen7-v1.4 + debug LED on DIO21."

STEPS_PER_M_X=$(grep STEPS_PER_M_X ${CONFIG} | \
                grep -v ^// | awk '{ print $3; }')
if [ "${STEPS_PER_M_X}"0 -eq 0 ]; then
  echo "STEPS_PER_M_X not found, assuming 80'000."
  STEPS_PER_M_X=80000
fi
STEPS_PER_M_Y=$(grep STEPS_PER_M_Y ${CONFIG} | \
                grep -v ^// | awk '{ print $3; }')
if [ "${STEPS_PER_M_Y}"0 -eq 0 ]; then
  echo "STEPS_PER_M_Y not found, assuming 80'000."
  STEPS_PER_M_Y=80000
fi
echo "Taking STEPS_PER_M_X = ${STEPS_PER_M_X} and"
echo "       STEPS_PER_M_Y = ${STEPS_PER_M_Y} for mm/min calculation."


for GCODE_FILE in $*; do

  if [ ! -r "${GCODE_FILE}" ]; then
    echo "${GCODE_FILE} not readable, skipping."
    continue
  fi
  echo                              >> ${STATISTICS_FILE}
  echo "${GCODE_FILE} statistics:"  >> ${STATISTICS_FILE}

  FILE="${GCODE_FILE%.gcode}"
  VCD_FILE="${FILE}.vcd"
  DATA_FILE="${FILE}.data"
  VEL_FILE="${FILE}.processed.vcd"


  # Start the simulator and send the file line by line.
  exec 3<>${PIPE_IN_FILE}
  exec 4<>${PIPE_OUT_FILE}

  "${SIMULAVR}" -c vcd:${TRACEIN_FILE}:"${VCD_FILE}" \
                -f "${TEACUP_ELF}" \
                -m 60000000000 -v <&3 >&4 2>&4 &

  while read -rs -u 4 LINE; do
    echo "${LINE}"
    case "${LINE}" in
      *"Ran too long"*)
        echo ">> SimulAVR ended."
        break
        ;;
      "ok"*)
        read LINE
        echo ">> Sending ${LINE}"
        echo "${LINE}" >&3
        ;;
      "stop")
        echo ">> Got \"stop\", killing SimulAVR."
        killall -INT $(basename "${SIMULAVR}") 2> /dev/null || \
          killall -INT "lt-"$(basename "${SIMULAVR}")
        ;;
    esac
  done < "${GCODE_FILE}"

  exec 3>&-
  exec 4>&-


  # Make plottable files from VCD files.
  # This is very rudimentary and does a lot of assumptions.
  # For examble, pin data is assumed to always be b0/b1, pin naming
  # is assumed to match the order in ${TRACEIN_FILE} and starting at "0".
  awk '
    BEGIN {
      print "0 0 0 0 0";
      xDir = yDir = 0;
      xPos = yPos = 0;
      xVel = yVel = 0;
      yAcc = yAcc = 0;
      lastxTime = lastyTime = 0;
    }
    /^#/ {
      time = substr($0, 2);
      next;
    }
    {
      bit = substr($1, 2);
      if ($2 == "0") { # X Dir
        if (bit == 0) xDir = -1;
        else if (bit == 1) xDir = 1;
        else xDir = 0;
      }
      if ($2 == "2") { # Y Dir
        if (bit == 0) yDir = -1;
        else if (bit == 1) yDir = 1;
        else yDir = 0;
      }
      if ($2 == "1") { # X Step, count only raising flanges
        if (bit == 1) {
          xPos += xDir;
          xVel = 1000000000 / (time - lastxTime);
          print time " " xPos " " yPos " " xVel " " yVel;
          lastxTime = time;
        }
      }
      if ($2 == "3") { # Y Step, count only raising flanges
        if (bit == 1) {
          yPos += yDir;
          yVel = 1000000000 / (time - lastyTime);
          print time " " xPos " " yPos " " xVel " " yVel;
          lastyTime = time;
        }
      }
      # No usage for the LED pin signal ($2 == "4") so far.
    }
  ' < "${VCD_FILE}" > "${DATA_FILE}"


  # Create a plot.
  gnuplot << EOF
    set terminal png size 1024,768
    set output "${FILE}.png"

    set title "${GCODE_FILE}"

    set xlabel "X steps"
    set ylabel "Y steps"
    set y2label "feedrate [steps/s]"

    #set origin 10,10;
    plot "${FILE}.data" using (\$2):(\$3) with dots title "position", \
         "${FILE}.data" using (\$2):(\$4) with dots title "X feedrate", \
         "${FILE}.data" using (\$2):(\$5) with dots title "Y feedrate"
EOF


  # Next task: rewrite the VCD file to add speed values.
  #
  # This is a bit tricky, as VCD files demand timestamps in ascending order.
  # Strategy taken: write out all timestamped data with one line per event,
  # then run it through 'sort -g' and reformat it yet again to be a properly
  # formatted VCD file.
  awk '
    function print_binary(n, e) {  # n = number; e = number of bits
      string = "";
      for (i = 0; i < e; i++) {
        if (n >= (2 ^ (e - 1))) # top bit set
          string = string "1";
        else
          string = string "0";
        n *= 2;
        if (n >= (2 ^ e))
          n -= (2 ^ e);
      }
      return string;
    }
    BEGIN {
      # These lines must match the ones after the sort.
      intLen = 16;
      xStepID = "0"; xPosID = "1"; xUmID = "2"; xVelID = "3"; xMmmID = "4";
      yStepID = "5"; yPosID = "6"; yUmID = "7"; yVelID = "8"; yMmmID = "9";
      ledID = "10"; ledTimeID = "11";

      xDir = yDir = 0;
      xPos = yPos = 0;
      lastxTime = lastyTime = 0;
      ledOnTime = 0;

      ledTimeMin = ledTimeMax = 0;
      ledTimeCount = 0;
      ledTimeSum = 0;
    }
    /^#/ {
      time = substr($0, 2);
      if (time == 0) {
        do {
          getline;
        } while ($0 != "$end");
      }
      next;
    }
    {
      bit = substr($1, 2);
      if ($2 == "0") { # X Dir
        if (bit == 0) xDir = -1;
        else if (bit == 1) xDir = 1;
        else xDir = 0;
      }
      if ($2 == "2") { # Y Dir
        if (bit == 0) yDir = -1;
        else if (bit == 1) yDir = 1;
        else yDir = 0;
      }
      if ($2 == "1") { # X Step
        if (bit == 1) { # raising flange
          xPos += xDir;
          print time " b" print_binary(xPos, intLen) " " xPosID;
          # TODO: it might be better to output mm as real value, but ...
          #       ... does the VCD file format support this? If yes, how?
          print time " b" print_binary(xPos * 1000000 / '"${STEPS_PER_M_X}"', intLen) " " xUmID;
          vel = 1000000000 / (time - lastxTime);
          print lastxTime " b" print_binary(vel, intLen) " " xVelID;
          vel = vel * 60000 / '"${STEPS_PER_M_X}"';
          print lastxTime " b" print_binary(vel, intLen) " " xMmmID;
          print time " b" bit " " xStepID;
          lastxTime = time;
        } else { # falling flange
          print time " b" bit " " xStepID;
        }
      }
      if ($2 == "3") { # Y Step
        if (bit == 1) { # raising flange
          yPos += yDir;
          print time " b" print_binary(yPos, intLen) " " yPosID;
          print time " b" print_binary(yPos * 1000000 / '"${STEPS_PER_M_Y}"', intLen) " " yUmID;
          vel = 1000000000 / (time - lastyTime);
          print lastyTime " b" print_binary(vel, intLen) " " yVelID;
          vel = vel * 60000 / '"${STEPS_PER_M_Y}"';
          print lastyTime " b" print_binary(vel, intLen) " " yMmmID;
          print time " b" bit " " yStepID;
          lastyTime = time;
        } else { # falling flange
          print time " b" bit " " yStepID;
        }
      }
      if ($2 == "4") { # LED signal
        if (bit == 1) { # raising flange
          print time " b" bit " " ledID;
          ledOnTime = time;
        } else { # falling flange
          print time " b" bit " " ledID;
          if (ledOnTime != 0) {
            # Convert from nanoseconds to clock cycles.
            ledTime = (time - ledOnTime) / 50;
            print ledOnTime " b" print_binary(ledTime, 32) " " ledTimeID;
            if (ledTimeMin == 0 || ledTime < ledTimeMin) {
              ledTimeMin = ledTime;
            }
            if (ledTime > ledTimeMax) {
              ledTimeMax = ledTime;
            }
            ledTimeCount++;
            ledTimeSum += ledTime;
          }
        }
      }
    }
    END {
      if (ledTimeCount > 0) {
        print "LED on occurences: " ledTimeCount "." >> "'${STATISTICS_FILE}'";
        print "LED on time minimum: " ledTimeMin " clock cycles." \
            >> "'${STATISTICS_FILE}'";
        print "LED on time maximum: " ledTimeMax " clock cycles." \
            >> "'${STATISTICS_FILE}'";
        print "LED on time average: " ledTimeSum / ledTimeCount " clock cycles." \
            >> "'${STATISTICS_FILE}'";
      } else {
        print "Debug LED apparently unused." > "'${STATISTICS_FILE}'";
      }
    }
  ' < "${VCD_FILE}" | \
  sort -g | \
  awk '
    BEGIN {
      # These lines must match the ones before the sort.
      intLen = 16;
      xStepID = "0"; xPosID = "1"; xUmID = "2"; xVelID = "3"; xMmmID = "4";
      yStepID = "5"; yPosID = "6"; yUmID = "7"; yVelID = "8"; yMmmID = "9";
      ledID = "10"; ledTimeID = "11";

      lastTime = "";

      print "$timescale 1ns $end";
      print "$scope module Steppers $end";
      print "$var wire 1 " xStepID " X_step $end";
      print "$var integer " intLen " " xPosID " X_pos_steps $end";
      print "$var integer " intLen " " xUmID " X_pos_um $end";
      print "$var integer " intLen " " xVelID " X_steps/s $end";
      print "$var integer " intLen " " xMmmID " X_mm/min $end";
      print "$var wire 1 " yStepID " Y_step $end";
      print "$var integer " intLen " " yPosID " Y_pos_steps $end";
      print "$var integer " intLen " " yUmID " Y_pos_um $end";
      print "$var integer " intLen " " yVelID " Y_steps/s $end";
      print "$var integer " intLen " " yMmmID " Y_mm/min $end";
      print "$upscope $end";
      print "$scope module Timings $end";
      print "$var wire 1 " ledID " Debug_LED $end";
      print "$var integer " 32 " " ledTimeID " LED_on_clocks $end";
      print "$upscope $end";
      print "$enddefinitions $end";
      print "#0";
      print "$dumpvars";
      print "bz " xStepID;
      print "bz " xPosID;
      print "bz " xUmID;
      print "bz " xVelID;
      print "bz " xMmmID;
      print "bz " yStepID;
      print "bz " yPosID;
      print "bz " yUmID;
      print "bz " yVelID;
      print "bz " yMmmID;
      print "bz " ledID;
      print "bz " ledTimeID;
      print "$end";
    }
    {
      if ($1 != lastTime) {
        print "#" $1;
        lastTime = $1;
      }
      print $2 " " $3;
    }
  ' > "${VEL_FILE}"


done

