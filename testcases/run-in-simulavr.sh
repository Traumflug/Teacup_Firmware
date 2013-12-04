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


# Prepare a pin tracing file, assuming a Gen7-v1.4 configuration. See
# http://reprap.org/wiki/SimulAVR#Putting_things_together:_an_example
echo "# X Dir"              > /tmp/tracein.txt
echo "+ PORTA.A3-Out"      >> /tmp/tracein.txt
echo "# X Step"            >> /tmp/tracein.txt
echo "+ PORTA.A2-Out"      >> /tmp/tracein.txt
echo "# Y Dir"             >> /tmp/tracein.txt
echo "+ PORTA.A5-Out"      >> /tmp/tracein.txt
echo "# Y Step"            >> /tmp/tracein.txt
echo "+ PORTA.A4-Out"      >> /tmp/tracein.txt
echo "Assuming pin configuration for a Gen7-v1.4."

STEPS_PER_M_X=$(grep STEPS_PER_M_X ../config.h | \
                grep -v ^// | awk '{ print $3; }')
if [ "${STEPS_PER_M_X}"0 -eq 0 ]; then
  echo "STEPS_PER_M_X not found, assuming 80'000."
  STEPS_PER_M_X=80000
fi
STEPS_PER_M_Y=$(grep STEPS_PER_M_Y ../config.h | \
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

  FILE="${GCODE_FILE%.gcode}"
  VCD_FILE="${FILE}.vcd"
  DATA_FILE="${FILE}.data"
  VEL_FILE="${FILE}.processed.vcd"
  

  # We assume here queue and rx buffer are large enough to read
  # the file in one chunk. If not, raise MOVEBUFFER_SIZE in config.h.
  "${SIMULAVR}" -c vcd:/tmp/tracein.txt:"${VCD_FILE}" \
                -f ../build/teacup.elf \
                -m 60000000000  < "${GCODE_FILE}"


  # Make plottable files from VCD files.
  # This is very rudimentary and does a lot of assumptions.
  # For examble, pin data is assumed to always be b0/b1, pin naming
  # is assumed to match the order in tracein.txt and starting at "0".
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
      xStepID = "0"; xPosID = "1"; xUmID = "2"; xVelID = "3"; xMmmID = "4"
      yStepID = "5"; yPosID = "6"; yUmID = "7"; yVelID = "8"; yMmmID = "9"

      xDir = yDir = 0;
      xPos = yPos = 0;
      lastxTime = lastyTime = 0;
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
    }
  ' < "${VCD_FILE}" | \
  sort -g | \
  awk '
    BEGIN {
      # These lines must match the ones before the sort.
      intLen = 16;
      xStepID = "0"; xPosID = "1"; xUmID = "2"; xVelID = "3"; xMmmID = "4"
      yStepID = "5"; yPosID = "6"; yUmID = "7"; yVelID = "8"; yMmmID = "9"

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
      print "$enddefinitions $end";
      print "#0";
      print "$dumpvars";
      print "b0 " xStepID;
      print "b0 " xPosID;
      print "b0 " xUmID;
      print "b0 " xVelID;
      print "b0 " xMmmID;
      print "b0 " yStepID;
      print "b0 " yPosID;
      print "b0 " yUmID;
      print "b0 " yVelID;
      print "b0 " yMmmID;
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

