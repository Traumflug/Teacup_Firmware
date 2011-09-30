#!/bin/bash

# This is a script to send G-code from Mac OS X. The problem with Mac OS X is,
# stty settings on serial ports don't "stick", so it's impossible to set
# e.g. the baud rate before connecting to that port. The solution is to use
# a tool which can do both, setting the baud rate and sending characters.
#
# The tool of choice is the somewhat confusing "screen", part of the
# standard Mac OS X distribution.
#
# --- Usage ---
#
# A nice feature of screen is, you can "detach" a connection which means,
# you close the interactive part of screen, but the connection on the serial
# is kept open. This allows so send multiple files to the controller without
# triggering an auto-reset in between. You can do a full close as well, of
# course.
#
# Start sending commands manually or re-attach to an earlier session:
#
#     ./sender-mac.sh
#
# Send a file:
#
#     ./sender-mac.sh <G-code file>
#
# Send multiple files:
#
#     ./sender-mac.sh <G-code file 1>
#     ctrl-a d
#     ./sender-mac.sh <G-code file 2>
#
# The tricky part with screen is to leave such an interactive session. Screen
# doesn't react to the usual ctrl-c, but asks for ctrl-a. Following that ctrl-a
# you can type single-character commands, like d (detach) or k (kill). There
# is no visual feedback on hitting ctrl-a, type the command character blindly.
#
# Leave the session, keep the connection alive:
#
#     ctrl-a d
#
# Leave the session, drop the connection:
#
#     ctrl-a k y
#
# Stop sending the current file immediately:
#
#     ctrl-a k y
#
# After the file is sent, you're always dropped into interactive mode, so you
# can send additional commands and/or leave the session.


DEV=$(echo /dev/tty.usbserial*)
BAUD=115200


function strip_text {
  STRIP_TEXT=$(echo $1 | tr -d '\r\n')
  STRIP_TEXT="${STRIP_TEXT## }"
  STRIP_TEXT="${STRIP_TEXT%% }"
  STRIP_TEXT="${STRIP_TEXT##\t}"
  STRIP_TEXT="${STRIP_TEXT%%\t}"
}


if [ "${STY}" = "" ]; then
  # we're not inside a screen session, so
  # create one and restart our selfs

  # make sure we have a session
  screen -r teacup -X detach >/dev/null
  if [ $? -ne 0 ]; then
    # creating in detached mode prevents -X exec later, unfortunately
    (sleep 3 && screen -S teacup -d) &  ## hack alarm!
    screen -S teacup $DEV $BAUD
  fi

  screen -S teacup -X clear
  screen -S teacup -X exec '!!.' "$PWD"/$(basename $0) "$1"
  screen -r teacup
else
  # we're called from inside screen,
  # with the RepRap controller on stdin/stdout

  # get an initial prompt
  OK=""
  echo
  while true; do
    read -t 10 OK MESSAGE
    if [ "$OK" = "ok" ]; then break; fi
    echo  # trigger a new "ok"
  done

  if [ -r "$1" ]; then
    # send line by line and wait for "ok" each time
    exec 3<> "$1"
    while read <&3 LINE; do
      strip_text "$LINE" && LINE="$STRIP_TEXT"
      echo -n "$LINE" >&2
      echo $LINE
      OK=""
      while true; do
        read OK MESSAGE
        if [ $? -ne 0 ]; then
          # probably a disconnection from screen
          exit
        fi
        strip_text "$OK" && OK="$STRIP_TEXT"
        strip_text "$MESSAGE" && MESSAGE="$STRIP_TEXT"
        echo "<$OK>" >&2
        if [ "$OK" = "ok" ]; then
          break
        fi
      done
      if [ "$MESSAGE" != "" ]; then
        echo "$MESSAGE" >&2
        sleep 1
      fi
    done
    echo "File done, dropping to manual operation." >&2
  else
    echo "No readable file, proceed manually." >&2
  fi
fi

