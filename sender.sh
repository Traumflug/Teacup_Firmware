#!/bin/bash

DEV=/dev/arduino
BAUD=115200

waitfor avrdude
stty $BAUD raw ignbrk -hup -echo ixon < $DEV

(
	read -t 0.1; RV=$?
	while [ $RV -eq 0 ] || [ $RV -ge 128 ]
	do
		if [ $RV -eq 0 ]
		then
			echo "> $REPLY"
			echo "$REPLY" >&3
		fi
		while [ "$REPLY" != "OK" ] && [ "$REPLY" != "ok" ]
		do
			read -s -u 3
			echo "< $REPLY"
                        case "$REPLY" in
                                *ok*) REPLY=OK ;;
                                *OK*) REPLY=OK ;;
                        esac
		done
		read -t 1; RV=$?
	done
) 3<>$DEV
