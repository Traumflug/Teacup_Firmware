##############################################################################
#                                                                            #
# Teacup - alternative firmware for repraps                                  #
#                                                                            #
# by Triffid Hunter, Traumflug, jakepoz, Markus Hitter, many others          #
#                                                                            #
#                                                                            #
# This firmware is Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   #
#                                                                            #
# This program is free software; you can redistribute it and/or modify       #
# it under the terms of the GNU General Public License as published by       #
# the Free Software Foundation; either version 2 of the License, or          #
# (at your option) any later version.                                        #
#                                                                            #
# This program is distributed in the hope that it will be useful,            #
# but WITHOUT ANY WARRANTY; without even the implied warranty of             #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              #
# GNU General Public License for more details.                               #
#                                                                            #
# You should have received a copy of the GNU General Public License          #
# along with this program; if not, write to the Free Software                #
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA #
#                                                                            #
##############################################################################

##############################################################################
#                                                                            #
# Change these to suit your hardware                                         #
#                                                                            #
##############################################################################

# MCU_TARGET = atmega168
# MCU_TARGET = atmega328p
MCU_TARGET = atmega644p
# MCU_TARGET = atmega1280
# MCU_TARGET = atmega2560
# MCU_TARGET = at90usb1287

# CPU clock rate
F_CPU = 16000000L
# F_CPU = 8000000L
DEFS = -DF_CPU=$(F_CPU)

##############################################################################
#                                                                            #
# Programmer settings for "make program"                                     #
#                                                                            #
##############################################################################

AVRDUDE = avrdude
AVRDUDECONF = /etc/avrdude.conf

##############################################################################
#                                                                            #
# udev rule for /dev/arduino (insert into /etc/udev/rules.d/99-local.rules)  #
# SUBSYSTEMS=="usb", ATTRS{idProduct}=="6001", ATTRS{idVendor}=="0403",      #
#     NAME="%k", SYMLINK+="arduino", SYMLINK+="arduino_$attr{serial}",       #
#     MODE="0660"                                                            #
#                                                                            #
##############################################################################

PROGPORT = /dev/arduino
# PROGPORT = /dev/ttyUSB0

##############################################################################
#                                                                            #
# This depends on the bootloader (or programmer) in use.                     #
# Examples:                                                                  #
#                                                                            #
# Arduino Diecimilia with genuine bootloader:        19200                   #
# Sanguino bootloader:                               57600                   #
# Gen7 bootloader:                                  115200                   #
#                                                                            #
# Set PROGBAUD to 0 (Zero) for programmers.                                  #
#                                                                            #
##############################################################################

PROGBAUD = 115200

##############################################################################
#                                                                            #
# Firmware upload device type. Typically stk500 or stk500v2.                 #
#                                                                            #
##############################################################################

PROGID = stk500v2

##############################################################################
#                                                                            #
# These defaults should be ok, change if you need to                         #
#                                                                            #
##############################################################################

PROGRAM = mendel

SOURCES = $(PROGRAM).c gcode_parse.c gcode_process.c dda.c dda_maths.c dda_queue.c timer.c temp.c sermsg.c watchdog.c debug.c sersendf.c heater.c analog.c intercom.c pinio.c clock.c home.c crc.c delay.c

ARCH = avr-
CC = $(ARCH)gcc
OBJDUMP = $(ARCH)objdump
OBJCOPY = $(ARCH)objcopy

OPTIMIZE = -Os -ffunction-sections -finline-functions-called-once -mcall-prologues
# OPTIMIZE = -O0
CFLAGS = -g -Wall -Wstrict-prototypes $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -save-temps -Winline
LDFLAGS = -Wl,--as-needed -Wl,--gc-sections
LIBS = -lm
LIBDEPS =
SUBDIRS =

ifneq (,$(findstring usb,$(MCU_TARGET)))
LDFLAGS += -Llufa_serial
LIBS += -llufa_serial
SUBDIRS += lufa_serial
LIBDEPS += lufa_serial/liblufa_serial.a
else
SOURCES += serial.c
endif

ifeq ($(PROGBAUD),0)
PROGBAUD_FLAG =
else
PROGBAUD_FLAG = -b$(PROGBAUD)
endif

OBJ = $(patsubst %.c,%.o,${SOURCES})

.PHONY: all program clean size subdirs doc functionsbysize
.PRECIOUS: %.o %.elf

all: config.h subdirs $(PROGRAM).hex $(PROGRAM).lst $(PROGRAM).sym size

$(PROGRAM).elf: $(LIBDEPS)

subdirs:
	@for dir in $(SUBDIRS); do \
	  $(MAKE) -C $$dir; \
	done

program: $(PROGRAM).hex config.h
	stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	@sleep 0.1
	@stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	$(AVRDUDE) -c$(PROGID) $(PROGBAUD_FLAG) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF) -U flash:w:$^
	stty 115200 raw ignbrk -hup -echo ixoff < $(PROGPORT)

clean: clean-subdirs
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex *.al *.i *.s *~

clean-subdirs:
	@for dir in $(SUBDIRS); do \
	  $(MAKE) -C $$dir clean; \
	done

size: $(PROGRAM).elf
	@echo "    SIZES             ATmega...  '168    '328(P)    '644(P)    '1280"
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(text)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    FLASH : %5d bytes          %3d%%      %3d%%       %3d%%      %3d%%\n", $$a, ceil($$a * 100 / (14 * 1024)), ceil($$a * 100 / (30 * 1024)),ceil($$a * 100 / (62 * 1024)), ceil($$a * 100 / (126 * 1024)) }' 
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(data|bss)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    RAM   : %5d bytes          %3d%%      %3d%%       %3d%%      %3d%%\n", $$a, ceil($$a * 100 / (1 * 1024)), ceil($$a * 100 / (2 * 1024)),ceil($$a * 100 / (4 * 1024)), ceil($$a * 100 / (8 * 1024)) }'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(eeprom)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    EEPROM: %5d bytes          %3d%%      %3d%%       %3d%%      %3d%%\n", $$a, ceil($$a * 100 / (1 * 1024)), ceil($$a * 100 / (2 * 1024)), ceil($$a * 100 / (2 * 1024)), ceil($$a * 100 / (4 * 1024)) }'

config.h: config.default.h
	@echo "config.default.h is more recent than config.h. You likely want to"
	@echo "review (edit) config.h to match new features in config.default.h."
	@echo "To view the differences, run: diff -bBEu config.h config.default.h"
	@echo "If you just want to get rid of this message, run: touch config.h"
	@false

doc: Doxyfile *.c *.h
	doxygen $<

functionsbysize: $(OBJ)
	@avr-objdump -h $^ | grep '\.text\.' | perl -ne '/\.text\.(\S+)\s+([0-9a-f]+)/ && printf "%u\t%s\n", eval("0x$$2"), $$1;' | sort -n

%.o: %.c config.h Makefile
	@echo "  CC        $@"
	@$(CC) -c $(CFLAGS) -Wa,-adhlns=$(<:.c=.al) -o $@ $(subst .o,.c,$@)

%.elf: $(OBJ)
	@echo "  LINK      $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.lst: %.elf
	@echo "  OBJDUMP   $@"
	@$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	@echo "  OBJCOPY   $@"
	@$(OBJCOPY) -j .text -j .data -O ihex -R .eeprom -R .fuse -R .lock $< $@

%.bin: %.elf
	@echo "  OBJCOPY   $@"
	@$(OBJCOPY) -j .text -j .data -O binary $< $@

%.sym: %.elf
	@echo "  SYM       $@"
	@$(OBJDUMP) -t $< | perl -ne 'BEGIN { printf "  ADDR  NAME                  SIZE\n"; } /([0-9a-f]+)\s+(\w+)\s+O\s+\.(bss|data)\s+([0-9a-f]+)\s+(\w+)/ && printf "0x%04x  %-20s +%d\n", eval("0x$$1") & 0xFFFF, $$5, eval("0x$$4")' | sort -k1 > $@
