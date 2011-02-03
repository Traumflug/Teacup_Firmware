##############################################################################
#                                                                            #
# FiveD on Arduino - alternative firmware for repraps                        #
#                                                                            #
# by Triffid Hunter, Traumflug, jakepoz                                      #
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

# F_CPU = 16000000L

##############################################################################
#                                                                            #
# Available Defines:                                                         #
#                                                                            #
# DEBUG                                                                      #
#   enables tons of debugging output. may cause host-side talkers to choke   #
# XONXOFF                                                                    #
#   enables XON/XOFF flow control for stupid or crude talkers                #
# ACCELERATION_REPRAP                                                        #
#   enables reprap-style acceleration                                        #
# ACCELERATION_RAMPING                                                       #
#   enables start/stop ramping                                               #
# ACCELERATION_TEMPORAL                                                      #
#   enables experimental temporal step algorithm - not technically a type of #
#   acceleration, but since it controls step timing it seems appropriate     #
# GEN3                                                                       #
#   build for standard reprap electronics instead of your custom rig         #
# HOST                                                                       #
#   this is the motherboard for GEN3- don't touch! Extruder has its own      #
#   Makefile.                                                                #
#                                                                            #
##############################################################################

# DEFS = -DF_CPU=$(F_CPU) -DHOST -DGEN3
# DEFS += "-DDEBUG=1"

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

# atmega168
#PROGBAUD = 19200
# atmega328p, 644p, 1280
PROGBAUD = 57600


##############################################################################
#                                                                            #
# These defaults should be ok, change if you need to                         #
#                                                                            #
##############################################################################

PROGRAM = mendel

SOURCES = $(PROGRAM).c serial.c dda.c gcode_parse.c gcode_process.c timer.c temp.c sermsg.c dda_queue.c watchdog.c debug.c sersendf.c heater.c analog.c delay.c intercom.c pinio.c clock.c

ARCH = avr-
CC = $(ARCH)gcc
OBJDUMP = $(ARCH)objdump
OBJCOPY = $(ARCH)objcopy

OPTIMIZE = -Os -ffunction-sections -finline-functions-called-once -mcall-prologues
# OPTIMIZE = -O0
CFLAGS = -g -Wall -Wstrict-prototypes $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -save-temps
LDFLAGS = -Wl,--as-needed -Wl,--gc-sections

OBJ = $(patsubst %.c,%.o,${SOURCES})

.PHONY: all program clean size
.PRECIOUS: %.o %.elf

all: config.h $(PROGRAM).hex $(PROGRAM).lst $(PROGRAM).sym size

program: $(PROGRAM).hex config.h
	stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	@sleep 0.1
	@stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	$(AVRDUDE) -cstk500v1 -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF) -U flash:w:$^
	stty 115200 raw ignbrk -hup -echo ixoff < $(PROGPORT)

program-fuses:
	avr-objdump -s -j .fuse mendel.o | perl -ne '/\s0000\s([0-9a-f]{2})/ && print "$$1\n"' > lfuse
	avr-objdump -s -j .fuse mendel.o | perl -ne '/\s0000\s..([0-9a-f]{2})/ && print "$$1\n"' > hfuse
	avr-objdump -s -j .fuse mendel.o | perl -ne '/\s0000\s....([0-9a-f]{2})/ && print "$$1\n"' > efuse
	$(AVRDUDE) -cstk500v1 -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF) -U lfuse:w:lfuse
	$(AVRDUDE) -cstk500v1 -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF) -U hfuse:w:hfuse
	$(AVRDUDE) -cstk500v1 -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C$(AVRDUDECONF) -U efuse:w:efuse

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex *.al *.i *.s *~ *fuse

size: $(PROGRAM).elf
	@echo "  SIZE                   Atmega168        Atmega328p       Atmega644"
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(text)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    FLASH : %5d bytes  (%2d%% of %2dkb)    (%2d%% of %2dkb)    (%2d%% of %2dkb)\n", $$a, ceil($$a * 100 / (15 * 1024)), 15, ceil($$a * 100 / (31 * 1024)), 31, ceil($$a * 100 / (63 * 1024)), 63 }'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(data|bss)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    RAM   : %5d bytes  (%2d%% of %2dkb)    (%2d%% of %2dkb)    (%2d%% of %2dkb)\n", $$a, ceil($$a * 100 / (1 * 1024)), 1, ceil($$a * 100 / (2 * 1024)), 2, ceil($$a * 100 / (4 * 1024)), 4 }'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(eeprom)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    EEPROM: %5d bytes  (%2d%% of %2dkb)    (%2d%% of %2dkb)    (%2d%% of %2dkb)\n", $$a, ceil($$a * 100 / (1 * 1024)), 1, ceil($$a * 100 / (2 * 1024)), 2, ceil($$a * 100 / (2 * 1024)), 2 }'

config.h: config.h.dist
	@echo "Please review config.h, as config.h.dist is more recent."
	@echo
	@diff -bBEuF '^. [[:digit:]]. [[:upper:]]' config.h config.h.dist
	@false

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
