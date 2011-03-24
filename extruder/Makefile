##############################################################################
#                                                                            #
# Teacup - alternative firmware for repraps                                  #
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
# Change these to suit your application                                      #
#                                                                            #
##############################################################################

PROGRAM = extruder

SOURCES = $(PROGRAM).c intercom.c delay.c analog.c watchdog.c heater.c temp.c timer.c crc.c

##############################################################################
#                                                                            #
# Change these to suit your hardware                                         #
#                                                                            #
##############################################################################

MCU_TARGET = atmega168
F_CPU = 16000000L

##############################################################################
#                                                                            #
# These defaults should be ok, change if you need to                         #
#                                                                            #
##############################################################################

ARCH = avr-
CC = $(ARCH)gcc
OBJDUMP = $(ARCH)objdump
OBJCOPY = $(ARCH)objcopy

DEFS = -DF_CPU=$(F_CPU)

OPTIMIZE = -Os -ffunction-sections -finline-functions-called-once
# OPTIMIZE = -O0
CFLAGS = -g -Wall -Wstrict-prototypes $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -save-temps
LDFLAGS = -Wl,--as-needed -Wl,--gc-sections

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

PROGBAUD = 57600

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

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex *.al *.i *.s *~

size: $(PROGRAM).elf
	@echo "  SIZE                   Atmega168        Atmega328p       Atmega644"
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(text)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    FLASH : %5d bytes  (%2d%% of %2dkb)    (%2d%% of %2dkb)    (%2d%% of %2dkb)\n", $$a, ceil($$a * 100 / (14 * 1024)), 14, ceil($$a * 100 / (30 * 1024)), 30, ceil($$a * 100 / (63 * 1024)), 63 }'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(data|bss)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    RAM   : %5d bytes  (%2d%% of %2dkb)    (%2d%% of %2dkb)    (%2d%% of %2dkb)\n", $$a, ceil($$a * 100 / (1 * 1024)), 1, ceil($$a * 100 / (2 * 1024)), 2, ceil($$a * 100 / (4 * 1024)), 4 }'
	@$(OBJDUMP) -h $^ | perl -MPOSIX -ne '/.(eeprom)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    EEPROM: %5d bytes  (%2d%% of %2dkb)    (%2d%% of %2dkb)    (%2d%% of %2dkb)\n", $$a, ceil($$a * 100 / (1 * 1024)), 1, ceil($$a * 100 / (2 * 1024)), 2, ceil($$a * 100 / (2 * 1024)), 2 }'

config.h: config.h.dist
	@echo "Please review config.h, as config.h.dist is more recent."
	@false

%.o: %.c config.h
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
	@$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.bin: %.elf
	@echo "  OBJCOPY   $@"
	@$(OBJCOPY) -j .text -j .data -O binary $< $@

%.sym: %.elf
	@echo "  SYM       $@"
	@$(OBJDUMP) -t $< | perl -ne 'BEGIN { printf "  ADDR  NAME                  SIZE\n"; } /([0-9a-f]+)\s+(\w+)\s+O\s+\.(bss|data)\s+([0-9a-f]+)\s+(\w+)/ && printf "0x%04x  %-20s +%d\n", eval("0x$$1") & 0xFFFF, $$5, eval("0x$$4")' | sort -k1 > $@
