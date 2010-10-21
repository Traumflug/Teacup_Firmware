##############################################################################
#                                                                            #
# FiveD on Arduino - alternative firmware for repraps                        #
#                                                                            #
# by Triffid Hunter, Traumflug, jakepoz                                      #
#                                                                            #
##############################################################################

##############################################################################
#                                                                            #
# Change these to suit your application                                      #
#                                                                            #
##############################################################################

PROGRAM = mendel

SOURCES = $(PROGRAM).c serial.c dda.c gcode_parse.c gcode_process.c clock.c timer.c temp.c sermsg.c dda_queue.c watchdog.c debug.c sersendf.c heater.c analog.c delay.c

##############################################################################
#                                                                            #
# Change these to suit your hardware                                         #
#                                                                            #
##############################################################################

MCU_TARGET = atmega328p
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
#                                                                            #
##############################################################################

DEFS = -DF_CPU=$(F_CPU)
# DEFS += "-DDEBUG=1"

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
	rm -f sim

size: $(PROGRAM).elf
	@echo "  SIZE                   Atmega168        Atmega328p       Atmega644"
	@$(OBJDUMP) -h $^ | perl -ne '/.(text)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    FLASH : %5d bytes  (%2d%% of %2dkb)    (%2d%% of %2dkb)    (%2d%% of %2dkb)\n", $$a, $$a * 100 / (14 * 1024), 14, $$a * 100 / (30 * 1024), 30, $$a * 100 / (63 * 1024), 63 }'
	@$(OBJDUMP) -h $^ | perl -ne '/.(data|bss)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    RAM   : %5d bytes  (%2d%% of %2dkb)    (%2d%% of %2dkb)    (%2d%% of %2dkb)\n", $$a, $$a * 100 / (1 * 1024), 1, $$a * 100 / (2 * 1024), 2, $$a * 100 / (4 * 1024), 4 }'
	@$(OBJDUMP) -h $^ | perl -ne '/.(eeprom)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "    EEPROM: %5d bytes  (%2d%% of %2dkb)    (%2d%% of %2dkb)    (%2d%% of %2dkb)\n", $$a, $$a * 100 / (1 * 1024), 1, $$a * 100 / (2 * 1024), 2, $$a * 100 / (2 * 1024), 2 }'

config.h: config.h.dist
	@echo "Please review config.h, as config.h.dist is more recent."
	@false

%.o: %.c config.h
	@echo "  CC        $@"
	@$(CC) -c $(CFLAGS) -Wa,-adhlns=$(<:.c=.al) -o $@ $<

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


##############################################################################
#                                                                            #
# Simulation                                                                 #
#                                                                            #
##############################################################################

SIM_SOURCES = $(PROGRAM).c serial_sim.c dda.c gcode_parse.c gcode_process.c timer_sim.c clock_sim.c temp.c sermsg.c dda_queue.c debug.c sersendf.c heater.c analog_sim.c delay_sim.c simulation.c
SIM_HEADERS = config.h serial.h dda.h gcode_parse.h gcode_process.h timer.h clock.h temp.h sermsg.h dda_queue.h debug.h sersendf.h heater.h analog.h delay.h simulation.h

SIM_OBJ = $(patsubst %.c,%.sim.o,${SIM_SOURCES})
SIM_CFLAGS = -g -Wall -Wstrict-prototypes -Os $(DEFS) -std=gnu99 -funsigned-char -funsigned-bitfields -fshort-enums

%.sim.o: %.c $(SIM_HEADERS)
	@echo "  CC        $@"
	@cc -DDEBUG -DSIMULATION -Wa,-adhlns=$(<:.c=.al) -c $(SIM_CFLAGS) -o $@ $<

sim:	$(SIM_OBJ)
	@echo "  LINK      $@"
	@cc $(SIM_CFLAGS) -o $@ $^
