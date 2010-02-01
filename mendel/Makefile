##############################################################################
#                                                                            #
# AVR-GCC skeleton                                                           #
#                                                                            #
# by Triffid Hunter                                                          #
#                                                                            #
##############################################################################

##############################################################################
#                                                                            #
# Change these to suit your application                                      #
#                                                                            #
##############################################################################

PROGRAM = mendel

SOURCES = $(PROGRAM).c ringbuffer.c serial.c dda.c gcode.c timer.c clock.c temp.c sermsg.c

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
OPTIMIZE = -Os -ffunction-sections  -finline-functions-called-once -DDEBUG=1
# OPTIMIZE = -O0
CFLAGS = -g -Wall -Wstrict-prototypes $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(F_CPU) $(DEFS) -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -save-temps
LDFLAGS = -Wl,--as-needed -Wl,--gc-sections

CC = $(ARCH)gcc
OBJDUMP = $(ARCH)objdump
OBJCOPY = $(ARCH)objcopy
AVRDUDE = avrdude

PROGPORT = /dev/arduino
PROGBAUD = 19200

OBJ = $(patsubst %.c,%.o,${SOURCES})

.PHONY: all program clean size
.PRECIOUS: %.o %.elf

all: $(PROGRAM).hex $(PROGRAM).lst size

program: $(PROGRAM).hex
	stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	@sleep 0.2
	@stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	$(AVRDUDE) -cstk500v1 -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C/etc/avrdude.conf -U flash:w:$^
	stty 57600 raw ignbrk -hup -echo ixon < $(PROGPORT)

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex *.al *.i *.s *~

size: $(PROGRAM).hex
	@objdump -h mendel.elf | perl -ne '/.(data|text)\s+([0-9a-f]+)/ && do { $$a += eval "0x$$2" }; END { printf "%d bytes (%d%% of %dkb)\n", $$a, $$a * 100 / 16384, 16 }'

%.o: %.c
	$(CC) -c $(CFLAGS) -Wa,-adhlns=$(<:.c=.al) -o $@ $^

%.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
