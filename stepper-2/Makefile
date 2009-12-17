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

PROGRAM = stepper

SOURCES = $(PROGRAM).c ringbuffer.c serial.c lcd.c

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
OPTIMIZE = -Os
CFLAGS = -g -Wall -Wstrict-prototypes $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(F_CPU) $(DEFS) -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -ffunction-sections -save-temps
LDFLAGS = -Wl,-u,vfprintf -lprintf_min -Wl,--as-needed -Wl,--gc-sections -finline-functions-called-once

CC = $(ARCH)gcc
OBJDUMP = $(ARCH)objdump
OBJCOPY = $(ARCH)objcopy
AVRDUDE = avrdude -F

PROGPORT = /dev/arduino
PROGBAUD = 19200

OBJ = $(patsubst %.c,%.o,${SOURCES})

.PHONY: all program clean
.PRECIOUS: %.o %.elf

all: $(PROGRAM).hex $(PROGRAM).lst

program: $(PROGRAM).hex
	stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	@stty $(PROGBAUD) raw ignbrk hup < $(PROGPORT)
	$(AVRDUDE) -cstk500v1 -b$(PROGBAUD) -p$(MCU_TARGET) -P$(PROGPORT) -C/etc/avrdude.conf -U flash:w:$^
	stty -hup -echo < $(PROGPORT)

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex *.al *.i *.s *~

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
