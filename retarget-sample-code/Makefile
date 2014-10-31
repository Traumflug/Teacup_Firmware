
NAME = retarget

# Adjust this to your mbed installation location.
MBED = ./mbed/libraries/mbed

# Compiler & Linker
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

# Other target MCU specific adjustments:
#
# Search for "TARGET" (uppercase, without quotes) in this file and ajust
# all lines you find to your target MCU. To find out what exactly you need
# you currently have to search through mbed's file tree, watching out for
# files named more appropriately for your target MCU. Examples here are
# for an NXP LPC1114.

CFLAGS  = -mthumb -mcpu=cortex-m0
# Find a list at the bottom of $(MBED)/workspace_tools/targets.py
CFLAGS += -DTARGET_LPC1114
# Find a list in $(MBED)/workspace_tools/toolchains/
CFLAGS += -DTOOLCHAIN_GCC_ARM
# Turn off assertions.
CFLAGS += -DNDEBUG
# Startup definitions. Not target MCU specific.
CFLAGS += -D__STARTUP_CLEAR_BSS -D__START=main
# Always.
CFLAGS += -I$(MBED)
CFLAGS += -I$(MBED)/api
CFLAGS += -I$(MBED)/common
CFLAGS += -I$(MBED)/hal
CFLAGS += -I$(MBED)/targets/cmsis/
# Next four depend on target MCU.
CFLAGS += -I$(MBED)/targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX
CFLAGS += -I$(MBED)/targets/hal/TARGET_NXP/TARGET_LPC11XX_11CXX/TARGET_LPC11XX
CFLAGS += -I$(MBED)/targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX
CFLAGS += -I$(MBED)/targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/TARGET_LPC11XX
# Other options ...
CFLAGS += -Wall
CFLAGS += -Os
CFLAGS += -flto
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
# Need following option for LTO as LTO will treat retarget functions as
# unused without following option
CFLAGS += -fno-builtin

LDFLAGS  = --specs=nano.specs
LDFLAGS += --specs=nosys.specs
# Yet another target MCU specific line.
LDFLAGS += -T $(MBED)/targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/TOOLCHAIN_GCC_ARM/TARGET_LPC11XX/LPC1114.ld
LDFLAGS += -Wl,--gc-sections
#LDFLAGS += -Wl,-Map=$(NAME).map

OBJECTS  = $(NAME).o
OBJECTS += main.o
# These are part of mbed, target MCU dependent and get created on the fly.
# mbed_serial_api.c is created from serial_api.c for our target MCU,
# mbed_pinmap.c is created from pinmap.c for our target MCU, and so on.
OBJECTS += mbed_serial_api.o
OBJECTS += mbed_pinmap_common.o
OBJECTS += mbed_pinmap.o
OBJECTS += mbed_error.o
OBJECTS += mbed_system_LPC11xx.o


.PHONY: all program size clean

all: $(NAME).hex size

program: $(NAME).hex size
	lpc21isp -control $< /dev/ttyUSB0 115200 12000

$(NAME).hex: $(NAME).elf
	$(OBJCOPY) -O ihex $< $@

$(NAME).elf: $(OBJECTS) startup_LPC11xx.o
	$(CC) $^ $(CFLAGS) $(LDFLAGS) -o $@

$(OBJECTS): %.o: %.c Makefile
	$(CC) -c $(CFLAGS) $< -o $@

# Here the target dependent .c files get created on the fly.
%.c:
	@echo "Creating $@ on the fly."
	@echo "/* gcc -I parameters will find the right one for our target MCU. */" > $@
	@echo "#include <$(shell F=$@ && echo $${F#mbed_})>" >> $@

# No general rule for this one.
startup_LPC11xx.o: $(MBED)/targets/cmsis/TARGET_NXP/TARGET_LPC11XX_11CXX/TOOLCHAIN_GCC_ARM/startup_LPC11xx.s
	$(CC) -c $(CFLAGS) $< -o $@

size: $(NAME).elf
	$(SIZE) -B $<

clean:
	rm -f *.hex *.elf *.map *.o mbed_*.c
