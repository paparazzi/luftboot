PREFIX ?= arm-none-eabi

CC = $(PREFIX)-gcc
OBJCOPY = $(PREFIX)-objcopy

GDB?=$(PREFIX)-gdb

LIBOPENCM3 ?= libopencm3

# flashing options
OOCD_INTERFACE  ?= flossjtag
OOCD_TARGET     ?= stm32
OOCD = openocd
BMP_PORT =

VERSION = V1.1
DEV_SERIAL = NSERIAL

# Check if we are using gcc-arm-embedded compiler
# if so use nano.specs to save space
ifeq ($(shell $(CC) -specs=nano.specs 2>&1 | grep nano.specs),)
ifeq ($(V),1)
$(info Using gcc-arm-embedded, nano specs enabled!!!)
endif
SPECS = -specs=nano.specs
else
ifeq ($(V),1)
$(info Not using gcc-arm-embedded, nano specs disabled!!!)
endif
endif

CFLAGS += -Os -g3 -Istm32/include -mcpu=cortex-m3 -mthumb -msoft-float -DSTM32F1 \
	-I$(LIBOPENCM3)/include --function-sections --data-sections \
	-DVERSION="\"$(VERSION)\"" -DDEV_SERIAL="\"$(DEV_SERIAL)\""
LDFLAGS_BOOT = -lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,luftboot.ld $(SPECS) -nostartfiles -lc -lnosys -Wl,-Map=mapfile \
	-mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd -msoft-float -L$(LIBOPENCM3)/lib \
	-Wl,--gc-sections
LDFLAGS_BOOT += -Wl,--section-start=.devserial=0x8001FF0
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8002000

ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
else
LDFLAGS_BOOT += -Wl,--print-gc-sections
endif

ifeq ($(LUFTBOOT_USE_48MHZ_INTERNAL_OSC),1)
CFLAGS += -DLUFTBOOT_USE_48MHZ_INTERNAL_OSC=1
endif

all: luftboot.bin luftboot.hex

luftboot.elf: luftboot.o
	@echo "  CC      luftboot.elf"
	$(Q)$(CC) -o $@ $^ $(LDFLAGS_BOOT)

luftboot.bin: luftboot.elf
	@echo "  create  luftboot.bin"
	$(Q)$(Q)$(OBJCOPY) -O binary $^ $@

luftboot.hex: luftboot.elf
	@echo "  create  luftboot.hex"
	$(Q)$(OBJCOPY) -O ihex $^ $@

%.o: %.c
	@printf "  CC      $(<F)\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

clean:
	@echo "  CLEAN   luftboot"
	$(Q)-rm -rf luftboot.o luftboot.elf luftboot.bin luftboot.hex mapfile

ifeq ($(BMP_PORT),)
flash: luftboot.hex
	@echo "  OOCD  luftboot.hex"
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		-f target/$(OOCD_TARGET).cfg \
		-c init \
		-c "reset init" \
		-c "flash write_image erase luftboot.hex" \
		-c reset \
		-c shutdown
else
flash: luftboot.elf
	@echo "Using Black Magic Probe"
	@echo "Using GDB = $(GDB)"
	@echo "  BMP\t$<"
	$(Q)$(GDB) --batch \
		-ex 'target extended-remote $(BMP_PORT)' \
		-x ../scripts/black_magic_probe_flash.scr \
		$<
endif
