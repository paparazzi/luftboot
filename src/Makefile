CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
LIBOPENCM3 ?= /usr/local/arm-none-eabi
OOCD_INTERFACE  ?= flossjtag
OOCD_TARGET     ?= stm32
OOCD = openocd
Q = @


CFLAGS += -Istm32/include -mcpu=cortex-m3 -mthumb -msoft-float -DSTM32F1 -I$(LIBOPENCM3)/include
LDFLAGS_BOOT = -lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,luftboot.ld -nostartfiles -lc -lnosys -Wl,-Map=mapfile \
	-mthumb -march=armv7 -mcpu=cortex-m3 -mfix-cortex-m3-ldrd -msoft-float -L$(LIBOPENCM3)/lib/stm32/f1
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8002000

all: luftboot.bin luftboot.hex

luftboot: luftboot.o
	$(CC) $^ -o $@ $(LDFLAGS_BOOT)

luftboot.bin: luftboot
	$(OBJCOPY) -O binary $^ $@

luftboot.hex: luftboot
	$(OBJCOPY) -O ihex $^ $@

clean:
	-rm -rf luftboot.o luftboot luftboot.bin luftboot.hex mapfile

flash: luftboot.hex
	@echo "  OOCD  luftboot.hex"
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		-f target/$(OOCD_TARGET).cfg \
		-c init \
		-c "reset init" \
		-c "stm32f1x mass_erase 0" \
		-c "flash write_image luftboot.hex" \
		-c reset \
		-c shutdown
