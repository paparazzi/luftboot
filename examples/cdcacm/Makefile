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
	-Wl,-T,cdcacm.ld -nostartfiles -lc -lnosys -Wl,-Map=mapfile \
	-mthumb -march=armv7 -mcpu=cortex-m3 -mfix-cortex-m3-ldrd -msoft-float -L$(LIBOPENCM3)/lib/stm32/f1
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8002000

all: cdcacm.bin cdcacm.hex

cdcacm: cdcacm.o
	$(CC) $^ -o $@ $(LDFLAGS)

cdcacm.bin: cdcacm
	$(OBJCOPY) -O binary $^ $@

cdcacm.hex: cdcacm
	$(OBJCOPY) -O ihex $^ $@

clean:
	-rm -rf cdcacm.o cdcacm cdcacm.bin cdcacm.hex mapfile
