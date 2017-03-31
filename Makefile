#Build script for KR-firmware-platform for stm32f0

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size
ARCH=stm32f0

PROJ_NAME=kr_firmware_platform
ARCH_DIR=hardware/arch/$(ARCH)
# Location of the Libraries folder from the STM32F0xx Standard Peripheral Library
STD_PERIPH_LIB=$(ARCH_DIR)/libraries

# Location of the linker scripts
LDSCRIPT_INC=$(ARCH_DIR)/device/ldscripts

# location of OpenOCD Board .cfg files (only used with 'make program')
OPENOCD_BOARD_DIR=/usr/share/openocd/scripts/board

# Configuration (cfg) file containing programming directives for OpenOCD
OPENOCD_PROC_FILE=$(ARCH_DIR)/extra/stm32f0-openocd.cfg

# Colour code for printing

NC=\033[0m
RED=\033[0;31m
BLUE=\033[0;34m
GREEN=\033[0;32m
PURPLE=\033[0;35m

###################################################

CFLAGS  = -Wall -g -std=c99 -Os
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(PROJ_NAME).map
CFLAGS += -I $(STD_PERIPH_LIB) -I $(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32F0xx/Include
CFLAGS += -I $(STD_PERIPH_LIB)/CMSIS/Include -I $(STD_PERIPH_LIB)/STM32F0xx_StdPeriph_Driver/inc
CFLAGS += -include $(STD_PERIPH_LIB)/stm32f0xx_conf.h
CFLAGS += -I hardware/driver
CFLAGS += -I hardware/arch/common
CFLAGS += -I application
CFLAGS += -I service
###################################################

vpath %.c application service hardware/driver
vpath %.a $(STD_PERIPH_LIB)
vpath %.s $(ARCH_DIR)/device

BUILD_DIR 	= build
OUT 		= $(BUILD_DIR)/$(PROJ_NAME)

SRC_C = $(wildcard application/*.c)		\
	$(wildcard hardware/driver/*.c)		\
	$(wildcard hardware/arch/$(ARCH)/*.c)	\
	$(wildcard service/*.c)

# add startup file to build
SRC_S = $(wildcard $(ARCH_DIR)/device/*.s)

OBJ =  $(patsubst %c, $(BUILD_DIR)/%o, $(SRC_C))
OBJ += $(patsubst %s, $(BUILD_DIR)/%o, $(SRC_S))

###################################################
vpath %c application hardware/driver service
vpath %s $(ARCH_DIR)/device

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C $(STD_PERIPH_LIB)

proj: 	$(BUILD_DIR)/$(PROJ_NAME).elf

$(OUT).elf : $(OBJ)
	@echo "$(RED)Lingking ...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@$(CC) $(CFLAGS) $^ -o $@ -L$(STD_PERIPH_LIB) -lstm32f0 -L$(LDSCRIPT_INC) -Tstm32f0.ld -lm #add lm for math.h

	@$(OBJCOPY) -O ihex $(OUT).elf $(BUILD_DIR)/$(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(OUT).elf $(BUILD_DIR)/$(PROJ_NAME).bin
	@$(OBJDUMP) -St $(OUT).elf >$(BUILD_DIR)/$(PROJ_NAME).lst
	$(SIZE) $(OUT).elf
	@echo "$(RED)Successful! $(NC)"

$(BUILD_DIR)/%o:%c
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%o:%s
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS) -c $< -o $@

program: $(OUT).bin
	openocd -f $(OPENOCD_BOARD_DIR)/stm32f0discovery.cfg -f $(OPENOCD_PROC_FILE) -c "stm_flash `pwd`/build/bin/$(PROJ_NAME).bin" -c shutdown

debug_server: $(OUT).elf
	# . extra/debug_nemivier.sh
	openocd -f $(OPENOCD_BOARD_DIR)/stm32f0discovery.cfg
clean:
	find ./ -name '*~' | xargs rm -f
	rm -f *.o
	rm -rf $(BUILD_DIR)

distclean: clean
	find ./ -name '*~' | xargs rm -f
	rm -f *.o
	rm -rf $(BUILD_DIR)
	$(MAKE) -C $(STD_PERIPH_LIB) clean

.PHONY: clean debug_server debug_nemivier debug_cli
