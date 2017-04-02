#Build script for KR-firmware-platform for stm32f0

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size
ARCH=stm32f0

PROJ_NAME	= kr_firmware_platform
BUILD_DIR 	= build
OUT 		= $(BUILD_DIR)/$(PROJ_NAME)
TARGET 		= $(BUILD_DIR)/$(PROJ_NAME).elf

ARCH_DIR=hardware/arch/$(ARCH)

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

CFLAGS += -I hardware/driver
CFLAGS += -I hardware/arch/common
CFLAGS += -I application
CFLAGS += -I service

LINK_FLAGS  = -lm #add lm for math.h

###################################################

vpath %.c application service hardware/driver
vpath %.s $(ARCH_DIR)/device

# Source for C code
SRC_C = $(wildcard application/*.c)		\
	$(wildcard hardware/driver/*.c)		\
	$(wildcard hardware/arch/$(ARCH)/*.c)	\
	$(wildcard service/*.c)

# Source for assembly code
SRC_S =

OBJ =  $(patsubst %c, $(BUILD_DIR)/%o, $(SRC_C))

###################################################
vpath %c application hardware/driver service
vpath %s $(ARCH_DIR)/device

# Include specific config from architecture
include $(ARCH_DIR)/build.mk

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C $(STD_PERIPH_LIB)

proj: 	$(TARGET)

$(TARGET) : $(OBJ)
	@echo "$(RED)Lingking ...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@$(CC) $(CFLAGS) $^ -o $@ $(LINK_FLAGS)

	@$(OBJCOPY) -O ihex   $(TARGET)   $(OUT).hex
	@$(OBJCOPY) -O binary $(TARGET)   $(OUT).bin
	@$(OBJDUMP) -St       $(TARGET) > $(OUT).lst
	$(SIZE) $(TARGET)

	@echo "$(RED)Successful! $(NC)"

# Build C code
$(BUILD_DIR)/%o:%c
	@mkdir -p $(dir $@)
	@echo "Building file: $< ..."
	@$(CC) $(CFLAGS) -c $< -o $@

# Build Assembly code
$(BUILD_DIR)/%o:%s
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS) -c $< -o $@

.PHONY: program

program: $(OUT).bin
	openocd -f $(OPENOCD_BOARD_DIR)/stm32f0discovery.cfg -f $(OPENOCD_PROC_FILE) -c "stm_flash `pwd` $(OUT).bin" -c shutdown

.PHONY: debug_server

debug_server: $(TARGET)
	# . extra/debug_nemivier.sh
	openocd -f $(OPENOCD_BOARD_DIR)/stm32f0discovery.cfg

.PHONY: clean distclean

clean:
	find ./ -name '*~' | xargs rm -f
	rm -f *.o
	rm -rf $(BUILD_DIR)

distclean: clean
	find ./ -name '*~' | xargs rm -f
	rm -f *.o
	rm -rf $(BUILD_DIR)
	$(MAKE) -C $(STD_PERIPH_LIB) clean
