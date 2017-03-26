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
CFLAGS += -Wl,--gc-sections -Wl,-Map=$(BIN_DIR)/$(PROJ_NAME).map
###################################################

vpath %.c application
vpath %.a $(STD_PERIPH_LIB)

ROOT=$(shell pwd)
BUILD_DIR 	= $(ROOT)/build
OBJ_DIR		= $(BUILD_DIR)/obj
BIN_DIR		= $(BUILD_DIR)/bin
OUT 		= $(BIN_DIR)/$(PROJ_NAME)

# add startup file to build
SRC_STARTUP_DIR = $(ROOT)/$(ARCH_DIR)/device/
SRC_STARTUP 	= $(SRC_STARTUP_DIR)/startup_stm32f0xx.s
OBJ_STARTUP 	= $(patsubst %.s,$(OBJ_DIR)/%.o,$(notdir $(SRC_STARTUP)))

# Hardware core object
SRC_CORE_DIR 	= $(ROOT)/$(ARCH_DIR)
SRC_CORE 	= $(wildcard $(SRC_CORE_DIR)/*.c)
OBJ_CORE 	= $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(SRC_CORE)))

# Driver object
SRC_DRIVER_DIR 	= $(ROOT)/hardware/driver
SRC_DRIVER	= $(wildcard $(SRC_DRIVER_DIR)/*.c)
OBJ_DRIVER	= $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(SRC_DRIVER)))

# Service source dir
SRC_SVDIR 	= $(ROOT)/service
SRC_SV 		= $(wildcard $(SRC_SVDIR)/*.c)
OBJ_SV		= $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(SRC_SV)))

# Application source dir
SRC_APPDIR 	= $(ROOT)/application
SRC_APP 	= $(wildcard $(SRC_APPDIR)/*.c)
OBJ_APP 	= $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(SRC_APP)))

CFLAGS += -I $(STD_PERIPH_LIB) -I $(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32F0xx/Include
CFLAGS += -I $(STD_PERIPH_LIB)/CMSIS/Include -I $(STD_PERIPH_LIB)/STM32F0xx_StdPeriph_Driver/inc
CFLAGS += -include $(STD_PERIPH_LIB)/stm32f0xx_conf.h
CFLAGS += -I $(ROOT)/hardware/driver
CFLAGS += -I $(ROOT)/hardware/arch/common
CFLAGS += -I $(ROOT)/application
CFLAGS += -I $(ROOT)/service

# need if you want to build with -DUSE_CMSIS
#SRCS += stm32f0_discovery.c
#SRCS += stm32f0_discovery.c stm32f0xx_it.c

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C $(STD_PERIPH_LIB)

proj: 	$(BIN_DIR)/$(PROJ_NAME).elf

$(OUT).elf : $(OBJ_CORE) $(OBJ_DRIVER) $(OBJ_SV) $(OBJ_APP) $(OBJ_STARTUP)
	@echo "$(RED)Lingking ...$(NC)"
	@mkdir -p $(BIN_DIR)
	@$(CC) $(CFLAGS) $^ -o $@ -L$(STD_PERIPH_LIB) -lstm32f0 -L$(LDSCRIPT_INC) -Tstm32f0.ld -lm #add lm for math.h

	@$(OBJCOPY) -O ihex $(OUT).elf $(BIN_DIR)/$(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(OUT).elf $(BIN_DIR)/$(PROJ_NAME).bin
	@$(OBJDUMP) -St $(OUT).elf >$(BIN_DIR)/$(PROJ_NAME).lst
	$(SIZE) $(OUT).elf
	@echo "$(RED)Successful! $(NC)"

$(OBJ_DIR)/%.o:$(SRC_STARTUP_DIR)/%.s
	@echo  "$(GREEN)Building $(notdir $<)...$(NC)"
	@mkdir -p $(OBJ_DIR)
	@$(CC) $(CFLAGS) -c $< -o $@

# Bulding hardware object
$(OBJ_DIR)/%.o:$(SRC_CORE_DIR)/%.c
	@echo "$(GREEN)Building $(notdir $<)...$(NC)"
	@mkdir -p $(OBJ_DIR)
	@$(CC) $(CFLAGS) -c $< -o $@

# Bulding peripherals object
$(OBJ_DIR)/%.o:$(SRC_DRIVER_DIR)/%.c
	@echo "$(GREEN)Building $(notdir $<)...$(NC)"
	@mkdir -p $(OBJ_DIR)
	@$(CC) $(CFLAGS) -c $< -o $@

# Bulding service object
$(OBJ_DIR)/%.o:$(SRC_SVDIR)/%.c
	@echo "$(GREEN)Building $(notdir $<)...$(NC)"
	@mkdir -p $(OBJ_DIR)
	@$(CC) $(CFLAGS) -c $< -o $@

# Bulding application object
$(OBJ_DIR)/%.o:$(SRC_APPDIR)/%.c
	@echo "$(GREEN)Building $(notdir $<)...$(NC)"
	@mkdir -p $(OBJ_DIR)
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
