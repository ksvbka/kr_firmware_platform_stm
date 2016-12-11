#Build script for KR-firmware-platform for stm32f0

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size

PROJ_NAME=kr_firmware_platform

# Location of the Libraries folder from the STM32F0xx Standard Peripheral Library
STD_PERIPH_LIB=libraries

# Location of the linker scripts
LDSCRIPT_INC=device/ldscripts

# location of OpenOCD Board .cfg files (only used with 'make program')
OPENOCD_BOARD_DIR=/usr/share/openocd/scripts/board

# Configuration (cfg) file containing programming directives for OpenOCD
OPENOCD_PROC_FILE=extra/stm32f0-openocd.cfg

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
SRC_STARTUP_DIR = $(ROOT)/device/
SRC_STARTUP = $(SRC_STARTUP_DIR)/startup_stm32f0xx.s
OBJ_STARTUP = $(patsubst %.s,$(OBJ_DIR)/%.o,$(notdir $(SRC_STARTUP)))

# Hardware object
SRC_HWDIR 	= $(ROOT)/hardware/src
SRC_HW 		= $(wildcard $(SRC_HWDIR)/*.c)
OBJ_HW 		= $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(SRC_HW)))

# Service source dir
SRC_SVDIR 	= $(ROOT)/service/src
SRC_SV 		= $(wildcard $(SRC_SVDIR)/*.c)
OBJ_SV		= $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(SRC_SV)))

# Application source dir
SRC_APPDIR 	= $(ROOT)/application
SRC_APP 	= $(wildcard $(SRC_APPDIR)/*.c)
OBJ_APP 	= $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(SRC_APP)))

CFLAGS += -I $(STD_PERIPH_LIB) -I $(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32F0xx/Include
CFLAGS += -I $(STD_PERIPH_LIB)/CMSIS/Include -I $(STD_PERIPH_LIB)/STM32F0xx_StdPeriph_Driver/inc
CFLAGS += -include $(STD_PERIPH_LIB)/stm32f0xx_conf.h
CFLAGS += -I $(ROOT)/hardware/include
CFLAGS += -I $(ROOT)/service/include
CFLAGS += -I $(ROOT)/application

# need if you want to build with -DUSE_CMSIS
#SRCS += stm32f0_discovery.c
#SRCS += stm32f0_discovery.c stm32f0xx_it.c

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C $(STD_PERIPH_LIB)

proj: 	$(BIN_DIR)/$(PROJ_NAME).elf

$(OUT).elf : $(OBJ_HW) $(OBJ_SV) $(OBJ_APP) $(OBJ_STARTUP)
	@echo Lingking ...
	@mkdir -p $(BIN_DIR)
	@$(CC) $(CFLAGS) $^ -o $@ -L$(STD_PERIPH_LIB) -lstm32f0 -L$(LDSCRIPT_INC) -Tstm32f0.ld

	@$(OBJCOPY) -O ihex $(OUT).elf $(BIN_DIR)/$(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(OUT).elf $(BIN_DIR)/$(PROJ_NAME).bin
	@$(OBJDUMP) -St $(OUT).elf >$(BIN_DIR)/$(PROJ_NAME).lst
	$(SIZE) $(OUT).elf
	@echo Successful!

$(OBJ_DIR)/%.o:$(SRC_STARTUP_DIR)/%.s
	@echo Building $(notdir $<) ...
	@mkdir -p $(OBJ_DIR)
	@$(CC) $(CFLAGS) -c $< -o $@

# Bulding hardware object
$(OBJ_DIR)/%.o:$(SRC_HWDIR)/%.c
	@echo Building $(notdir $<) ...
	@mkdir -p $(OBJ_DIR)
	@$(CC) $(CFLAGS) -c $< -o $@

# Bulding service object
$(OBJ_DIR)/%.o:$(SRC_SVDIR)/%.c
	@echo Building $(notdir $<) ...
	@mkdir -p $(OBJ_DIR)
	@$(CC) $(CFLAGS) -c $< -o $@

# Bulding application object
$(OBJ_DIR)/%.o:$(SRC_APPDIR)/%.c
	@echo Building $(notdir $<) ...
	@mkdir -p $(OBJ_DIR)
	@$(CC) $(CFLAGS) -c $< -o $@

program: $(OUT).bin
	openocd -f $(OPENOCD_BOARD_DIR)/stm32f0discovery.cfg -f $(OPENOCD_PROC_FILE) -c "stm_flash `pwd`/build/bin/$(PROJ_NAME).bin" -c shutdown

debug: $(OUT).elf
	. extra/debug_nemiver.sh

clean:
	find ./ -name '*~' | xargs rm -f
	rm -f *.o
	rm -rf $(BUILD_DIR)

distclean: clean
	find ./ -name '*~' | xargs rm -f
	rm -f *.o
	rm -rf $(BUILD_DIR)
	$(MAKE) -C $(STD_PERIPH_LIB) clean
