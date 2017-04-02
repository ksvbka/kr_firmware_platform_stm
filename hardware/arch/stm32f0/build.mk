# @Author: Trung Kien
# @Date:   2017-04-01 14:46:44
# @Propose: Define specific LINK_FLAGS for earch architecture
#

# Location of the Libraries folder from the STM32F0xx Standard Peripheral Library
STD_PERIPH_LIB 	= $(ARCH_DIR)/libraries

vpath %.a $(STD_PERIPH_LIB)
vpath %.s $(ARCH_DIR)/device

# Startup code
SRC_S 	+= $(wildcard $(ARCH_DIR)/device/*.s)
OBJ  	+= $(patsubst %s, $(BUILD_DIR)/%o, $(SRC_S))

# CFLAGS config for STM32F0
CFLAGS 	+= -ffunction-sections -fdata-sections
CFLAGS 	+= -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(PROJ_NAME).map
CFLAGS 	+= -I $(STD_PERIPH_LIB) -I $(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32F0xx/Include
CFLAGS 	+= -I $(STD_PERIPH_LIB)/CMSIS/Include -I $(STD_PERIPH_LIB)/STM32F0xx_StdPeriph_Driver/inc
CFLAGS 	+= -include $(STD_PERIPH_LIB)/stm32f0xx_conf.h
CFLAGS 	+= -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb


# Location of the linker scripts
LDSCRIPT_INC 	= $(ARCH_DIR)/device/ldscripts

LINK_FLAGS 	+= -L$(STD_PERIPH_LIB) -L$(LDSCRIPT_INC)

LINK_FLAGS += -lstm32f0
LINK_FLAGS += -Tstm32f0.ld
