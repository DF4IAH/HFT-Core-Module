###############################################################################
#
#  	    File        : Makefile
#
#       Abstract    : Example Makefile for a C Project
#
#       Environment : Atollic TrueSTUDIO(R)
#
###############################################################################

SHELL=cmd

# System configuration
CC = arm-atollic-eabi-gcc
RM=rm -rf

# Assembler, Compiler and Linker flags and linker script settings
LINKER_FLAGS=-lm -mthumb -mhard-float -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -Wl,--gc-sections -T$(LINK_SCRIPT) -static  -Wl,--start-group -lc -lm -Wl,--end-group -specs=nano.specs -specs=nosys.specs  -Wl,-cref "-Wl,-Map=$(BIN_DIR)/HFT-Core-Module.map" -Wl,--defsym=malloc_getpagesize_P=0x1000
LINK_SCRIPT="stm32l4_flash.ld"
ASSEMBLER_FLAGS=-c -g -O0 -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mthumb -mhard-float -specs=nano.specs -D"STM32L496xx"  -x assembler-with-cpp  -Isrc -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32L4xx/Include -IDrivers/STM32L4xx_HAL_Driver/Inc
COMPILER_FLAGS=-c -g -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -O0 -Wall -ffunction-sections -fdata-sections -mthumb -mhard-float -specs=nano.specs -D"STM32L496xx"   -Isrc -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32L4xx/Include -IDrivers/STM32L4xx_HAL_Driver/Inc 

# Define output directory
OBJECT_DIR = Debug
BIN_DIR = $(OBJECT_DIR)

# Define sources and objects
SRC := $(wildcard */*/*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*.c) \
	$(wildcard */*/*/*/*.c) \
	$(wildcard */*/*/*.c) \
	$(wildcard */*/*.c) \
	$(wildcard */*.c)
SRCSASM := 	$(wildcard */*/*/*/*/*/*/*/*.s) \
	$(wildcard */*/*/*/*/*/*/*.s) \
	$(wildcard */*/*/*/*/*/*.s) \
	$(wildcard */*/*/*/*/*.s) \
	$(wildcard */*/*/*/*.s) \
	$(wildcard */*/*/*.s) \
	$(wildcard */*/*.s) \
	$(wildcard */*.s)
OBJS := $(SRC:%.c=$(OBJECT_DIR)/%.o) $(SRCSASM:%.s=$(OBJECT_DIR)/%.o)
OBJS := $(OBJS:%.S=$(OBJECT_DIR)/%.o)  

###############
# Build project
# Major targets
###############
all: buildelf

buildelf: $(OBJS) 
	$(CC) -o "$(BIN_DIR)/HFT-Core-Module.elf" $(OBJS) $(LINKER_FLAGS)

clean:
	$(RM) $(OBJS) "$(BIN_DIR)/HFT-Core-Module.elf" "$(BIN_DIR)/HFT-Core-Module.map"


##################
# Specific targets
##################
$(OBJECT_DIR)/src/main.o: src/main.c
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(COMPILER_FLAGS) src/main.c -o $(OBJECT_DIR)/src/main.o 


##################
# Implicit targets
##################
$(OBJECT_DIR)/%.o: %.c
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(COMPILER_FLAGS) $< -o $@

$(OBJECT_DIR)/%.o: %.s
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.S
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@