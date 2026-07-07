######################################
# Makefile for motorcontrol firmware
# Mirrors the STM32CubeIDE build settings, but works standalone with
# the GNU Arm Embedded Toolchain (arm-none-eabi-gcc) and make.
#
# Targets:
#   make [all]    - build build/motorcontrol.elf/.bin/.hex
#   make clean    - remove the build directory
#   make flash    - flash over ST-Link (requires STM32_Programmer_CLI or st-flash on PATH)
######################################

TARGET = motorcontrol

# Set DEBUG=1 for a -Og/-g3 build (default is optimized)
DEBUG ?= 0
OPT = -O2
ifeq ($(DEBUG), 1)
OPT = -Og -g3
endif

BUILD_DIR = build

######################################
# toolchain
######################################
PREFIX = arm-none-eabi-
# Override with e.g. `make GCC_PATH=/path/to/toolchain/bin` if not on PATH
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
OD = $(GCC_PATH)/$(PREFIX)objdump
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
OD = $(PREFIX)objdump
endif

######################################
# sources
######################################
C_SOURCES = \
$(wildcard Core/Src/*.c) \
$(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)

ASM_SOURCES = \
Core/Startup/startup_stm32f446retx.s

######################################
# flags
######################################
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32F446xx

C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include

ASFLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=gnu11

# generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

######################################
# linker
######################################
LDSCRIPT = STM32F446RETX_FLASH.ld

LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -u _printf_float -T$(LDSCRIPT) $(LIBS) \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

######################################
# build rules
######################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# list of objects (flattened into build dir; source basenames are unique)
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile $(LDSCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary -S $< $@

$(BUILD_DIR)/%.list: $(BUILD_DIR)/%.elf
	$(OD) -h -S $< > $@

$(BUILD_DIR):
	mkdir -p $@

######################################
# flash (pick whichever tool you have)
######################################
flash: $(BUILD_DIR)/$(TARGET).elf
	@STM32_Programmer_CLI -c port=SWD -w $(BUILD_DIR)/$(TARGET).elf -v -rst 2>/dev/null || \
	st-flash --reset write $(BUILD_DIR)/$(TARGET).bin 0x8000000

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean flash
