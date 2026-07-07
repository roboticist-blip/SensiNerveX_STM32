##############################################################################
# SensiNerveX STM32F405 — Terminal Makefile
#
# Toolchain: arm-none-eabi-gcc (GNU Arm Embedded Toolchain 13+)
# Target:    STM32F405RGT6 (ARM Cortex-M4F, 168 MHz, 1 MB Flash, 192 KB SRAM)
#
# Usage:
#   make all        — build firmware ELF + binary + hex
#   make flash      — flash via OpenOCD (ST-Link)
#   make flash_stl  — flash via st-flash (stlink-tools)
#   make clean      — remove build artifacts
#   make size       — print section sizes
#   make disasm     — generate annotated disassembly
#
##############################################################################

# ============================================================================
# TOOLCHAIN
# ============================================================================
PREFIX   := arm-none-eabi-
CC       := $(PREFIX)gcc
AS       := $(PREFIX)as
LD       := $(PREFIX)gcc
OBJCOPY  := $(PREFIX)objcopy
OBJDUMP  := $(PREFIX)objdump
SIZE     := $(PREFIX)size
GDB      := $(PREFIX)gdb

TARGET   := SensiNerveX_STM32
BUILD    := build
ELF      := $(BUILD)/$(TARGET).elf
BIN      := $(BUILD)/$(TARGET).bin
HEX      := $(BUILD)/$(TARGET).hex
MAP      := $(BUILD)/$(TARGET).map

MCU_FLAGS := \
    -mcpu=cortex-m4          \
    -mthumb                  \
    -mfpu=fpv4-sp-d16        \
    -mfloat-abi=hard         \
    -mabi=aapcs

DEFS := \
    -DSTM32F405xx            \
    -DUSE_HAL_DRIVER         \
    -DARM_MATH_CM4           \


INC := \
    -ICore/Inc                                              \
    -IDrivers/STM32F4xx_HAL_Driver/Inc                     \
    -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy               \
    -IDrivers/CMSIS/Device/ST/STM32F4xx/Include             \
    -IDrivers/CMSIS/Include                                 \
    -IMiddlewares/Third_Party/FatFs/src

CUSTOM_SRCS := \
    Core/Src/main.c              \
    Core/Src/MPU6050.c           \
    Core/Src/ComplementaryFilter.c \
    Core/Src/FeatureExtractor.c  \
    Core/Src/NeuralNetwork.c     \
    Core/Src/Serialization.c     \
    Core/Src/FederatedClient.c   \
    Core/Src/Utils.c             \
    Core/Src/AppModule.c         \
    Core/Src/SDCard.c            \
    Core/Src/diskio.c            \
    Core/Src/DataLogger.c
FATFS_SRCS := \
    Middlewares/Third_Party/FatFs/src/ff.c        \
    Middlewares/Third_Party/FatFs/src/ffunicode.c

HAL_SRCS := \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c             \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c      \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c         \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c      \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c        \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c        \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c         \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c         \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c      \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c         \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c      \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c         \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c       \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c    \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c          \
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c

CUBE_SRCS := \
    Core/Src/stm32f4xx_hal_msp.c    \
    Core/Src/stm32f4xx_it.c         \
    Core/Src/system_stm32f4xx.c

STARTUP_SRC := \
    Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f405xx.s

FATFS_PRESENT := $(wildcard Middlewares/Third_Party/FatFs/src/ff.c)
ifneq ($(FATFS_PRESENT),)
SRCS := $(CUSTOM_SRCS) $(FATFS_SRCS) $(HAL_SRCS) $(CUBE_SRCS)
else
SRCS := $(CUSTOM_SRCS) $(HAL_SRCS) $(CUBE_SRCS)
endif
OBJS := $(patsubst %.c,$(BUILD)/%.o,$(SRCS))
STARTUP_OBJ := $(BUILD)/startup_stm32f405xx.o

OPTFLAGS := -O2           

WARNFLAGS := \
    -Wall -Wextra -Wshadow -Wdouble-promotion \
    -Wformat=2 -Wundef -Wno-unused-parameter

CFLAGS := \
    $(MCU_FLAGS)         \
    $(OPTFLAGS)          \
    $(WARNFLAGS)         \
    $(DEFS)              \
    $(INC)               \
    -std=c11             \
    -ffunction-sections  \
    -fdata-sections      \
    -fno-common          \
    -fstack-usage        \
    --specs=nano.specs   \
    -fno-strict-aliasing

ASFLAGS := \
    $(MCU_FLAGS)   \
    -x assembler-with-cpp

LINKER_SCRIPT := STM32CubeMX/STM32F405RGTx_FLASH.ld

LDFLAGS := \
    $(MCU_FLAGS)                    \
    -T$(LINKER_SCRIPT)              \
    -Wl,-Map=$(MAP),--cref          \
    -Wl,--gc-sections               \
    -Wl,--print-memory-usage        \
    -Wl,-u,_printf_float            \
    --specs=nano.specs              \
    --specs=nosys.specs             \
    -lm                             \
    -lc

.PHONY: all clean flash flash_stl size disasm

all: $(BUILD) $(ELF) $(BIN) $(HEX) size

$(BUILD):
	@mkdir -p $(BUILD)
	@mkdir -p $(dir $(OBJS))

# Compile C sources
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "  CC   $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble startup file
$(STARTUP_OBJ): $(STARTUP_SRC)
	@mkdir -p $(dir $@)
	@echo "  AS   $<"
	$(CC) $(ASFLAGS) -c $< -o $@

# Link
$(ELF): $(OBJS) $(STARTUP_OBJ)
	@echo "  LD   $@"
	$(LD) $(OBJS) $(STARTUP_OBJ) $(LDFLAGS) -o $@

# Generate binary (for st-flash)
$(BIN): $(ELF)
	@echo "  BIN  $@"
	$(OBJCOPY) -O binary -S $< $@

# Generate hex (for OpenOCD)
$(HEX): $(ELF)
	@echo "  HEX  $@"
	$(OBJCOPY) -O ihex $< $@

# Print section sizes
size: $(ELF)
	@echo ""
	@echo "--- Section sizes ---"
	$(SIZE) --format=berkeley $(ELF)
	@echo ""

# Generate disassembly (useful for optimization review)
disasm: $(ELF)
	$(OBJDUMP) -d -S --demangle $(ELF) > $(BUILD)/$(TARGET).dis
	@echo "Disassembly written to $(BUILD)/$(TARGET).dis"

clean:
	@echo "  CLEAN"
	rm -rf $(BUILD)

OPENOCD_CFG := -f interface/stlink.cfg -f target/stm32f4x.cfg

flash: $(ELF)
	@echo "Flashing via OpenOCD (ST-Link)..."
	openocd $(OPENOCD_CFG) \
	    -c "program $(ELF) verify reset exit"

# Flash using st-flash (stlink-tools, binary format)
flash_stl: $(BIN)
	@echo "Flashing via st-flash..."
	st-flash --reset write $(BIN) 0x8000000

# GDB debug session via OpenOCD
debug: $(ELF)
	@echo "Starting GDB debug session..."
	openocd $(OPENOCD_CFG) &
	$(GDB) $(ELF) -ex "target remote localhost:3333" -ex "monitor reset halt"

