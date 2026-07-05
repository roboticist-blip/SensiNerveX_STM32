##############################################################################
# FedVibroSense STM32F405 — CMakeLists.txt
#
# CMake alternative to the Makefile.  Useful when integrating with
# CLion, VS Code CMake Tools, or CI pipelines.
#
# Usage:
#   mkdir build && cd build
#   cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake
#   make -j4
#   make flash
##############################################################################

cmake_minimum_required(VERSION 3.20)

# ============================================================================
# PROJECT
# ============================================================================
project(FedVibroSense
    VERSION 1.0.0
    LANGUAGES C ASM
    DESCRIPTION "STM32F405 Federated Learning Vibration Client"
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)   # Allow GNU C extensions (__attribute__ etc.)

# ============================================================================
# MCU SETTINGS
# ============================================================================
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mabi=aapcs
)

# ============================================================================
# COMPILE DEFINITIONS
# ============================================================================
set(MCU_DEFINES
    STM32F405xx
    USE_HAL_DRIVER
    ARM_MATH_CM4
    __FPU_PRESENT=1
    USE_FULL_ASSERT
)

# ============================================================================
# SOURCE FILES
# ============================================================================

# Custom firmware sources (manually written)
set(CUSTOM_SOURCES
    Core/Src/main.c
    Core/Src/MPU6050.c
    Core/Src/ComplementaryFilter.c
    Core/Src/FeatureExtractor.c
    Core/Src/NeuralNetwork.c
    Core/Src/Serialization.c
    Core/Src/FederatedClient.c
    Core/Src/Utils.c
    Core/Src/AppModule.c
    Core/Src/SDCard.c
    Core/Src/diskio.c
    Core/Src/DataLogger.c
)
# DataLogger.c only #includes "ff.h" when Config.h's SD_LOGGING_ENABLE is 1
# (the shipped default). Add the FatFs core (see FATFS_SOURCES below) or
# set SD_LOGGING_ENABLE to 0 before configuring if you haven't added it yet.

# CubeMX-generated application stubs
set(CUBE_SOURCES
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_it.c
    Core/Src/system_stm32f4xx.c
)

# HAL driver sources (from STM32CubeMX Drivers/)
set(HAL_SOURCES
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
)
# stm32f4xx_hal_sd.c + ll_sdmmc.c: SDIO block driver for the WeAct
# STM32F405RGT6 microSD socket. Requires HAL_SD_MODULE_ENABLED, already
# turned on in Core/Inc/stm32f4xx_hal_conf.h.

# FatFs core — third-party middleware, NOT included in this repo. Add it
# at Middlewares/Third_Party/FatFs/src/ (via STM32CubeMX's FATFS
# middleware or a direct download — see Core/Inc/ffconf.h header comment)
# and this glob will pick it up automatically; otherwise it's silently
# empty and DataLogger.c's "ff.h" include will fail at configure/build
# time if SD_LOGGING_ENABLE is left at its default of 1 in Config.h.
file(GLOB FATFS_SOURCES
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FatFs/src/ff.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FatFs/src/ffunicode.c
)

# Startup assembly file
set(STARTUP_SOURCE
    Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f405xx.s
)

# ============================================================================
# INCLUDE DIRECTORIES
# ============================================================================
set(INCLUDE_DIRS
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Middlewares/Third_Party/FatFs/src
)

# ============================================================================
# LINKER SCRIPT
# ============================================================================
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32CubeMX/STM32F405RGTx_FLASH.ld)

# ============================================================================
# EXECUTABLE TARGET
# ============================================================================
add_executable(${PROJECT_NAME}.elf
    ${CUSTOM_SOURCES}
    ${CUBE_SOURCES}
    ${HAL_SOURCES}
    ${FATFS_SOURCES}
    ${STARTUP_SOURCE}
)

target_include_directories(${PROJECT_NAME}.elf PRIVATE ${INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE ${MCU_DEFINES})

# Apply MCU flags to both compile and link steps
target_compile_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    -O2
    -Wall
    -Wextra
    -Wshadow
    -Wdouble-promotion
    -ffunction-sections
    -fdata-sections
    -fno-common
    -fstack-usage
    --specs=nano.specs
    -fno-strict-aliasing
    -std=c11
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    --specs=nano.specs
    --specs=nosys.specs
    -lm
    -lc
)

# ============================================================================
# POST-BUILD: generate .bin and .hex
# ============================================================================
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary -S
        ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND arm-none-eabi-objcopy -O ihex
        ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND arm-none-eabi-size --format=berkeley ${PROJECT_NAME}.elf
    COMMENT "Generating .bin and .hex, printing section sizes"
)

# ============================================================================
# FLASH TARGET
# ============================================================================
add_custom_target(flash
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
        -c "program ${PROJECT_NAME}.elf verify reset exit"
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing via OpenOCD (ST-Link)"
)

add_custom_target(flash_stl
    COMMAND st-flash --reset write ${PROJECT_NAME}.bin 0x8000000
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing via st-flash"
)
