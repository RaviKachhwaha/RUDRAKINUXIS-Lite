cmake_minimum_required(VERSION 3.20)

# Project Definition
project(RUDRAKINUXIS_Lite_Firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Microcontroller Target Architecture
set(TARGET_MCU "STM32G431xx")
set(ARM_OPTIONS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)

# Compiler & Optimization Flags
add_compile_definitions(
    ${TARGET_MCU}
    USE_HAL_DRIVER
    USE_FULL_LL_DRIVER
    ARM_MATH_CM4
    __FPU_PRESENT=1
)

add_compile_options(
    ${ARM_OPTIONS}
    -O3
    -ffast-math
    -Wall
    -Wextra
    -fdata-sections
    -ffunction-sections
    -fno-strict-aliasing
)

add_link_options(
    ${ARM_OPTIONS}
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -specs=nano.specs
    -specs=nosys.specs
    -u _printf_float
)

# Include Directories
include_directories(
    Core/Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/STM32G4xx_HAL_Driver/Inc
)

# Source Files
file(GLOB_RECURSE CORE_SOURCES
    "Core/Src/*.c"
)

add_executable(${PROJECT_NAME}.elf ${CORE_SOURCES})

target_link_libraries(${PROJECT_NAME}.elf
    m
)

# Post-Build Commands: Generate .bin and .hex
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND arm-none-eabi-objcopy -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)
