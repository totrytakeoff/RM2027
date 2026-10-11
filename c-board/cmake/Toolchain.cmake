# =============================================================================
# @file cmake/Toolchain.cmake
# @brief RM2026 ARM Cortex-M4 交叉编译工具链配置
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 统一设置 arm-none-eabi 编译器、MCU 编译/链接参数、链接脚本路径与 DSP 库，供所有 CMake 目标共享。
# =============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ----------------------------------------------------------------------------- 
# Locate toolchain binaries
# -----------------------------------------------------------------------------
find_program(ARM_NONE_EABI_GCC arm-none-eabi-gcc)
find_program(ARM_NONE_EABI_GPP arm-none-eabi-g++)
find_program(ARM_NONE_EABI_OBJCOPY arm-none-eabi-objcopy)
find_program(ARM_NONE_EABI_SIZE arm-none-eabi-size)

if(NOT ARM_NONE_EABI_GCC OR NOT ARM_NONE_EABI_GPP)
    message(FATAL_ERROR "arm-none-eabi-gcc/g++ not found. Please install the ARM GCC toolchain or add it to PATH.")
endif()

set(CMAKE_C_COMPILER   ${ARM_NONE_EABI_GCC})
set(CMAKE_CXX_COMPILER ${ARM_NONE_EABI_GPP})
set(CMAKE_ASM_COMPILER ${ARM_NONE_EABI_GCC})
set(CMAKE_OBJCOPY      ${ARM_NONE_EABI_OBJCOPY})
set(CMAKE_SIZE         ${ARM_NONE_EABI_SIZE})

# ----------------------------------------------------------------------------- 
# MCU flags and common compile options
# -----------------------------------------------------------------------------
set(MCU_FLAGS "-mcpu=cortex-m4;-mthumb;-mthumb-interwork;-mfloat-abi=hard;-mfpu=fpv4-sp-d16"
    CACHE STRING "MCU specific compiler flags")

set(MCU_LINKER_SCRIPT "${RM_BOARD_ROOT}/linker/STM32F407IGHx_FLASH.ld"
    CACHE FILEPATH "Linker script for STM32F407IGHx")

set(DSP_LIB "${CMAKE_SOURCE_DIR}/third_party/cmsis_dsp/Lib/libarm_cortexM4lf_math.a"
    CACHE FILEPATH "ARM CMSIS-DSP static library")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -fdiagnostics-color=always")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")

add_compile_definitions(
    USE_HAL_DRIVER
    STM32F407xx
    ARM_MATH_CM4
)

add_compile_options(
    ${MCU_FLAGS}
    -pipe
    -Wall
    -Wno-unused-variable
    -fmessage-length=0
    -ffunction-sections
    -fdata-sections
    -fno-common
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Og -g -gdwarf-2)
    add_compile_definitions(DEBUG)
else()
    # Keep IEEE-754 exceptional values observable in safety checks.  The
    # previous -Ofast implied -ffinite-math-only and could turn isfinite()
    # guards into unconditional true results in Release firmware.
    add_compile_options(-O3)
endif()

# ----------------------------------------------------------------------------- 
# Common link options
# -----------------------------------------------------------------------------
set(GENERIC_LINK_OPTIONS
    ${MCU_FLAGS}
    -Wl,--gc-sections
    # -Wl,--no-warn-rwx-segments  # 不支持，已注释
    -specs=nano.specs
    -specs=nosys.specs
    CACHE STRING "Common linker options for Cortex-M4"
)
