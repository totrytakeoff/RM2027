# =============================================================================
# @file tests/firmware/infantry/infantry_omni_demo/CMakeLists.txt
# @brief Infantry omni demo CMake config
# @project RM2026
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(infantry_omni_demo C ASM)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

create_embedded_test(infantry_omni_demo
    SOURCES
        main.c
    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
    LINK_LIBRARIES
        rm_components
        rm_platform_stm32f4
        rm_vendor_runtime
        rm_common
        rm_vendor_stm32_hal
    LINKER_SCRIPT ${MCU_LINKER_SCRIPT}
)
