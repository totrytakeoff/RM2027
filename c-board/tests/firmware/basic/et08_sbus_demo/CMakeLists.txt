# =============================================================================
# @file tests/firmware/basic/et08_sbus_demo/CMakeLists.txt
# @brief ET08 SBUS telemetry demo firmware CMake config
# @project RM2026
# @author Codex
# @version 1.0.0
# @date 2026-07-19
# @details Builds a simple firmware that dumps SBUS channel data via USART6 for
#          bring-up and mapping verification.
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(test_et08_sbus_demo C ASM)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

create_embedded_test(et08_sbus_demo
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
