# =============================================================================
# @file tests/firmware/basic/vt_link_monitor_demo/CMakeLists.txt
# @brief VT link monitor demo firmware CMake config
# @project RM2026
# @author Codex
# @version 1.0.0
# @date 2026-07-19
# @details Builds a simple firmware that receives VT frames on USART6 and dumps
#          parsed fields via USART1 in vt_serial_monitor-compatible format.
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(vt_link_monitor_demo C ASM)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

create_embedded_test(vt_link_monitor_demo
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
