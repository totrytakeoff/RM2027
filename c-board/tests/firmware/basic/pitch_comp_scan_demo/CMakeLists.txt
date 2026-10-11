# =============================================================================
# @file tests/firmware/basic/pitch_comp_scan_demo/CMakeLists.txt
# @brief Pitch补偿扫描 Demo
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(pitch_comp_scan_demo C ASM)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

create_embedded_test(pitch_comp_scan_demo
    SOURCES
        main.c
    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/tests/firmware/infantry/infantry_minimal
    LINK_LIBRARIES
        rm_robot_config
        rm_components
        rm_platform_stm32f4
        rm_vendor_runtime
        rm_common
        rm_vendor_stm32_hal
    LINKER_SCRIPT ${MCU_LINKER_SCRIPT}
)
