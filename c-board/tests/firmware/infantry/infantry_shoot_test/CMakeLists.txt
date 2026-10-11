# =============================================================================
# @file tests/firmware/infantry/infantry_shoot_test/CMakeLists.txt
# @brief Infantry gimbal + shooter integration test firmware CMake config
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 构建步兵云台 + 发射机构联测固件（ET08 遥控）
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(infantry_shoot_test C ASM)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

create_embedded_test(infantry_shoot_test
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
