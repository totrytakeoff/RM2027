# =============================================================================
# @file tests/firmware/infantry/infantry_chassis_gimbal_et08_demo/CMakeLists.txt
# @brief Infantry ET08 底盘云台融合测试 CMake 配置
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(infantry_chassis_gimbal_et08_demo C ASM)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

create_embedded_test(infantry_chassis_gimbal_et08_demo
    SOURCES
        main.c
    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/tests/firmware/infantry/infantry_minimal
    LINK_LIBRARIES
        rm_robot_config
        rm_components
        rm_platform_stm32f4
        rm_common
        rm_vendor_stm32_hal
    LINKER_SCRIPT ${MCU_LINKER_SCRIPT}
)
