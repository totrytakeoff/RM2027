# =============================================================================
# @file tests/firmware/basic/test_dm_imu/CMakeLists.txt
# @brief DM-IMU 测试固件的 CMake 配置
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(test_dm_imu C ASM)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

create_embedded_test(dm_imu
    SOURCES
        main.c
    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
    LINK_LIBRARIES
        rm_device_dm_imu
        rm_components
        rm_platform_stm32f4
        rm_vendor_runtime
        rm_common
        rm_vendor_stm32_hal
    LINKER_SCRIPT ${MCU_LINKER_SCRIPT}
)
