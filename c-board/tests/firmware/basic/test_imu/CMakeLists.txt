# =============================================================================
# @file tests/firmware/basic/test_imu/CMakeLists.txt
# @brief IMU 测试固件的 CMake 配置
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 定义 test_imu 工程并调用 create_embedded_test()，链接框架与 HAL 以输出可下载的固件。
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(test_imu C ASM)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

create_embedded_test(imu
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
