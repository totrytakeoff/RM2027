# =============================================================================
# @file tests/firmware/basic/omni_chassis_demo/CMakeLists.txt
# @brief 十字全向轮底盘遥控控制固件 CMake 配置
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 构建一个集成遥控接收和电机控制的固件，实现十字全向轮底盘控制
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(omni_chassis_demo C ASM)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

create_embedded_test(omni_chassis_demo
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
