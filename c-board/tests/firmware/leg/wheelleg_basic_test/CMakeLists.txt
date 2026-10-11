# =============================================================================
# @file tests/firmware/leg/wheelleg_basic_test/CMakeLists.txt
# @brief 轮腿底盘基础模块测试（IMU + 轮毂 + 关节）
# =============================================================================

project(wheelleg_basic_test)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

set(SOURCES main.c)

create_embedded_test(${PROJECT_NAME}
    SOURCES ${SOURCES}
    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
    LINK_LIBRARIES
        rm_components
        rm_platform_stm32f4
        rm_vendor_runtime
        rm_common
        rm_vendor_stm32_hal
)
