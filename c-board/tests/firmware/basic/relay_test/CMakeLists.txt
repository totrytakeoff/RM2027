# =============================================================================
# @file tests/firmware/basic/relay_test/CMakeLists.txt
# @brief 继电器测试固件的 CMake 配置
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 通过 create_embedded_test() 生成 test_relay_test 目标，并继承框架库与 HAL 依赖。
# =============================================================================

project(relay_test)

# Include the cmake file that defines our test creation function
include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

# This test only needs its own source files.
# All other dependencies are handled by linking against the framework libraries.
set(SOURCES main.c)

# Create the test executable by calling the correct function.
# It links the explicit component, platform, vendor, and board targets below.
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
