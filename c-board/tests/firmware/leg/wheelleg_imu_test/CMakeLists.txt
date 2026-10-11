# =============================================================================
# @file tests/firmware/leg/wheelleg_imu_test/CMakeLists.txt
# @brief Wheel-leg IMU basic test (DM-IMU over CAN2)
# =============================================================================

project(wheelleg_imu_test)

include(${CMAKE_SOURCE_DIR}/cmake/TestFunctions.cmake)

set(SOURCES main.c)

create_embedded_test(${PROJECT_NAME}
    SOURCES ${SOURCES}
    INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
    LINK_LIBRARIES
        rm_device_dm_imu
        rm_components
        rm_platform_stm32f4
        rm_vendor_runtime
        rm_common
        rm_vendor_stm32_hal
)
