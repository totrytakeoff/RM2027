# =============================================================================
# @file tests/firmware/leg/wheelleg_joint_pos_test/CMakeLists.txt
# @brief Wheel-leg joint position observe demo (DM8009P)
# =============================================================================

project(wheelleg_joint_pos_test)

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
