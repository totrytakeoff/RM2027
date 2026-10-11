# =============================================================================
# @file tests/firmware/basic/shoot_test/CMakeLists.txt
# @brief Shooter test firmware CMake config
# @project RM2026
# @author Codex
# @version 1.0.0
# @date 2026-07-19
# @details Builds a standalone shooter test that drives 3x M3508 (2 friction + 1 loader).
# =============================================================================

project(shoot_test)

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
