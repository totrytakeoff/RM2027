# =============================================================================
# @file CMakeLists.txt
# @brief RM2026 顶层 CMake 配置入口
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 统一加载工具链，按外部依赖、平台、组件、应用和固件分层组织构建。
# =============================================================================

cmake_minimum_required(VERSION 3.16)

set(RM_BOARD_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/platform/boards/rm_f407"
    CACHE INTERNAL "Active board ownership root")
set(RM_BOARD_INCLUDE_DIR
    "${RM_BOARD_ROOT}/include"
    CACHE INTERNAL "Active board-generated include directory")

# =============================================================================
# 项目基本设置（简化入口，只保留全局配置和子目录组织）
# =============================================================================
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/Toolchain.cmake" CACHE FILEPATH "Toolchain file")
endif()

project(RM2026 C CXX ASM)

# =============================================================================
# 全局日志默认配置（默认 RTT，不占用业务串口）
# 可通过目标级/命令行 -D 覆盖
# =============================================================================
add_compile_definitions(
    BSP_LOG_USE_UART=0
    BSP_LOG_UART_PORT=1
    BSP_LOG_UART_TIMEOUT_MS=100
)

include(CTest)
enable_testing()

# 统一的产物收集目录（供烧录脚本/CI使用）
set(GLOBAL_OUTPUT_DIR ${CMAKE_BINARY_DIR}/output CACHE INTERNAL "Directory that collects flashable artifacts")
file(MAKE_DIRECTORY ${GLOBAL_OUTPUT_DIR})

# =============================================================================
# 添加子模块 (顺序很重要)
# =============================================================================
# 1. 外部依赖必须先于所有仓库自有代码定义。
add_subdirectory(third_party)

# 2. 板级配置、平台适配与可复用组件。
add_subdirectory(platform)
add_subdirectory(components)

# 3. 应用行为与具体固件组合层。
add_subdirectory(applications)
add_subdirectory(firmware)

# 4. 板级与回归测试目标。
# 定义所有测试项目的输出根目录, 以便 TestFunctions.cmake 使用
set(TEST_OUTPUT_DIR ${CMAKE_BINARY_DIR}/tests CACHE INTERNAL "Root directory for test outputs")
add_subdirectory(tests)

# =============================================================================
# 自定义全局命令 (Clean / Upload)
# =============================================================================
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}
    COMMENT "Removing build directory..."
)

# OpenOCD 下载命令。BIN 不携地址，因此必须显式指定 Flash 基地址。
set(OPENOCD_CFG "${RM_BOARD_ROOT}/openocd/openocd_dap.cfg")

# 优先使用当前 PATH 中维护的新版本；PlatformIO 副本仅作为最终回退。
find_program(OPENOCD_EXECUTABLE NAMES openocd)
if(NOT OPENOCD_EXECUTABLE)
    find_program(OPENOCD_EXECUTABLE
        NAMES openocd
        PATHS /home/myself/.platformio/packages/tool-openocd/bin
        NO_DEFAULT_PATH
    )
endif()

# 设置OpenOCD路径变量
if(OPENOCD_EXECUTABLE)
    set(OPENOCD_PATH ${OPENOCD_EXECUTABLE})
    message(STATUS "Found OpenOCD: ${OPENOCD_PATH}")
else()
    # 设置默认路径作为备用
    set(OPENOCD_PATH "/home/myself/.platformio/packages/tool-openocd/bin/openocd")
    message(WARNING "OpenOCD not found, using default path: ${OPENOCD_PATH}")
endif()

add_custom_target(upload
    COMMAND ${CMAKE_COMMAND} -E echo "[UPLOAD] Starting upload to STM32..."
    COMMAND ${OPENOCD_PATH} -f ${OPENOCD_CFG}
            -c "init"
            -c "reset halt"
            -c "flash write_image erase ${GLOBAL_OUTPUT_DIR}/app.bin 0x08000000 bin"
            -c "verify_image ${GLOBAL_OUTPUT_DIR}/app.bin 0x08000000 bin"
            -c "reset run"
            -c "shutdown"
    COMMAND ${CMAKE_COMMAND} -E echo "[SUCCESS] Upload completed successfully!"
    DEPENDS app.elf
    COMMENT "Uploading to STM32..."
)

# 添加一个更详细的上传命令，显示更多调试信息
add_custom_target(upload-verbose
    COMMAND ${CMAKE_COMMAND} -E echo "[UPLOAD-VERBOSE] Starting verbose upload to STM32..."
    COMMAND ${OPENOCD_PATH} -f ${OPENOCD_CFG}
            -c "init"
            -c "reset halt"
            -c "flash write_image erase ${GLOBAL_OUTPUT_DIR}/app.bin 0x08000000 bin"
            -c "verify_image ${GLOBAL_OUTPUT_DIR}/app.bin 0x08000000 bin"
            -c "reset run"
            -c "shutdown"
    COMMAND ${CMAKE_COMMAND} -E echo "[SUCCESS] Upload completed successfully!"
    DEPENDS app.elf
    COMMENT "Verbose upload to STM32..."
)

# 添加一个仅验证命令，不重新烧录
add_custom_target(verify
    COMMAND ${CMAKE_COMMAND} -E echo "[VERIFY] Verifying STM32 flash..."
    COMMAND ${OPENOCD_PATH} -f ${OPENOCD_CFG}
            -c "init"
            -c "reset halt"
            -c "verify_image ${GLOBAL_OUTPUT_DIR}/app.bin 0x08000000 bin"
            -c "reset run"
            -c "shutdown"
    COMMAND ${CMAKE_COMMAND} -E echo "[SUCCESS] Verification completed!"
    DEPENDS app.elf
    COMMENT "Verifying STM32 flash..."
)
