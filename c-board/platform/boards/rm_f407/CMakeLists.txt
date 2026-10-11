# =============================================================================
# @file platform/boards/rm_f407/CMakeLists.txt
# @brief RM F407 板级生成代码与启动配置
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 板级 target 统一管理 CubeMX 生成源、启动文件、中断与公共头文件。
# =============================================================================

# CubeMX 生成源码显式登记；中断处理文件单独作为 OBJECT target。
set(RM_BOARD_GENERATED_SOURCES
    generated/adc.c
    generated/can.c
    generated/crc.c
    generated/dac.c
    generated/dma.c
    generated/freertos.c
    generated/gpio.c
    generated/i2c.c
    generated/rng.c
    generated/rtc.c
    generated/spi.c
    generated/stm32f4xx_hal_msp.c
    generated/stm32f4xx_hal_timebase_tim.c
    generated/system_stm32f4xx.c
    generated/tim.c
    generated/usart.c
    generated/usb_device.c
    generated/usbd_cdc_if.c
    generated/usbd_conf.c
    generated/usbd_desc.c
)

set(RM_BOARD_STARTUP_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/startup/startup_stm32f407xx.s")
list(APPEND RM_BOARD_GENERATED_SOURCES ${RM_BOARD_STARTUP_SOURCE})

set_source_files_properties(${RM_BOARD_STARTUP_SOURCE}
    PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

add_library(rm_board_rm_f407_headers INTERFACE)
add_library(RM::board_rm_f407_headers ALIAS rm_board_rm_f407_headers)
target_include_directories(rm_board_rm_f407_headers INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(rm_board_rm_f407 STATIC ${RM_BOARD_GENERATED_SOURCES})
add_library(RM::board_rm_f407 ALIAS rm_board_rm_f407)

# 中断处理文件单独建 OBJECT 库，避免被归档优化掉
add_library(rm_board_rm_f407_irq OBJECT
    generated/stm32f4xx_it.c
    fault/rm_fault.c
)
target_include_directories(rm_board_rm_f407_irq PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/fault
)
target_link_libraries(rm_board_rm_f407_irq PUBLIC
    rm_board_rm_f407_headers
    rm_vendor_stm32_hal
)
target_compile_options(rm_board_rm_f407_irq PRIVATE ${MCU_FLAGS})

target_include_directories(rm_board_rm_f407 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/generated
    # usbd_conf.c currently uses the platform monotonic timer during USB init.
    ${CMAKE_SOURCE_DIR}/platform/stm32f4/dwt
    ${CMAKE_SOURCE_DIR}/platform/stm32f4/log
)

target_compile_options(rm_board_rm_f407 PRIVATE ${MCU_FLAGS})

target_link_libraries(rm_board_rm_f407 PUBLIC
    rm_board_rm_f407_headers
    rm_vendor_runtime
    rm_vendor_stm32_hal
)

message(STATUS "Board rm_f407 configured.")
