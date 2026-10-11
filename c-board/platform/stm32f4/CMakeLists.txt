# STM32F4 board-support adapters.  Sources are explicit so adding a file does
# not silently add it to every firmware image.
add_library(rm_platform_stm32f4 STATIC
    adc/bsp_adc.c
    bsp_tools.c
    can/bsp_can.c
    dwt/bsp_dwt.c
    flash/bsp_flash.c
    gpio/bsp_gpio.c
    iic/bsp_iic.c
    log/bsp_log.c
    pwm/bsp_pwm.c
    spi/bsp_spi.c
    time/rm_time.c
    usart/bsp_usart.c
    usb/bsp_usb.c
    utils/utils.c
    watchdog/rm_watchdog.c
)
add_library(RM::platform_stm32f4 ALIAS rm_platform_stm32f4)

# Directly linked into every embedded executable so newlib never falls back to
# libnosys warning stubs.  Keep this separate from the platform archive: libc
# can introduce the syscall references after ordinary archive scanning.
add_library(rm_platform_syscalls OBJECT
    runtime/rm_syscalls.c
)
target_link_libraries(rm_platform_syscalls PRIVATE
    rm_vendor_segger_rtt
    rm_vendor_stm32_hal
)
target_compile_options(rm_platform_syscalls PRIVATE ${MCU_FLAGS})

target_include_directories(rm_platform_stm32f4 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/adc
    ${CMAKE_CURRENT_SOURCE_DIR}/can
    ${CMAKE_CURRENT_SOURCE_DIR}/critical
    ${CMAKE_CURRENT_SOURCE_DIR}/dwt
    ${CMAKE_CURRENT_SOURCE_DIR}/flash
    ${CMAKE_CURRENT_SOURCE_DIR}/gpio
    ${CMAKE_CURRENT_SOURCE_DIR}/iic
    ${CMAKE_CURRENT_SOURCE_DIR}/log
    ${CMAKE_CURRENT_SOURCE_DIR}/pwm
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime
    ${CMAKE_CURRENT_SOURCE_DIR}/spi
    ${CMAKE_CURRENT_SOURCE_DIR}/time
    ${CMAKE_CURRENT_SOURCE_DIR}/usart
    ${CMAKE_CURRENT_SOURCE_DIR}/usb
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/watchdog
)

target_link_libraries(rm_platform_stm32f4 PUBLIC
    rm_board_rm_f407_headers
    rm_platform_core
    rm_vendor_runtime
    rm_vendor_stm32_hal
)
target_compile_options(rm_platform_stm32f4 PRIVATE ${MCU_FLAGS})
