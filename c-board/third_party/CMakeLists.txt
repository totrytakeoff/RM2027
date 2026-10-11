# External dependencies used by the STM32F407 firmware.  Keep repository-owned
# sources out of this layer so vendor upgrades remain reviewable.

add_library(rm_vendor_cmsis INTERFACE)
add_library(RM::vendor_cmsis ALIAS rm_vendor_cmsis)
target_include_directories(rm_vendor_cmsis INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis/Device/ST/STM32F4xx/Include
)

add_library(rm_vendor_cmsis_dsp INTERFACE)
add_library(RM::vendor_cmsis_dsp ALIAS rm_vendor_cmsis_dsp)
target_include_directories(rm_vendor_cmsis_dsp INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis_dsp/Include
)
target_link_libraries(rm_vendor_cmsis_dsp INTERFACE
    rm_vendor_cmsis
    ${DSP_LIB}
)

add_library(rm_vendor_stm32_hal STATIC
    stm32f4xx_hal/Src/stm32f4xx_hal.c
    stm32f4xx_hal/Src/stm32f4xx_hal_adc.c
    stm32f4xx_hal/Src/stm32f4xx_hal_adc_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_can.c
    stm32f4xx_hal/Src/stm32f4xx_hal_cortex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_crc.c
    stm32f4xx_hal/Src/stm32f4xx_hal_dac.c
    stm32f4xx_hal/Src/stm32f4xx_hal_dac_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_dma.c
    stm32f4xx_hal/Src/stm32f4xx_hal_dma_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_exti.c
    stm32f4xx_hal/Src/stm32f4xx_hal_flash.c
    stm32f4xx_hal/Src/stm32f4xx_hal_flash_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_flash_ramfunc.c
    stm32f4xx_hal/Src/stm32f4xx_hal_gpio.c
    stm32f4xx_hal/Src/stm32f4xx_hal_i2c.c
    stm32f4xx_hal/Src/stm32f4xx_hal_i2c_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_pcd.c
    stm32f4xx_hal/Src/stm32f4xx_hal_pcd_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_pwr.c
    stm32f4xx_hal/Src/stm32f4xx_hal_pwr_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_rcc.c
    stm32f4xx_hal/Src/stm32f4xx_hal_rcc_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_rng.c
    stm32f4xx_hal/Src/stm32f4xx_hal_rtc.c
    stm32f4xx_hal/Src/stm32f4xx_hal_rtc_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_spi.c
    stm32f4xx_hal/Src/stm32f4xx_hal_tim.c
    stm32f4xx_hal/Src/stm32f4xx_hal_tim_ex.c
    stm32f4xx_hal/Src/stm32f4xx_hal_uart.c
    stm32f4xx_hal/Src/stm32f4xx_ll_adc.c
    stm32f4xx_hal/Src/stm32f4xx_ll_usb.c
)
add_library(RM::vendor_stm32_hal ALIAS rm_vendor_stm32_hal)
target_include_directories(rm_vendor_stm32_hal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_hal/Inc
    ${RM_BOARD_INCLUDE_DIR}
)
target_link_libraries(rm_vendor_stm32_hal PUBLIC rm_vendor_cmsis)
target_compile_options(rm_vendor_stm32_hal PRIVATE ${MCU_FLAGS})

# Dynamic allocation is disabled in the competition FreeRTOS configuration;
# heap_4.c is deliberately absent from this explicit source list.
add_library(rm_vendor_freertos STATIC
    freertos_kernel/Source/CMSIS_RTOS/cmsis_os.c
    freertos_kernel/Source/croutine.c
    freertos_kernel/Source/event_groups.c
    freertos_kernel/Source/list.c
    freertos_kernel/Source/portable/GCC/ARM_CM4F/port.c
    freertos_kernel/Source/queue.c
    freertos_kernel/Source/stream_buffer.c
    freertos_kernel/Source/tasks.c
    freertos_kernel/Source/timers.c
)
add_library(RM::vendor_freertos ALIAS rm_vendor_freertos)
target_include_directories(rm_vendor_freertos PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/freertos_kernel/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/freertos_kernel/Source/CMSIS_RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/freertos_kernel/Source/portable/GCC/ARM_CM4F
    ${RM_BOARD_INCLUDE_DIR}
)
target_link_libraries(rm_vendor_freertos PUBLIC rm_vendor_cmsis)
target_compile_options(rm_vendor_freertos PRIVATE ${MCU_FLAGS})

add_library(rm_vendor_usb_device STATIC
    stm32_usb_device/Class/CDC/Src/usbd_cdc.c
    stm32_usb_device/Core/Src/usbd_core.c
    stm32_usb_device/Core/Src/usbd_ctlreq.c
    stm32_usb_device/Core/Src/usbd_ioreq.c
)
add_library(RM::vendor_usb_device ALIAS rm_vendor_usb_device)
target_include_directories(rm_vendor_usb_device PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32_usb_device/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32_usb_device/Class/CDC/Inc
    ${RM_BOARD_INCLUDE_DIR}
)
target_link_libraries(rm_vendor_usb_device PUBLIC rm_vendor_stm32_hal)
target_compile_options(rm_vendor_usb_device PRIVATE ${MCU_FLAGS})

add_library(rm_vendor_segger_rtt STATIC
    segger_rtt/RTT/SEGGER_RTT.c
    segger_rtt/RTT/SEGGER_RTT_printf.c
)
add_library(RM::vendor_segger_rtt ALIAS rm_vendor_segger_rtt)
target_include_directories(rm_vendor_segger_rtt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/segger_rtt/RTT
    ${CMAKE_CURRENT_SOURCE_DIR}/segger_rtt/Config
)
target_compile_options(rm_vendor_segger_rtt PRIVATE ${MCU_FLAGS})

add_library(rm_vendor_runtime INTERFACE)
add_library(RM::vendor_runtime ALIAS rm_vendor_runtime)
target_link_libraries(rm_vendor_runtime INTERFACE
    rm_vendor_freertos
    rm_vendor_usb_device
    rm_vendor_segger_rtt
)
