add_library(rm_cmsis INTERFACE)
target_include_directories(rm_cmsis INTERFACE cmsis/Include cmsis/Device/ST/STM32H7xx/Include)
target_compile_definitions(rm_cmsis INTERFACE STM32H723xx USE_HAL_DRIVER USE_PWR_LDO_SUPPLY)
target_link_libraries(rm_cmsis INTERFACE rm_cpu)
add_library(rm_hal STATIC
    stm32h7xx_hal/Src/stm32h7xx_hal.c
    stm32h7xx_hal/Src/stm32h7xx_hal_cortex.c
    stm32h7xx_hal/Src/stm32h7xx_hal_dma.c
    stm32h7xx_hal/Src/stm32h7xx_hal_dma_ex.c
    stm32h7xx_hal/Src/stm32h7xx_hal_exti.c
    stm32h7xx_hal/Src/stm32h7xx_hal_fdcan.c
    stm32h7xx_hal/Src/stm32h7xx_hal_flash.c
    stm32h7xx_hal/Src/stm32h7xx_hal_flash_ex.c
    stm32h7xx_hal/Src/stm32h7xx_hal_gpio.c
    stm32h7xx_hal/Src/stm32h7xx_hal_iwdg.c
    stm32h7xx_hal/Src/stm32h7xx_hal_mdma.c
    stm32h7xx_hal/Src/stm32h7xx_hal_pwr.c
    stm32h7xx_hal/Src/stm32h7xx_hal_pwr_ex.c
    stm32h7xx_hal/Src/stm32h7xx_hal_rcc.c
    stm32h7xx_hal/Src/stm32h7xx_hal_rcc_ex.c
    stm32h7xx_hal/Src/stm32h7xx_hal_spi.c
    stm32h7xx_hal/Src/stm32h7xx_hal_spi_ex.c
    stm32h7xx_hal/Src/stm32h7xx_hal_tim.c
    stm32h7xx_hal/Src/stm32h7xx_hal_tim_ex.c
    stm32h7xx_hal/Src/stm32h7xx_hal_uart.c
    stm32h7xx_hal/Src/stm32h7xx_hal_uart_ex.c)
target_include_directories(rm_hal PUBLIC stm32h7xx_hal/Inc "${RM_BOARD_INCLUDE}")
target_link_libraries(rm_hal PUBLIC rm_cmsis)
if(NOT RM_FREERTOS_PORT MATCHES "^(ARM_CM7/r0p1|ARM_CM4F)$")
    message(FATAL_ERROR "Unsupported FreeRTOS GCC port")
endif()
add_library(rm_freertos STATIC freertos/tasks.c freertos/list.c freertos/queue.c
    freertos/portable/GCC/${RM_FREERTOS_PORT}/port.c)
target_include_directories(rm_freertos PUBLIC freertos/include
    freertos/portable/GCC/${RM_FREERTOS_PORT} "${RM_BOARD_INCLUDE}")
target_link_libraries(rm_freertos PUBLIC rm_cmsis rm_platform_api)
