add_subdirectory(freertos)

add_library(rm_robot_firmware STATIC
    robot_firmware.c
)
add_library(RM::robot_firmware ALIAS rm_robot_firmware)

target_include_directories(rm_robot_firmware PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(rm_robot_firmware PRIVATE ${MCU_FLAGS})

target_link_libraries(rm_robot_firmware PUBLIC
    rm_robot_freertos
    rm_robot_app
    rm_platform_stm32f4
)

# -----------------------------------------------------------------------------
# Formal robot firmware image
# -----------------------------------------------------------------------------
if(NOT DEFINED LINKER_SCRIPT)
    set(LINKER_SCRIPT "${MCU_LINKER_SCRIPT}")
endif()

if(NOT DSP_LIB OR NOT EXISTS ${DSP_LIB})
    message(FATAL_ERROR "DSP library not found: ${DSP_LIB}")
endif()

if(NOT DEFINED GLOBAL_OUTPUT_DIR)
    set(GLOBAL_OUTPUT_DIR ${CMAKE_BINARY_DIR}/output)
endif()

add_executable(app.elf main.c)
set_property(TARGET app.elf APPEND PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})
# 正式固件包含 C++ 遥控适配层，显式使用 C++ 驱动链接，避免依赖
# CMake 对静态库传递语言的隐式推断。
set_property(TARGET app.elf PROPERTY LINKER_LANGUAGE CXX)

target_link_libraries(app.elf PRIVATE
    rm_platform_syscalls
    -Wl,--start-group
    rm_robot_firmware
    rm_robot_freertos
    rm_robot_app
    rm_components_runtime
    rm_platform_stm32f4
    rm_vendor_freertos
    rm_vendor_usb_device
    rm_vendor_segger_rtt
    rm_common
    rm_vendor_stm32_hal
    rm_board_rm_f407
    $<TARGET_OBJECTS:rm_board_rm_f407_irq>
    -Wl,--end-group
    m
    c
)

target_compile_options(app.elf PRIVATE ${MCU_FLAGS})
target_link_options(app.elf PRIVATE
    ${GENERIC_LINK_OPTIONS}
    -T${LINKER_SCRIPT}
    -Wl,--defsym=RM_MIN_HEAP_SIZE=0
    -Wl,--defsym=RM_MIN_STACK_SIZE=0x2000
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/app.map
    -Wl,--print-memory-usage
)

add_custom_command(TARGET app.elf POST_BUILD
    COMMAND ${CMAKE_COMMAND}
            -DNM=${CMAKE_NM}
            -DELF=$<TARGET_FILE:app.elf>
            -P ${CMAKE_SOURCE_DIR}/cmake/CheckFormalFirmwareSymbols.cmake
    COMMAND ${CMAKE_COMMAND}
            -DREADELF=${CMAKE_READELF}
            -DELF=$<TARGET_FILE:app.elf>
            -P ${CMAKE_SOURCE_DIR}/cmake/CheckElfSegments.cmake
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:app.elf> ${CMAKE_CURRENT_BINARY_DIR}/app.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:app.elf> ${CMAKE_CURRENT_BINARY_DIR}/app.bin
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GLOBAL_OUTPUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:app.elf> ${GLOBAL_OUTPUT_DIR}/app.elf
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/app.hex ${GLOBAL_OUTPUT_DIR}/app.hex
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/app.bin ${GLOBAL_OUTPUT_DIR}/app.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:app.elf>
    COMMENT "Generating formal firmware ELF / HEX / BIN artifacts"
)
