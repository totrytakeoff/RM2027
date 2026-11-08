cmake_minimum_required(VERSION 3.21)
project(RM2027_MC02 LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
option(RM_BUILD_FIRMWARE "Build MC-02 ARM firmware" ON)
option(RM_ENABLE_DCACHE "Enable D-Cache with non-cacheable DMA regions" ON)
option(RM_BUILD_TESTS "Build host component tests" OFF)
option(RM_CHASSIS_ENABLE_MOTION "Allow motor drive output in the dedicated chassis demo" ON)
option(RM_MECANUM_ENABLE_MOTION "Allow motor drive in the mecanum demo" ON)
option(RM_MOTOR_TEST_ENABLE_MOTION "Allow motor drive in the motor test" ON)
set(RM_REMOTE_PROTOCOL "SBUS" CACHE STRING "Bringup UART5 input protocol: SBUS or DBUS")
set_property(CACHE RM_REMOTE_PROTOCOL PROPERTY STRINGS SBUS DBUS)
if(NOT RM_REMOTE_PROTOCOL MATCHES "^(SBUS|DBUS)$")
    message(FATAL_ERROR "RM_REMOTE_PROTOCOL must be SBUS or DBUS")
endif()
set(RM_FREERTOS_PORT "ARM_CM7/r0p1" CACHE STRING "FreeRTOS GCC port, defaults to universal M7 workaround")
set_property(CACHE RM_FREERTOS_PORT PROPERTY STRINGS "ARM_CM7/r0p1" "ARM_CM4F")
add_library(rm_platform_api INTERFACE)
target_include_directories(rm_platform_api INTERFACE platform/api)
add_library(rm_matrix STATIC
    third_party/cmsis_dsp_matrix/arm_mat_init_f32.c
    third_party/cmsis_dsp_matrix/arm_mat_add_f32.c
    third_party/cmsis_dsp_matrix/arm_mat_sub_f32.c
    third_party/cmsis_dsp_matrix/arm_mat_mult_f32.c
    third_party/cmsis_dsp_matrix/arm_mat_trans_f32.c
    third_party/cmsis_dsp_matrix/arm_mat_inverse_f32.c)
target_include_directories(rm_matrix PUBLIC components/algorithms/matrix)
target_compile_definitions(rm_matrix PRIVATE ARM_MATH_MATRIX_CHECK ARM_MATH_LOOPUNROLL)
add_library(rm_logic STATIC
    components/algorithms/pid/controller.c
    components/services/safety/safety_manager.c
    components/devices/vision/vision_protocol.c
    components/devices/vision/vision_link_state.c
    components/devices/imu/bmi088/bmi088.c
    components/algorithms/filters/kalman/kalman_filter.c
    components/algorithms/quaternion_ekf/QuaternionEKF.c
    components/services/ins/ins.c
    components/devices/remote/remote.c
    components/devices/motor/dji/dji.c
    components/devices/motor/dm/dm.c)
target_include_directories(rm_logic PUBLIC
    components/algorithms/pid
    components/services/safety
    components/devices/vision components/devices/imu/bmi088
    components/algorithms/filters/kalman components/algorithms/quaternion_ekf
    components/services/ins components/devices/remote components/devices/motor/dji components/devices/motor/dm)
target_link_libraries(rm_logic PUBLIC rm_platform_api rm_matrix)
target_compile_options(rm_logic PRIVATE -Wall -Wextra -Wpedantic)
add_library(rm_chassis_logic STATIC
    applications/chassis_demo/chassis_control.c
    applications/chassis_demo/chassis_kinematics.c
    applications/chassis_demo/chassis_follow.c)
target_include_directories(rm_chassis_logic PUBLIC applications/chassis_demo config/robots/chassis_demo)
target_link_libraries(rm_chassis_logic PUBLIC rm_logic)
target_compile_options(rm_chassis_logic PRIVATE -Wall -Wextra -Wpedantic)
add_library(rm_mecanum_logic STATIC
    applications/mecanum_demo/mecanum_control.c
    applications/mecanum_demo/mecanum_kinematics.c
    applications/mecanum_demo/mecanum_follow.c)
target_include_directories(rm_mecanum_logic PUBLIC applications/mecanum_demo config/robots/mecanum_demo)
target_link_libraries(rm_mecanum_logic PUBLIC rm_logic)
target_compile_options(rm_mecanum_logic PRIVATE -Wall -Wextra -Wpedantic)
if(NOT RM_BUILD_FIRMWARE)
    if(RM_BUILD_TESTS)
        enable_testing()
        add_executable(chassis_demo_test tests/host/chassis_demo_test.c config/robots/chassis_demo/binding.c)
        target_include_directories(chassis_demo_test PRIVATE config/robots/bringup)
        target_link_libraries(chassis_demo_test PRIVATE rm_chassis_logic m)
        add_test(NAME chassis_demo_control COMMAND chassis_demo_test)
        add_executable(motor_test_host tests/host/motor_test.c
            applications/motor_test/motor_test_control.c config/robots/motor_test/binding.c)
        target_compile_definitions(motor_test_host PRIVATE MOTOR_TEST_MODEL=3508)
        target_include_directories(motor_test_host PRIVATE applications/motor_test config/robots/motor_test config/robots/bringup)
        target_link_libraries(motor_test_host PRIVATE rm_chassis_logic m)
        add_test(NAME motor_test_modes COMMAND motor_test_host)
        add_executable(dm_protocol_test tests/host/dm_protocol_test.c)
        target_link_libraries(dm_protocol_test PRIVATE rm_logic m)
        add_test(NAME dm_protocol COMMAND dm_protocol_test)
        add_executable(motor_dm_backend_test tests/host/motor_dm_backend_test.c
            applications/motor_test/motor_test_dm.c components/services/safety/output_gate.c)
        target_compile_definitions(motor_dm_backend_test PRIVATE MOTOR_TEST_MODEL=4310 RM_ENABLE_MOTION=1)
        target_include_directories(motor_dm_backend_test PRIVATE applications/motor_test applications/board_bringup
            config/robots/motor_test runtime/freertos config/profiles)
        target_link_libraries(motor_dm_backend_test PRIVATE rm_logic m)
        add_test(NAME motor_dm_backend COMMAND motor_dm_backend_test)
    endif()
    message(STATUS "Host build: platform-independent components and scalar matrix library")
    return()
endif()
if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    message(FATAL_ERROR "Use h7-debug/h7-release presets or cmake/arm-gcc.cmake for firmware")
endif()
add_library(rm_cpu INTERFACE)
target_compile_definitions(rm_cpu INTERFACE $<$<CONFIG:Debug>:RM_DEBUG=1>)
target_compile_definitions(rm_cpu INTERFACE RM_UART5_DBUS=$<STREQUAL:${RM_REMOTE_PROTOCOL},DBUS>)
target_compile_options(rm_cpu INTERFACE -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
    -ffunction-sections -fdata-sections -fno-common
    $<$<CONFIG:Debug>:-Og> $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Release>:-O2> $<$<CONFIG:Release>:-g>) # Optimized image retains offline DWARF for live SWD telemetry.
target_link_options(rm_cpu INTERFACE -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)
target_link_libraries(rm_logic PRIVATE rm_cpu)
target_link_libraries(rm_matrix PRIVATE rm_cpu)
target_link_libraries(rm_chassis_logic PRIVATE rm_cpu)
target_link_libraries(rm_mecanum_logic PRIVATE rm_cpu)
set(RM_BOARD_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/platform/boards/dm_mc02/include")
add_subdirectory(third_party)
add_library(rm_board OBJECT
    platform/boards/dm_mc02/startup/startup_stm32h723vgtx.s
    platform/boards/dm_mc02/generated/system_stm32h7xx.c
    platform/boards/dm_mc02/generated/stm32h7xx_hal_msp.c
    platform/boards/dm_mc02/generated/gpio.c
    platform/boards/dm_mc02/generated/spi.c
    platform/boards/dm_mc02/generated/usart.c
    platform/boards/dm_mc02/generated/fdcan.c
    platform/boards/dm_mc02/board.c
    platform/boards/dm_mc02/irq/interrupts.c
    platform/boards/dm_mc02/fault/fault.c)
target_include_directories(rm_board PRIVATE platform/stm32h7 config/boards/dm_mc02 config/profiles)
target_link_libraries(rm_board PRIVATE rm_hal rm_freertos rm_platform_api)
target_compile_options(rm_board PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic>)
add_library(rm_platform OBJECT
    platform/stm32h7/memory.c
    platform/stm32h7/time.c
    platform/stm32h7/critical.c
    platform/stm32h7/serial.c
    platform/stm32h7/can.c
    platform/stm32h7/sensor_bus.c
    platform/stm32h7/watchdog.c)
target_include_directories(rm_platform PRIVATE platform/stm32h7 config/boards/dm_mc02 config/profiles)
target_compile_definitions(rm_platform PRIVATE RM_ENABLE_DCACHE=$<BOOL:${RM_ENABLE_DCACHE}>)
target_compile_options(rm_platform PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(rm_platform PRIVATE rm_platform_api rm_hal rm_logic)
add_library(rm_runtime OBJECT runtime/freertos/runtime.c runtime/freertos/hooks.c)
target_include_directories(rm_runtime PRIVATE runtime/freertos config/profiles)
target_compile_options(rm_runtime PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(rm_runtime PRIVATE rm_freertos rm_platform_api)
add_library(rm_services STATIC
    components/services/safety/output_gate.c
    components/services/logging/logging.c)
target_include_directories(rm_services PUBLIC
    components/services/safety components/services/logging components/devices/imu/bmi088)
target_compile_options(rm_services PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(rm_services PUBLIC rm_platform_api PRIVATE rm_cpu)
set(RM_LINKER "${CMAKE_CURRENT_SOURCE_DIR}/platform/boards/dm_mc02/linker/STM32H723VG.ld")
add_executable(board_bringup.elf firmware/dm_mc02/main.c firmware/dm_mc02/syscalls.c applications/board_bringup/bringup.c
    config/robots/bringup/binding.c
    $<TARGET_OBJECTS:rm_board> $<TARGET_OBJECTS:rm_platform> $<TARGET_OBJECTS:rm_runtime>)
target_include_directories(board_bringup.elf PRIVATE runtime/freertos applications/board_bringup config/profiles config/robots/bringup)
target_link_libraries(board_bringup.elf PRIVATE rm_services rm_logic rm_freertos rm_hal rm_platform_api m)
target_compile_options(board_bringup.elf PRIVATE -Wall -Wextra -Wpedantic)
target_link_options(board_bringup.elf PRIVATE --specs=nano.specs --specs=nosys.specs
    -nostartfiles "-T${RM_LINKER}" -Wl,--gc-sections,--print-memory-usage
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/board_bringup.map")
set_property(TARGET board_bringup.elf APPEND PROPERTY LINK_DEPENDS "${RM_LINKER}")
add_custom_command(TARGET board_bringup.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:board_bringup.elf> ${CMAKE_CURRENT_BINARY_DIR}/board_bringup.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:board_bringup.elf> ${CMAKE_CURRENT_BINARY_DIR}/board_bringup.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:board_bringup.elf>)

if(RM_REMOTE_PROTOCOL STREQUAL "SBUS")
    # UART1 is the demo console; other firmware retains its vision configuration.
    get_target_property(RM_BOARD_SOURCES rm_board SOURCES)
    add_library(rm_board_chassis OBJECT ${RM_BOARD_SOURCES})
    target_include_directories(rm_board_chassis PRIVATE platform/stm32h7 config/boards/dm_mc02 config/profiles config/robots/chassis_demo)
    target_compile_definitions(rm_board_chassis PRIVATE RM_CHASSIS_DEMO_BOARD=1)
    target_link_libraries(rm_board_chassis PRIVATE rm_hal rm_freertos rm_platform_api)
    target_compile_options(rm_board_chassis PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic>)
    add_executable(chassis_demo.elf
        firmware/dm_mc02/chassis_main.c firmware/dm_mc02/syscalls.c
        applications/board_bringup/bringup.c applications/chassis_demo/chassis_app.c
        config/robots/chassis_demo/binding.c
        $<TARGET_OBJECTS:rm_board_chassis> $<TARGET_OBJECTS:rm_platform> $<TARGET_OBJECTS:rm_runtime>)
    target_include_directories(chassis_demo.elf PRIVATE runtime/freertos applications/board_bringup config/profiles config/robots/bringup)
    target_compile_definitions(chassis_demo.elf PRIVATE RM_CHASSIS_DEMO=1 RM_ENABLE_MOTION=$<BOOL:${RM_CHASSIS_ENABLE_MOTION}>)
    target_link_libraries(chassis_demo.elf PRIVATE rm_chassis_logic rm_services rm_logic rm_freertos rm_hal rm_platform_api m)
    target_compile_options(chassis_demo.elf PRIVATE -Wall -Wextra -Wpedantic)
    target_link_options(chassis_demo.elf PRIVATE --specs=nano.specs --specs=nosys.specs -nostartfiles
        "-T${RM_LINKER}" -Wl,--gc-sections,--print-memory-usage "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/chassis_demo.map")
    set_property(TARGET chassis_demo.elf APPEND PROPERTY LINK_DEPENDS "${RM_LINKER}")
    add_custom_command(TARGET chassis_demo.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:chassis_demo.elf> ${CMAKE_CURRENT_BINARY_DIR}/chassis_demo.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:chassis_demo.elf> ${CMAKE_CURRENT_BINARY_DIR}/chassis_demo.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:chassis_demo.elf>)

    function(rm_add_motor_test target)
        add_executable(${target}.elf
            firmware/dm_mc02/motor_test_main.c firmware/dm_mc02/syscalls.c
            applications/board_bringup/bringup.c applications/motor_test/motor_test.c applications/motor_test/motor_test_control.c applications/motor_test/motor_test_dm.c
            config/robots/motor_test/binding.c
            $<TARGET_OBJECTS:rm_board_chassis> $<TARGET_OBJECTS:rm_platform> $<TARGET_OBJECTS:rm_runtime>)
        target_include_directories(${target}.elf PRIVATE config/robots/motor_test runtime/freertos applications/board_bringup config/profiles config/robots/bringup)
        target_compile_definitions(${target}.elf PRIVATE RM_CHASSIS_DEMO=1 RM_ENABLE_MOTION=$<BOOL:${RM_MOTOR_TEST_ENABLE_MOTION}>)
        target_link_libraries(${target}.elf PRIVATE rm_chassis_logic rm_services rm_logic rm_freertos rm_hal rm_platform_api m)
        target_compile_options(${target}.elf PRIVATE -Wall -Wextra -Wpedantic)
        target_link_options(${target}.elf PRIVATE --specs=nano.specs --specs=nosys.specs -nostartfiles
            "-T${RM_LINKER}" -Wl,--gc-sections,--print-memory-usage "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${target}.map")
        set_property(TARGET ${target}.elf APPEND PROPERTY LINK_DEPENDS "${RM_LINKER}")
        add_custom_command(TARGET ${target}.elf POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}.elf> ${CMAKE_CURRENT_BINARY_DIR}/${target}.bin
            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}.elf> ${CMAKE_CURRENT_BINARY_DIR}/${target}.hex
            COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}.elf>)
        if(ARGC GREATER 1)
            target_compile_definitions(${target}.elf PRIVATE MOTOR_TEST_MODEL=${ARGV1})
        endif()
    endfunction()
    rm_add_motor_test(motor_test)
    rm_add_motor_test(motor_test_dm4310 4310)

    add_library(rm_board_mecanum OBJECT ${RM_BOARD_SOURCES})
    target_include_directories(rm_board_mecanum PRIVATE platform/stm32h7 config/boards/dm_mc02 config/profiles config/robots/mecanum_demo)
    target_compile_definitions(rm_board_mecanum PRIVATE RM_MECANUM_DEMO_BOARD=1)
    target_link_libraries(rm_board_mecanum PRIVATE rm_hal rm_freertos rm_platform_api)
    target_compile_options(rm_board_mecanum PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic>)
    add_executable(mecanum_demo.elf
        firmware/dm_mc02/mecanum_main.c firmware/dm_mc02/syscalls.c
        applications/board_bringup/bringup.c applications/mecanum_demo/mecanum_app.c
        config/robots/mecanum_demo/binding.c
        $<TARGET_OBJECTS:rm_board_mecanum> $<TARGET_OBJECTS:rm_platform> $<TARGET_OBJECTS:rm_runtime>)
    target_include_directories(mecanum_demo.elf PRIVATE runtime/freertos applications/board_bringup config/profiles config/robots/bringup)
    target_compile_definitions(mecanum_demo.elf PRIVATE RM_MECANUM_DEMO=1 RM_CHASSIS_DEMO=1 RM_ENABLE_MOTION=$<BOOL:${RM_MECANUM_ENABLE_MOTION}>)
    target_link_libraries(mecanum_demo.elf PRIVATE rm_mecanum_logic rm_services rm_logic rm_freertos rm_hal rm_platform_api m)
    target_compile_options(mecanum_demo.elf PRIVATE -Wall -Wextra -Wpedantic)
    target_link_options(mecanum_demo.elf PRIVATE --specs=nano.specs --specs=nosys.specs -nostartfiles
        "-T${RM_LINKER}" -Wl,--gc-sections,--print-memory-usage "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/mecanum_demo.map")
    set_property(TARGET mecanum_demo.elf APPEND PROPERTY LINK_DEPENDS "${RM_LINKER}")
    add_custom_command(TARGET mecanum_demo.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:mecanum_demo.elf> ${CMAKE_CURRENT_BINARY_DIR}/mecanum_demo.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:mecanum_demo.elf> ${CMAKE_CURRENT_BINARY_DIR}/mecanum_demo.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:mecanum_demo.elf>)
endif()
