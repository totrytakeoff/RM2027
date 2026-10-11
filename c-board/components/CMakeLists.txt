# Shared repository-owned types.  This remains header-only until the public
# data contracts are split into narrower component APIs.
add_library(rm_common INTERFACE)
add_library(RM::common ALIAS rm_common)
target_include_directories(rm_common INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

add_subdirectory(services/safety)

# Production-supported component set used by the formal robot firmware.
# Keep this list narrow: compatibility demos must not silently promote every
# historical driver and heap-backed registration API into app.elf.
add_library(rm_components_runtime STATIC
    algorithms/checksum/crc16.c
    algorithms/checksum/crc8.c
    algorithms/filters/kalman/kalman_filter.c
    algorithms/math/user_lib.c
    algorithms/pid/controller.c
    algorithms/quaternion_ekf/QuaternionEKF.c
    devices/imu/bmi088/BMI088Middleware.c
    devices/imu/bmi088/BMI088driver.c
    devices/imu/ins/ins_task.c
    devices/motor/dji/dji_motor.c
    devices/referee/crc_ref.c
    devices/referee/rm_referee.c
    devices/remote/dt7/dt7_remote.c
    devices/remote/et08/et08_remote.c
    devices/remote/vt/vt_remote.c
    devices/vision/vision_link.c
    devices/vision/vision_link_state.c
    devices/vision/vision_protocol.c
    services/device_health/daemon.c
)
add_library(RM::components_runtime ALIAS rm_components_runtime)

target_include_directories(rm_components_runtime PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/checksum
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/filters/kalman
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/pid
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/quaternion_ekf
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/imu/bmi088
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/imu/ins
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/motor
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/motor/dji
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/referee
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/remote
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/remote/dt7
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/remote/et08
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/remote/vt
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/vision
    ${CMAKE_CURRENT_SOURCE_DIR}/services/device_health
)

target_link_libraries(rm_components_runtime PUBLIC
    rm_common
    rm_platform_stm32f4
    rm_vendor_cmsis_dsp
    rm_vendor_runtime
    rm_vendor_stm32_hal
)
target_compile_options(rm_components_runtime PRIVATE ${MCU_FLAGS})

# Historical drivers retained only for board demos and regression builds.
# They remain reachable through RM::components, but not through the formal
# runtime target above.  The old vision protocol is kept here for legacy
# demos; the production target uses vision_link + vision_protocol only.
add_library(rm_components_compat STATIC
    devices/actuator/buzzer/buzzer.c
    devices/communication/can_comm/can_comm.c
    devices/communication/hc05/HC05.c
    devices/display/oled/oled.c
    devices/imu/bmi088_alt/bmi088.c
    devices/imu/ist8310/ist8310.c
    devices/motor/dm/dm8009p.c
    devices/motor/dm/dmmotor.c
    devices/motor/ht/HT04.c
    devices/motor/lk/LK9025.c
    devices/motor/motor_task.c
    devices/motor/servo_motor/servo_motor.c
    devices/motor/step_motor/step_motor.c
    devices/power/supercap/super_cap.c
    devices/rangefinder/tfmini_plus/tfminiplus.c
    devices/referee/referee_UI.c
    devices/referee/referee_task.c
    devices/vision/master_process.c
    devices/vision/serial_frame_protocol.c
    services/command/std_cmd.c
    services/message_bus/message_center.c
    services/transport/unicomm/unicomm.c
)

target_include_directories(rm_components_compat PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/actuator/buzzer
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/communication/can_comm
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/communication/hc05
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/display/oled
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/imu/bmi088_alt
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/imu/ist8310
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/motor
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/motor/dm
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/motor/ht
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/motor/lk
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/motor/servo_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/motor/step_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/power/supercap
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/rangefinder/tfmini_plus
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/remote
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/vision
    ${CMAKE_CURRENT_SOURCE_DIR}/services/command
    ${CMAKE_CURRENT_SOURCE_DIR}/services/message_bus
    ${CMAKE_CURRENT_SOURCE_DIR}/services/transport/unicomm
)

target_link_libraries(rm_components_compat PUBLIC
    rm_components_runtime
)
target_compile_options(rm_components_compat PRIVATE ${MCU_FLAGS})

# Compatibility umbrella for existing demo CMakeLists. New firmware should
# select RM::components_runtime or narrower targets explicitly.
add_library(rm_components INTERFACE)
add_library(RM::components ALIAS rm_components)
target_link_libraries(rm_components INTERFACE rm_components_compat)

add_library(rm_device_dm_imu STATIC
    devices/imu/dm/dm_imu.c
)
add_library(RM::device_dm_imu ALIAS rm_device_dm_imu)
target_include_directories(rm_device_dm_imu PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/imu/dm
)
target_link_libraries(rm_device_dm_imu PUBLIC
    rm_common
    rm_platform_stm32f4
    rm_vendor_stm32_hal
)
target_compile_options(rm_device_dm_imu PRIVATE ${MCU_FLAGS})
