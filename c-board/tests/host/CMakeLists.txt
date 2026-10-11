cmake_minimum_required(VERSION 3.16)

project(rm2026_host_unit_tests C CXX)

enable_testing()

add_library(rm_safety_host STATIC
    ../../components/services/safety/safety_manager.c
)
target_include_directories(rm_safety_host PUBLIC
    ../../components/services/safety
)
target_compile_features(rm_safety_host PUBLIC c_std_11)
target_compile_options(rm_safety_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_safety_manager
    safety_manager_test.c
)
target_link_libraries(test_safety_manager PRIVATE rm_safety_host)
target_compile_features(test_safety_manager PRIVATE c_std_11)
target_compile_options(test_safety_manager PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_test(NAME safety_manager COMMAND test_safety_manager)

add_executable(test_remote_control_adapter
    remote_control_adapter_test.cpp
)
target_include_directories(test_remote_control_adapter PRIVATE
    ../../components/devices/remote
)
target_compile_features(test_remote_control_adapter PRIVATE cxx_std_14)
target_compile_options(test_remote_control_adapter PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
add_test(NAME remote_control_adapter COMMAND test_remote_control_adapter)

add_library(rm_et08_profile_host STATIC
    ../../applications/robot/command/et08_input_profile.cpp
)
target_include_directories(rm_et08_profile_host PUBLIC
    ../../applications/robot/command
    ../../applications/robot/include
    ../../applications/robot/referee
    ../../components/devices/remote
    ../../components/devices/remote/et08
)
target_compile_features(rm_et08_profile_host PUBLIC cxx_std_14)
target_compile_options(rm_et08_profile_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_et08_input_profile
    et08_input_profile_test.cpp
)
target_link_libraries(test_et08_input_profile PRIVATE
    rm_et08_profile_host
    rm_safety_host
)
target_compile_features(test_et08_input_profile PRIVATE cxx_std_14)
target_compile_options(test_et08_input_profile PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
add_test(NAME et08_input_profile
         COMMAND test_et08_input_profile)

add_library(rm_gimbal_limits_host STATIC
    ../../applications/robot/gimbal/gimbal_limits.c
)
target_include_directories(rm_gimbal_limits_host PUBLIC
    ../../applications/robot/gimbal
)
target_compile_features(rm_gimbal_limits_host PUBLIC c_std_11)
target_compile_options(rm_gimbal_limits_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_gimbal_limits
    gimbal_limits_test.c
)
target_link_libraries(test_gimbal_limits PRIVATE
    rm_gimbal_limits_host
)
target_compile_features(test_gimbal_limits PRIVATE c_std_11)
target_compile_options(test_gimbal_limits PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
add_test(NAME gimbal_limits COMMAND test_gimbal_limits)

add_library(rm_gimbal_axis_state_host STATIC
    ../../applications/robot/gimbal/gimbal_axis_state.c
)
target_include_directories(rm_gimbal_axis_state_host PUBLIC
    ../../applications/robot/gimbal
    ../../applications/robot/include
    ../../applications/robot/referee
)
target_compile_features(rm_gimbal_axis_state_host PUBLIC c_std_11)
target_compile_options(rm_gimbal_axis_state_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_gimbal_axis_state
    gimbal_axis_state_test.c
)
target_link_libraries(test_gimbal_axis_state PRIVATE
    rm_gimbal_axis_state_host
    m
)
target_compile_features(test_gimbal_axis_state PRIVATE c_std_11)
target_compile_options(test_gimbal_axis_state PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
add_test(NAME gimbal_axis_state
         COMMAND test_gimbal_axis_state)

add_executable(test_gimbal_units
    gimbal_units_test.c
)
target_include_directories(test_gimbal_units PRIVATE
    ../../applications/robot/gimbal
)
target_compile_features(test_gimbal_units PRIVATE c_std_11)
target_compile_options(test_gimbal_units PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
target_link_libraries(test_gimbal_units PRIVATE m)
add_test(NAME gimbal_units COMMAND test_gimbal_units)

add_executable(test_shoot_config
    shoot_config_test.c
)
target_include_directories(test_shoot_config PRIVATE
    stubs
    ../../applications/robot/config
)
target_compile_features(test_shoot_config PRIVATE c_std_11)
target_compile_options(test_shoot_config PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
target_link_libraries(test_shoot_config PRIVATE m)
add_test(NAME shoot_config COMMAND test_shoot_config)

add_library(rm_shoot_trigger_host STATIC
    ../../applications/robot/shoot/shoot_trigger.c
)
target_include_directories(rm_shoot_trigger_host PUBLIC
    ../../applications/robot/shoot
)
target_compile_features(rm_shoot_trigger_host PUBLIC c_std_11)
target_compile_options(rm_shoot_trigger_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_shoot_trigger
    shoot_trigger_test.c
)
target_link_libraries(test_shoot_trigger PRIVATE
    rm_shoot_trigger_host
)
target_compile_features(test_shoot_trigger PRIVATE c_std_11)
target_compile_options(test_shoot_trigger PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
add_test(NAME shoot_trigger COMMAND test_shoot_trigger)

add_library(rm_chassis_kinematics_host STATIC
    ../../applications/robot/chassis/chassis_follow.c
    ../../applications/robot/chassis/chassis_kinematics.c
)
target_include_directories(rm_chassis_kinematics_host PUBLIC
    ../../applications/robot/chassis
)
target_compile_features(rm_chassis_kinematics_host PUBLIC c_std_11)
target_compile_options(rm_chassis_kinematics_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_chassis_kinematics
    chassis_kinematics_test.c
)
target_link_libraries(test_chassis_kinematics PRIVATE
    rm_chassis_kinematics_host
    m
)
target_compile_features(test_chassis_kinematics PRIVATE c_std_11)
target_compile_options(test_chassis_kinematics PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
add_test(NAME chassis_kinematics
         COMMAND test_chassis_kinematics)

add_library(rm_device_health_host STATIC
    ../../components/services/device_health/daemon.c
)
target_include_directories(rm_device_health_host PUBLIC
    ../../components/services/device_health
    ../../platform/stm32f4/time
)
target_compile_features(rm_device_health_host PUBLIC c_std_11)
target_compile_options(rm_device_health_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_device_health
    device_health_test.c
)
target_link_libraries(test_device_health PRIVATE rm_device_health_host)
target_compile_features(test_device_health PRIVATE c_std_11)
target_compile_options(test_device_health PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_test(NAME device_health COMMAND test_device_health)

add_library(rm_format_host STATIC
    ../../platform/stm32f4/utils/utils.c
)
target_include_directories(rm_format_host PUBLIC
    ../../platform/stm32f4/utils
)
target_compile_features(rm_format_host PUBLIC c_std_11)
target_compile_options(rm_format_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_format
    format_test.c
)
target_link_libraries(test_format PRIVATE rm_format_host m)
target_compile_features(test_format PRIVATE c_std_11)
target_compile_options(test_format PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_test(NAME format COMMAND test_format)

add_library(rm_rx_queue_host STATIC
    ../../platform/common/transport/rm_rx_queue.c
)
target_include_directories(rm_rx_queue_host PUBLIC
    ../../platform/common/transport
)
target_compile_features(rm_rx_queue_host PUBLIC c_std_11)
target_compile_options(rm_rx_queue_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_rx_queue
    rx_queue_test.c
)
target_link_libraries(test_rx_queue PRIVATE rm_rx_queue_host)
target_compile_features(test_rx_queue PRIVATE c_std_11)
target_compile_options(test_rx_queue PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_test(NAME rx_queue COMMAND test_rx_queue)

add_library(rm_pid_host STATIC
    ../../components/algorithms/pid/controller.c
)
target_include_directories(rm_pid_host PUBLIC
    stubs
    ../../components/algorithms/pid
)
target_compile_features(rm_pid_host PUBLIC c_std_11)
target_compile_options(rm_pid_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_pid_reset
    pid_reset_test.c
)
target_link_libraries(test_pid_reset PRIVATE rm_pid_host m)
target_compile_features(test_pid_reset PRIVATE c_std_11)
target_compile_options(test_pid_reset PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_test(NAME pid_reset COMMAND test_pid_reset)

add_library(rm_vision_protocol_host STATIC
    ../../components/devices/vision/vision_protocol.c
)
target_include_directories(rm_vision_protocol_host PUBLIC
    ../../components/devices/vision
)
target_compile_features(rm_vision_protocol_host PUBLIC c_std_11)
target_compile_options(rm_vision_protocol_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_vision_protocol
    vision_protocol_test.c
)
target_link_libraries(test_vision_protocol PRIVATE
    rm_vision_protocol_host
    m
)
target_compile_features(test_vision_protocol PRIVATE c_std_11)
target_compile_options(test_vision_protocol PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_test(NAME vision_protocol COMMAND test_vision_protocol)

add_library(rm_vision_link_state_host STATIC
    ../../components/devices/vision/vision_link_state.c
)
target_include_directories(rm_vision_link_state_host PUBLIC
    ../../components/devices/vision
)
target_compile_features(rm_vision_link_state_host PUBLIC c_std_11)
target_compile_options(rm_vision_link_state_host PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_executable(test_vision_link_state
    vision_link_state_test.c
)
target_link_libraries(test_vision_link_state PRIVATE
    rm_vision_link_state_host
)
target_compile_features(test_vision_link_state PRIVATE c_std_11)
target_compile_options(test_vision_link_state PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

add_test(NAME vision_link_state COMMAND test_vision_link_state)

add_executable(test_vision_aim
    vision_aim_test.c
)
target_include_directories(test_vision_aim PRIVATE
    ../../applications/robot/gimbal
)
target_compile_features(test_vision_aim PRIVATE c_std_11)
target_compile_options(test_vision_aim PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
target_link_libraries(test_vision_aim PRIVATE m)

add_test(NAME vision_aim COMMAND test_vision_aim)
