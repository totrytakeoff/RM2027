add_library(rm_robot_config INTERFACE)
add_library(RM::robot_config ALIAS rm_robot_config)

target_include_directories(rm_robot_config INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

add_library(rm_robot_app STATIC
    robot_app.c
    command/input.cpp
    command/et08_input_profile.cpp
    chassis/chassis.c
    chassis/chassis_follow.c
    chassis/chassis_kinematics.c
    diagnostics/debug.c
    diagnostics/tuning_telemetry.c
    gimbal/gimbal.c
    gimbal/gimbal_axis_state.c
    gimbal/gimbal_limits.c
    referee/referee.c
    shoot/shoot.c
    shoot/shoot_trigger.c
)
add_library(RM::robot_app ALIAS rm_robot_app)

target_include_directories(rm_robot_app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/command
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/diagnostics
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal
    ${CMAKE_CURRENT_SOURCE_DIR}/referee
    ${CMAKE_CURRENT_SOURCE_DIR}/shoot
)

target_compile_options(rm_robot_app PRIVATE ${MCU_FLAGS})
target_compile_features(rm_robot_app PRIVATE cxx_std_14)
set_target_properties(rm_robot_app PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
target_compile_options(rm_robot_app PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-asynchronous-unwind-tables>
)

target_link_libraries(rm_robot_app PUBLIC
    rm_robot_config
    rm_service_safety
    rm_components_runtime
    rm_platform_stm32f4
    rm_common
    rm_vendor_stm32_hal
    m
)
