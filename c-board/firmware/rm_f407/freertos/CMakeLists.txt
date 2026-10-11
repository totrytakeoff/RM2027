add_library(rm_robot_freertos STATIC
    robot_tasks.c
    rtos_hooks.c
)
add_library(RM::robot_freertos ALIAS rm_robot_freertos)

target_include_directories(rm_robot_freertos PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(rm_robot_freertos PRIVATE ${MCU_FLAGS})

target_link_libraries(rm_robot_freertos PUBLIC
    rm_robot_app
    rm_components_runtime
    rm_platform_stm32f4
    rm_vendor_runtime
)
