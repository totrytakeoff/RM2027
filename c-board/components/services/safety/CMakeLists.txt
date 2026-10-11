add_library(rm_service_safety STATIC
    safety_manager.c
)
add_library(RM::service_safety ALIAS rm_service_safety)

target_include_directories(rm_service_safety PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(rm_service_safety PRIVATE ${MCU_FLAGS})
