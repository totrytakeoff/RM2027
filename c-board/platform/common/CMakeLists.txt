add_library(rm_platform_core STATIC
    transport/rm_rx_queue.c
)
add_library(RM::platform_core ALIAS rm_platform_core)

target_include_directories(rm_platform_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/transport
)

target_compile_features(rm_platform_core PUBLIC c_std_11)
target_compile_options(rm_platform_core PRIVATE ${MCU_FLAGS})
