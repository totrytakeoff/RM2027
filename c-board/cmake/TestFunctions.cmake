# =============================================================================
# @file cmake/TestFunctions.cmake
# @brief RoboMaster 嵌入式测试项目函数集合
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 提供 create_embedded_test() 与 create_unit_test()，统一处理链接、下载命令与产物输出路径。
# =============================================================================

# 创建嵌入式测试项目的函数
function(create_embedded_test test_name)
    # 解析参数
    set(options "")
    set(oneValueArgs "LINKER_SCRIPT")
    set(multiValueArgs "SOURCES" "EXTRA_SOURCES" "INCLUDE_DIRS" "LINK_LIBRARIES")
    
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    # 设置默认链接脚本
    if(NOT ARG_LINKER_SCRIPT)
        set(ARG_LINKER_SCRIPT "${MCU_LINKER_SCRIPT}")
    endif()
    
    # 规范化名称，统一输出/目标命名：去掉前缀 test_ 或后缀 _test，再加上 test_ 前缀
    set(test_raw_name "${test_name}")
    string(REGEX REPLACE "^test_" "" test_base "${test_raw_name}")
    string(REGEX REPLACE "_test$" "" test_base "${test_base}")
    if(test_base STREQUAL "")
        set(test_base "${test_raw_name}")
    endif()

    set(test_target_name "test_${test_base}")
    set(test_output_dir ${TEST_OUTPUT_DIR}/${test_base})

    # 统一产物收集目录（由顶层定义）
    if(NOT DEFINED GLOBAL_OUTPUT_DIR)
        set(GLOBAL_OUTPUT_DIR ${CMAKE_BINARY_DIR}/output)
    endif()
    
    # 创建测试可执行文件。板级demo只在显式指定目标时构建，不能污染正式固件的
    # 默认构建；部分历史demo保留独立参数契约，后续按需逐个迁移。
    # 启动文件由板级 target 统一提供。
    # 只需要添加测试本身的源文件即可。
    add_executable(${test_target_name} EXCLUDE_FROM_ALL
        ${ARG_SOURCES}
        ${ARG_EXTRA_SOURCES}
        ${CMAKE_SOURCE_DIR}/tests/firmware/error_handler_stub.c
    )
    set_property(TARGET ${test_target_name} APPEND PROPERTY
        LINK_DEPENDS ${ARG_LINKER_SCRIPT})
    
    # 汇编文件属性在板级 target 中处理。
    
    if(NOT DSP_LIB OR NOT EXISTS ${DSP_LIB})
        message(FATAL_ERROR "DSP_LIB not found: ${DSP_LIB}")
    endif()

    # 包含目录
    target_include_directories(${test_target_name}
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${ARG_INCLUDE_DIRS}
    )
    
    # 链接库 (mirroring the main app.elf linkage)
    target_link_libraries(${test_target_name}
        PUBLIC
        rm_platform_syscalls
        -Wl,--start-group
        ${ARG_LINK_LIBRARIES}
        rm_board_rm_f407
        $<TARGET_OBJECTS:rm_board_rm_f407_irq>
        -Wl,--end-group
        rm_vendor_cmsis_dsp
        m
        c
    )
    
    # 链接选项 (mirroring the main app.elf options)
    target_link_options(${test_target_name} PRIVATE 
        ${GENERIC_LINK_OPTIONS}
        -T${ARG_LINKER_SCRIPT} 
        -Wl,-Map=${test_output_dir}/${test_target_name}.map 
        -Wl,--print-memory-usage
    )
    
    # 设置输出目录
    # 目标输出为 .elf，并放到 tests/<name>/ 目录，方便调试和烧录
    set_target_properties(${test_target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${test_output_dir}
        OUTPUT_NAME ${test_target_name}
        SUFFIX ".elf"
    )
    
    # 创建输出目录
    file(MAKE_DIRECTORY ${test_output_dir})
    file(MAKE_DIRECTORY ${GLOBAL_OUTPUT_DIR})
    
    # 生成HEX和BIN文件
    add_custom_command(TARGET ${test_target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND}
                -DREADELF=${CMAKE_READELF}
                -DELF=$<TARGET_FILE:${test_target_name}>
                -P ${CMAKE_SOURCE_DIR}/cmake/CheckElfSegments.cmake
        COMMAND ${CMAKE_COMMAND} -E make_directory ${test_output_dir}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${GLOBAL_OUTPUT_DIR}
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${test_target_name}> ${test_output_dir}/${test_target_name}.hex
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${test_target_name}> ${test_output_dir}/${test_target_name}.bin
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${test_target_name}> ${test_output_dir}/${test_target_name}.elf
        COMMAND ${CMAKE_COMMAND} -E copy ${test_output_dir}/${test_target_name}.elf ${GLOBAL_OUTPUT_DIR}/${test_target_name}.elf
        COMMAND ${CMAKE_COMMAND} -E copy ${test_output_dir}/${test_target_name}.hex ${GLOBAL_OUTPUT_DIR}/${test_target_name}.hex
        COMMAND ${CMAKE_COMMAND} -E copy ${test_output_dir}/${test_target_name}.bin ${GLOBAL_OUTPUT_DIR}/${test_target_name}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${test_target_name}>
        COMMENT "为测试项目 ${test_name} 生成 ELF / HEX / BIN 文件"
    )
    
    # 添加到测试目标列表 (此函数未定义,暂时注释掉)
    # add_test_executable(${test_target_name})
    
    # =============================================================================
    # 下载目标
    # =============================================================================
    
    # OpenOCD下载目标
    find_program(OPENOCD openocd PATHS
        /usr/bin
        /usr/local/bin
        /home/myself/.platformio/packages/tool-openocd/bin
        /opt/openocd/bin
    )
    
    if(OPENOCD)
        add_custom_target(flash-${test_target_name}
            COMMAND ${OPENOCD} 
                    -f ${RM_BOARD_ROOT}/openocd/openocd_dap.cfg
                    -c init -c halt 
                    -c "flash write_image erase ${test_output_dir}/${test_target_name}.bin 0x08000000" 
                    -c "verify_image ${test_output_dir}/${test_target_name}.bin 0x08000000"
                    -c reset -c shutdown
            COMMENT "下载测试项目 ${test_name} 到STM32"
            DEPENDS ${test_target_name}
        )
        
        add_custom_target(build-and-flash-${test_target_name}
            COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${test_target_name}
            COMMAND ${OPENOCD} 
                    -f ${RM_BOARD_ROOT}/openocd/openocd_dap.cfg
                    -c init -c halt 
                    -c "flash write_image erase ${test_output_dir}/${test_target_name}.bin 0x08000000" 
                    -c "verify_image ${test_output_dir}/${test_target_name}.bin 0x08000000"
                    -c reset -c shutdown
            COMMENT "编译并下载测试项目 ${test_name} 到STM32"
        )
    endif()
    
    # JLink下载目标
    find_program(JLINK_EXE JLinkExe PATHS
        /usr/bin
        /usr/local/bin
        /opt/SEGGER/JLink
    )
    
    if(JLINK_EXE)
        set(jlink_script ${CMAKE_BINARY_DIR}/flash_${test_target_name}.jlink)
        # 创建JLink脚本文件
        file(WRITE ${jlink_script}
            "device STM32F407IG\n"
            "speed 4000\n"
            "connect\n"
            "erase\n"
            "loadfile ${test_output_dir}/${test_target_name}.hex\n"
            "r\n"
            "go\n"
            "exit\n"
        )
        
        add_custom_target(flash-${test_target_name}-jlink
            COMMAND ${JLINK_EXE} -CommanderScript ${jlink_script}
            COMMENT "使用JLink下载测试项目 ${test_name} 到STM32"
            DEPENDS ${test_target_name}
        )
    endif()
    
    # =============================================================================
    # 输出信息
    # =============================================================================
    message(STATUS "创建测试项目: ${test_target_name}")
    message(STATUS "  - 源文件: ${ARG_SOURCES}")
    message(STATUS "  - 额外源文件: ${ARG_EXTRA_SOURCES}")
    message(STATUS "  - 链接库: ${ARG_LINK_LIBRARIES}")
    message(STATUS "  - 输出目录: ${test_output_dir}")
    
endfunction()

# =============================================================================
# 创建单元测试函数（用于主机测试）
# =============================================================================
function(create_unit_test test_name)
    # 解析参数
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs "SOURCES" "INCLUDE_DIRS" "LINK_LIBRARIES")
    
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    # 测试目标名称
    set(test_target_name "unit_test_${test_name}")
    
    # 创建测试可执行文件（主机环境）
    add_executable(${test_target_name}
        ${ARG_SOURCES}
    )
    
    # 包含目录
    target_include_directories(${test_target_name}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${RM_BOARD_INCLUDE_DIR}
            ${ARG_INCLUDE_DIRS}
    )
    
    # 链接库
    target_link_libraries(${test_target_name}
        PRIVATE
            ${ARG_LINK_LIBRARIES}
    )
    
    # 添加为测试
    add_test(NAME ${test_target_name}
        COMMAND ${test_target_name}
    )
    
    # 添加到测试目标列表 (此函数未定义,暂时注释掉)
    # add_test_executable(${test_target_name})
    
    message(STATUS "创建单元测试: ${test_target_name}")
endfunction()
