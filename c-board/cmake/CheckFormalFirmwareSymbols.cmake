if(NOT DEFINED NM OR NOT EXISTS "${NM}")
    message(FATAL_ERROR "ARM nm executable is unavailable: ${NM}")
endif()
if(NOT DEFINED ELF OR NOT EXISTS "${ELF}")
    message(FATAL_ERROR "Formal firmware ELF is unavailable: ${ELF}")
endif()

execute_process(
    COMMAND "${NM}" "${ELF}"
    RESULT_VARIABLE nm_result
    OUTPUT_VARIABLE nm_output
    ERROR_VARIABLE nm_error
)
if(NOT nm_result EQUAL 0)
    message(FATAL_ERROR "Unable to audit ${ELF}: ${nm_error}")
endif()

set(forbidden_symbols
    malloc
    _malloc_r
    calloc
    _calloc_r
    realloc
    _realloc_r
    free
    _free_r
    pvPortMalloc
    vPortFree
    _sbrk
    _sbrk_r
    snprintf
    vsnprintf
    _Znwj
    _Znaj
    _ZdlPv
    _ZdaPv
    _ZdlPvj
    _ZdaPvj
    __cxa_allocate_exception
    __cxa_throw
    __cxa_guard_acquire
    __cxa_guard_release
    __cxa_atexit
    __gxx_personality_v0
    _Unwind_Resume
)

foreach(symbol IN LISTS forbidden_symbols)
    string(REGEX MATCH "(^|\n)[^\n]*[ \t]${symbol}(\n|$)" match
           "${nm_output}")
    if(match)
        message(FATAL_ERROR
            "Formal firmware contains forbidden runtime symbol: ${symbol}")
    endif()
endforeach()

message(STATUS "Formal firmware heap/stdio/C++ runtime symbol audit passed")
