if(NOT DEFINED READELF OR NOT EXISTS "${READELF}")
    message(FATAL_ERROR "ARM readelf executable is unavailable: ${READELF}")
endif()
if(NOT DEFINED ELF OR NOT EXISTS "${ELF}")
    message(FATAL_ERROR "ELF is unavailable for segment audit: ${ELF}")
endif()

execute_process(
    COMMAND "${READELF}" -lW "${ELF}"
    RESULT_VARIABLE readelf_result
    OUTPUT_VARIABLE program_headers
    ERROR_VARIABLE readelf_error
)
if(NOT readelf_result EQUAL 0)
    message(FATAL_ERROR "Unable to inspect ${ELF}: ${readelf_error}")
endif()

string(REGEX MATCH "LOAD[^\n]*RWE" rwx_segment "${program_headers}")
if(rwx_segment)
    message(FATAL_ERROR "ELF contains a writable/executable LOAD segment: ${rwx_segment}")
endif()

message(STATUS "ELF LOAD-segment permission audit passed")
