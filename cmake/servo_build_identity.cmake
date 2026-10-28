# Re-run at every build, but rewrite the header only when identity changes.
execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE revision OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
execute_process(COMMAND git status --porcelain --untracked-files=normal WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE status OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
execute_process(COMMAND git diff HEAD -- WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE changes ERROR_QUIET)
string(SHA256 diff_hash "${changes}\n${status}")
file(GLOB_RECURSE inputs LIST_DIRECTORIES false
    "${SOURCE_DIR}/src/*.cpp" "${SOURCE_DIR}/include/*.h" "${SOURCE_DIR}/include/*.hpp"
    "${SOURCE_DIR}/orchestrator/cpp/*.cpp" "${SOURCE_DIR}/orchestrator/cpp/*.h"
    "${SOURCE_DIR}/config/*.yaml" "${SOURCE_DIR}/cmake/*.cmake")
list(APPEND inputs "${SOURCE_DIR}/CMakeLists.txt")
list(SORT inputs)
set(source_digests "")
foreach(input IN LISTS inputs)
    file(SHA256 "${input}" digest)
    file(RELATIVE_PATH name "${SOURCE_DIR}" "${input}")
    string(APPEND source_digests "${name}:${digest}\n")
endforeach()
string(SHA256 source_hash "${source_digests}")
if(status)
    set(dirty 1)
else()
    set(dirty 0)
endif()
set(content "#pragma once\n#define T170_SERVO_FULL_REVISION \"${revision}\"\n#define T170_SERVO_BUILD_DIRTY ${dirty}\n#define T170_SERVO_DIFF_SHA256 \"${diff_hash}\"\n#define T170_SERVO_SOURCE_SHA256 \"${source_hash}\"\n")
set(previous "")
if(EXISTS "${OUTPUT_FILE}")
    file(READ "${OUTPUT_FILE}" previous)
endif()
if(NOT previous STREQUAL content)
    file(WRITE "${OUTPUT_FILE}" "${content}")
endif()
