if(NOT DEFINED INPUT_FILE)
    message(FATAL_ERROR "INPUT_FILE is required")
endif()

if(NOT EXISTS "${INPUT_FILE}")
    message(FATAL_ERROR "Patch target file not found: ${INPUT_FILE}")
endif()

file(READ "${INPUT_FILE}" FILE_CONTENT)

if(FILE_CONTENT MATCHES "#include <cstdint>")
    message(STATUS "Assimp draco patch already applied: ${INPUT_FILE}")
    return()
endif()

set(NEEDLE "#include <vector>")
if(NOT FILE_CONTENT MATCHES "${NEEDLE}")
    message(FATAL_ERROR "Expected include not found in ${INPUT_FILE}: ${NEEDLE}")
endif()

string(REPLACE
    "#include <vector>"
    "#include <vector>\n#include <cstdint>"
    PATCHED_CONTENT
    "${FILE_CONTENT}"
)

if(PATCHED_CONTENT STREQUAL FILE_CONTENT)
    message(FATAL_ERROR "Failed to patch ${INPUT_FILE}")
endif()

file(WRITE "${INPUT_FILE}" "${PATCHED_CONTENT}")
message(STATUS "Applied GCC 13 draco cstdint patch: ${INPUT_FILE}")
