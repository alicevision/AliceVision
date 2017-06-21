###########################################################
#                  Find Lemon Library
#----------------------------------------------------------

find_path(LEMON_DIR list_graph.h
    HINTS "${LEMON_ROOT}" "$ENV{LEMON_ROOT}" "${LEMON_INCLUDE_DIR_HINTS}"
    PATHS "$ENV{PROGRAMFILES}/lemon" "$ENV{PROGRAMW6432}/lemon"
    PATH_SUFFIXES lemon
    DOC "Root directory of LEMON includes")

##====================================================
## Include LEMON library
##----------------------------------------------------
if(EXISTS "${LEMON_DIR}" AND NOT "${LEMON_DIR}" STREQUAL "")
  set(LEMON_FOUND TRUE)
  # Remove /lemon from path (math.h cannot be exposed all time)
  get_filename_component(LEMON_INCLUDE_DIRS "${LEMON_DIR}" PATH)
  set(LEMON_DIR "${LEMON_DIR}" CACHE PATH "" FORCE)
  mark_as_advanced(LEMON_DIR)
  # Extract Lemon version from config.h
  set(LEMON_VERSION_FILE ${LEMON_INCLUDE_DIRS}/lemon/config.h)
  if(NOT EXISTS ${LEMON_VERSION_FILE})
    if(Lemon_FIND_REQUIRED)
      message(FATAL_ERROR
        "Could not find file: ${LEMON_VERSION_FILE} "
        "containing version information in Lemon install located at: "
        "${LEMON_INCLUDE_DIRS}.")
    elseif(NOT Lemon_FIND_QUIETLY)
      message(SEND_ERROR
      "Could not find file: ${LEMON_VERSION_FILE} "
      "containing version information in Lemon install located at: "
      "${LEMON_INCLUDE_DIRS}.")
    endif()
  else(NOT EXISTS ${LEMON_VERSION_FILE})
    file(READ ${LEMON_VERSION_FILE} LEMON_VERSION_FILE_CONTENTS)
    string(REGEX MATCH "#define LEMON_VERSION \"([0-9.]+)\""
    LEMON_VERSION "${LEMON_VERSION_FILE_CONTENTS}")
    string(REGEX REPLACE "#define LEMON_VERSION \"([0-9.]+)\"" "\\1"
    LEMON_VERSION "${LEMON_VERSION}")
  endif(NOT EXISTS ${LEMON_VERSION_FILE})
  
  set(LEMON_INCLUDE_DIR ${LEMON_DIR})

  find_library(LEMON_LIBRARY NAMES Lemon lemon emon)

  # locate Lemon libraries
  if(DEFINED LEMON_LIBRARY)
    set(LEMON_LIBRARIES ${LEMON_LIBRARY})
  endif()

  message(STATUS "Lemon ${LEMON_VERSION} found (include: ${LEMON_DIR})")
else()
  message(FATAL_ERROR "You are attempting to build without Lemon. "
          "Please use cmake variable -DLEMON_INCLUDE_DIR_HINTS:STRING=\"PATH\" "
          "or LEMON_INCLUDE_DIR_HINTS env. variable to a valid Lemon path. "
          "Or install last Lemon version.")
  package_report_not_found(LEMON "Lemon cannot be found")
endif()
##====================================================
