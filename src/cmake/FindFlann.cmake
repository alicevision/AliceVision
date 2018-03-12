###########################################################
#                  Find Flann Library
#----------------------------------------------------------

find_path(FLANN_DIR flann.hpp
    HINTS "${FLANN_ROOT}" "$ENV{FLANN_ROOT}" "${FLANN_INCLUDE_DIR_HINTS}"
    PATHS "$ENV{PROGRAMFILES}/flann" "$ENV{PROGRAMW6432}/flann"
    PATH_SUFFIXES flann
    DOC "Root directory of FLANN includes")

##====================================================
## Include FLANN library
##----------------------------------------------------
if(EXISTS "${FLANN_DIR}" AND NOT "${FLANN_DIR}" STREQUAL "")
  set(FLANN_FOUND TRUE)
  set(FLANN_INCLUDE_DIRS ${FLANN_DIR})
  set(FLANN_DIR "${FLANN_DIR}" CACHE PATH "" FORCE)
  MARK_AS_ADVANCED(FLANN_DIR)

  # Extract Flann version from config.h
  set(FLANN_VERSION_FILE ${FLANN_INCLUDE_DIRS}/config.h)
  if(NOT EXISTS ${FLANN_VERSION_FILE})
    FLANN_REPORT_NOT_FOUND(
      "Could not find file: ${FLANN_VERSION_FILE} "
      "containing version information in Flann install located at: "
      "${FLANN_INCLUDE_DIRS}.")
  else(NOT EXISTS ${FLANN_VERSION_FILE})
    file(READ ${FLANN_VERSION_FILE} FLANN_VERSION_FILE_CONTENTS)
    string(REGEX MATCH "#define FLANN_VERSION_ \"([0-9.]+)\""
      FLANN_VERSION "${FLANN_VERSION_FILE_CONTENTS}")
    string(REGEX REPLACE "#define FLANN_VERSION_ \"([0-9.]+)\"" "\\1"
      FLANN_VERSION "${FLANN_VERSION}")
  endif(NOT EXISTS ${FLANN_VERSION_FILE})
  set(FLANN_INCLUDE_DIR ${FLANN_DIR})
  find_library(FLANN_LIBRARY NAMES flann_cpp)

  # locate Flann libraries
  if(DEFINED FLANN_LIBRARY)
    set(FLANN_LIBRARIES ${FLANN_LIBRARY})
  endif()

  message(STATUS "Flann ${FLANN_VERSION} found (include: ${FLANN_INCLUDE_DIRS})")
else()
  message(FATAL_ERROR "You are attempting to build without Flann. "
          "Please use cmake variable -DFLANN_INCLUDE_DIR_HINTS:STRING=\"PATH\" "
          "or FLANN_INCLUDE_DIR_HINTS env. variable to a valid Flann path. "
          "Or install last Flann version.")
  package_report_not_found(FLANN "Flann cannot be found")
endif()
##====================================================
