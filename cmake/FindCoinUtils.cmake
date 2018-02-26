###########################################################
#                  Find CoinUtils Library
#----------------------------------------------------------

find_path(COINUTILS_DIR CoinUtilsConfig.h
  HINTS "${COINUTILS_ROOT}" "$ENV{COINUTILS_ROOT}" "${COINUTILS_INCLUDE_DIR_HINTS}"
  PATHS "$ENV{PROGRAMFILES}/CoinUtils" "$ENV{PROGRAMW6432}/CoinUtils" "/usr" "/usr/local"
  PATH_SUFFIXES CoinUtils
  DOC "Root directory of COINUTILS includes")

##====================================================
## Include COINUTILS library
##----------------------------------------------------
if(EXISTS "${COINUTILS_DIR}" AND NOT "${COINUTILS_DIR}" STREQUAL "")
  set(COINUTILS_FOUND TRUE)
  set(COINUTILS_INCLUDE_DIRS ${COINUTILS_DIR})
  set(COINUTILS_DIR "${COINUTILS_DIR}" CACHE PATH "" FORCE)
  mark_as_advanced(COINUTILS_DIR)

  # Extract CoinUtils version from CoinUtilsConfig.h
  set(COINUTILS_VERSION_FILE ${COINUTILS_INCLUDE_DIRS}/CoinUtilsConfig.h)
  # Extract CoinUtils version from alternative config_coinutils_default.h
  if(EXISTS ${COINUTILS_INCLUDE_DIRS}/config_coinutils_default.h)
    set(COINUTILS_VERSION_FILE ${COINUTILS_INCLUDE_DIRS}/config_coinutils_default.h)
  endif()
  if(NOT EXISTS ${COINUTILS_VERSION_FILE})
    COINUTILS_REPORT_NOT_FOUND(
      "Could not find file: ${COINUTILS_VERSION_FILE} "
      "containing version information in CoinUtils install located at: "
      "${COINUTILS_INCLUDE_DIRS}.")
  else (NOT EXISTS ${COINUTILS_VERSION_FILE})
    file(READ ${COINUTILS_VERSION_FILE} COINUTILS_VERSION_FILE_CONTENTS)
    string(REGEX MATCH "#define COINUTILS_VERSION_MAJOR [0-9]+"
      COINUTILS_VERSION_MAJOR "${COINUTILS_VERSION_FILE_CONTENTS}")
    string(REGEX REPLACE "#define COINUTILS_VERSION_MAJOR ([0-9]+)" "\\1"
      COINUTILS_VERSION_MAJOR "${COINUTILS_VERSION_MAJOR}")
    string(REGEX MATCH "#define COINUTILS_VERSION_MINOR [0-9]+"
      COINUTILS_VERSION_MINOR "${COINUTILS_VERSION_FILE_CONTENTS}")
    string(REGEX REPLACE "#define COINUTILS_VERSION_MINOR ([0-9]+)" "\\1"
      COINUTILS_VERSION_MINOR "${COINUTILS_VERSION_MINOR}")
    string(REGEX MATCH "#define COINUTILS_VERSION_RELEASE [0-9]+"
      COINUTILS_VERSION_RELEASE "${COINUTILS_VERSION_FILE_CONTENTS}")
    string(REGEX REPLACE "#define COINUTILS_VERSION_RELEASE ([0-9]+)" "\\1"
      COINUTILS_VERSION_RELEASE "${COINUTILS_VERSION_RELEASE}")
    set(COINUTILS_VERSION "${COINUTILS_VERSION_MAJOR}.${COINUTILS_VERSION_MINOR}.${COINUTILS_VERSION_RELEASE}")
  endif(NOT EXISTS ${COINUTILS_VERSION_FILE})
  set(COINUTILS_INCLUDE_DIR ${COINUTILS_DIR})

  find_library(COINUTILS_LIBRARY NAMES CoinUtils)

  # locate CoinUtils libraries
  if(DEFINED COINUTILS_LIBRARY)
    set(COINUTILS_LIBRARIES ${COINUTILS_LIBRARY})
  endif()

  message(STATUS "CoinUtils ${COINUTILS_VERSION} found (include: ${COINUTILS_INCLUDE_DIRS})")
else()
  message(FATAL_ERROR "You are attempting to build without CoinUtils. "
          "Please use cmake variable -DCOINUTILS_INCLUDE_DIR_HINTS:STRING=\"PATH\" "
          "or COINUTILS_INCLUDE_DIR_HINTS env. variable to a valid CoinUtils path. "
          "Or install last CoinUtils version.")
  package_report_not_found(COINUTILS "CoinUtils cannot be found")
endif()
##====================================================
