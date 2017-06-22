###########################################################
#                  Find Osi Library
#----------------------------------------------------------

find_path(OSI_DIR OsiConfig.h
    HINTS "${OSI_ROOT}" "$ENV{OSI_ROOT}" "${OSI_INCLUDE_DIR_HINTS}"
    PATHS "$ENV{PROGRAMFILES}/Osi" "$ENV{PROGRAMW6432}/Osi" "/usr" "/usr/local"
    PATH_SUFFIXES Osi
    DOC "Root directory of OSI includes")

##====================================================
## Include OSI library
##----------------------------------------------------
if(EXISTS "${OSI_DIR}" AND NOT "${OSI_DIR}" STREQUAL "")
  set(OSI_FOUND TRUE)
  set(OSI_INCLUDE_DIRS ${OSI_DIR})
  set(OSI_DIR "${OSI_DIR}" CACHE PATH "" FORCE)
  mark_as_advanced(OSI_DIR)

  # Extract Osi version from OsiConfig.h
  set(OSI_VERSION_FILE ${OSI_INCLUDE_DIRS}/OsiConfig.h)
  # Extract Osi version from alternative config_osi_default.h
  if(EXISTS ${OSI_INCLUDE_DIRS}/config_osi_default.h)
    set(OSI_VERSION_FILE ${OSI_INCLUDE_DIRS}/config_osi_default.h)
  endif()
  if(NOT EXISTS ${OSI_VERSION_FILE})
    OSI_REPORT_NOT_FOUND(
      "Could not find file: ${OSI_VERSION_FILE} "
      "containing version information in Osi install located at: "
      "${OSI_INCLUDE_DIRS}.")
  else(NOT EXISTS ${OSI_VERSION_FILE})
      file(READ ${OSI_VERSION_FILE} OSI_VERSION_FILE_CONTENTS)
      string(REGEX MATCH "#define OSI_VERSION_MAJOR [0-9]+"
        OSI_VERSION_MAJOR "${OSI_VERSION_FILE_CONTENTS}")
      string(REGEX REPLACE "#define OSI_VERSION_MAJOR ([0-9]+)" "\\1"
        OSI_VERSION_MAJOR "${OSI_VERSION_MAJOR}")
      string(REGEX MATCH "#define OSI_VERSION_MINOR [0-9]+"
        OSI_VERSION_MINOR "${OSI_VERSION_FILE_CONTENTS}")
      string(REGEX REPLACE "#define OSI_VERSION_MINOR ([0-9]+)" "\\1"
        OSI_VERSION_MINOR "${OSI_VERSION_MINOR}")
      string(REGEX MATCH "#define OSI_VERSION_RELEASE [0-9]+"
        OSI_VERSION_RELEASE "${OSI_VERSION_FILE_CONTENTS}")
      string(REGEX REPLACE "#define OSI_VERSION_RELEASE ([0-9]+)" "\\1"
        OSI_VERSION_RELEASE "${OSI_VERSION_RELEASE}")
      set(OSI_VERSION "${OSI_VERSION_MAJOR}.${OSI_VERSION_MINOR}.${OSI_VERSION_RELEASE}")
  endif(NOT EXISTS ${OSI_VERSION_FILE})
  set(OSI_INCLUDE_DIR ${OSI_DIR})

  find_library(OSI_LIBRARY NAMES Osi)

  # locate Osi libraries
  if(DEFINED OSI_LIBRARY)
    set(OSI_LIBRARIES ${OSI_LIBRARY})
  endif()

  message(STATUS "Osi ${OSI_VERSION} found (include: ${OSI_INCLUDE_DIRS})")
else()
  message(FATAL_ERROR "You are attempting to build without Osi. "
    "Please use cmake variable -DOSI_INCLUDE_DIR_HINTS:STRING=\"PATH\" "
    "or OSI_INCLUDE_DIR_HINTS env. variable to a valid Osi path. "
    "Or install last Osi version.")
  package_report_not_found(OSI "Osi cannot be found")
endif()
##====================================================
