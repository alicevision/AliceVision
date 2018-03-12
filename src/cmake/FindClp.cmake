###########################################################
#                  Find Clp Library
#----------------------------------------------------------

find_path(CLP_DIR ClpConfig.h
    HINTS "${CLP_ROOT}" "$ENV{CLP_ROOT}" "${CLP_INCLUDE_DIR_HINTS}"
    PATHS "$ENV{PROGRAMFILES}/Clp" "$ENV{PROGRAMW6432}/Clp" "/usr" "/usr/local"
    PATH_SUFFIXES Clp
    DOC "Root directory of CLP includes")

##====================================================
## Include CLP library
##----------------------------------------------------
if(EXISTS "${CLP_DIR}" AND NOT "${CLP_DIR}" STREQUAL "")
  set(CLP_FOUND TRUE)
  set(CLP_INCLUDE_DIRS ${CLP_DIR})
  set(CLP_DIR "${CLP_DIR}" CACHE PATH "" FORCE)
  mark_as_advanced(CLP_DIR)

  # Extract Clp version from ClpConfig.h
  set(CLP_VERSION_FILE ${CLP_INCLUDE_DIRS}/ClpConfig.h)
  # Extract Clp version from alternative config_clp_default.h
  if (EXISTS ${CLP_INCLUDE_DIRS}/config_clp_default.h)
    set(CLP_VERSION_FILE ${CLP_INCLUDE_DIRS}/config_clp_default.h)
  endif()
  if(NOT EXISTS ${CLP_VERSION_FILE})
    CLP_REPORT_NOT_FOUND(
      "Could not find file: ${CLP_VERSION_FILE} "
      "containing version information in Clp install located at: "
      "${CLP_INCLUDE_DIRS}.")
  else (NOT EXISTS ${CLP_VERSION_FILE})
      file(READ ${CLP_VERSION_FILE} CLP_VERSION_FILE_CONTENTS)
      string(REGEX MATCH "#define CLP_VERSION_MAJOR [0-9]+"
        CLP_VERSION_MAJOR "${CLP_VERSION_FILE_CONTENTS}")
      string(REGEX REPLACE "#define CLP_VERSION_MAJOR ([0-9]+)" "\\1"
        CLP_VERSION_MAJOR "${CLP_VERSION_MAJOR}")
      string(REGEX MATCH "#define CLP_VERSION_MINOR [0-9]+"
        CLP_VERSION_MINOR "${CLP_VERSION_FILE_CONTENTS}")
      string(REGEX REPLACE "#define CLP_VERSION_MINOR ([0-9]+)" "\\1"
        CLP_VERSION_MINOR "${CLP_VERSION_MINOR}")
      string(REGEX MATCH "#define CLP_VERSION_RELEASE [0-9]+"
        CLP_VERSION_RELEASE "${CLP_VERSION_FILE_CONTENTS}")
      string(REGEX REPLACE "#define CLP_VERSION_RELEASE ([0-9]+)" "\\1"
        CLP_VERSION_RELEASE "${CLP_VERSION_RELEASE}")
      set(CLP_VERSION "${CLP_VERSION_MAJOR}.${CLP_VERSION_MINOR}.${CLP_VERSION_RELEASE}")
  endif (NOT EXISTS ${CLP_VERSION_FILE})
  set(CLP_INCLUDE_DIR ${CLP_DIR})

  find_library(CLP_LIBRARY NAMES Clp)
  find_library(CLPSOLVER_LIBRARY NAMES ClpSolver)
  find_library(OSICLP_LIBRARY NAMES OsiClp)

  # locate Clp libraries
  if(DEFINED CLP_LIBRARY AND DEFINED CLPSOLVER_LIBRARY AND DEFINED OSICLP_LIBRARY)
    set(CLP_LIBRARIES ${CLP_LIBRARY} ${CLPSOLVER_LIBRARY} ${OSICLP_LIBRARY})
  endif()

  message(STATUS "Clp ${CLP_VERSION} found (include: ${CLP_INCLUDE_DIRS})")
else()
  message(FATAL_ERROR "You are attempting to build without Clp. "
          "Please use cmake variable -DCLP_INCLUDE_DIR_HINTS:STRING=\"PATH\" "
          "or CLP_INCLUDE_DIR_HINTS env. variable to a valid Clp path. "
          "Or install last Clp version.")
  package_report_not_found(CLP "Clp cannot be found")
endif()
##====================================================
