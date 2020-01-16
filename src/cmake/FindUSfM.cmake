# Locate the USfM libraries.
#
# Defines the following variables:
#
#   USFM_FOUND        - TRUE if the USfM headers and libs are found
#   USFM_INCLUDE_DIRS - The path to USfM headers
#
#   USFM_LIBRARY      - The opengv library
#   USFM_LIBRARY_DIR  - The directory where the libraries are located
#
# Accepts the following variables as input:
#
#   USFM_DIR - (as a CMake or environment variable)
#                The root directory of the USfM install prefix

MESSAGE(STATUS "Looking for USfM.")

FIND_PATH(USFM_INCLUDE_DIR USfM/usfm_Algorithm.hpp
  HINTS
    $ENV{USFM_DIR}
    ${USFM_DIR}
  PATH_SUFFIXES
    include
)

FIND_LIBRARY(USFM_LIBRARY NAMES usfm
  HINTS
    $ENV{USFM_DIR}
    ${USFM_DIR}
  PATH_SUFFIXES
    lib
)

IF(USFM_INCLUDE_DIR)
  MESSAGE(STATUS "USfM headers found in ${USFM_INCLUDE_DIR}")
  IF(NOT CERES_FOUND)
    MESSAGE(STATUS "Looking for CERES dependency...")
    FIND_PACKAGE(CERES)
    IF(CERES_FOUND)
      set(USFM_INCLUDE_DIR ${USFM_INCLUDE_DIR} ${CERES_INCLUDE_DIRS})
      set(USFM_LIBRARY ${USFM_LIBRARY} ${CERES_LIBRARIES})
    ELSE()
        MESSAGE(WARNING "Couldn't find Ceres, this is needed for compiling with USfM")
      # this is to make the find_package_handle_standard_args  fail
      SET(USFM_INCLUDE_DIR "USFM_INCLUDE_DIR-NOTFOUND")
    ENDIF(CERES_FOUND)
  ELSE(NOT CERES_FOUND)
    MESSAGE(STATUS "Ceres already found")
  ENDIF(NOT CERES_FOUND)
ELSE(USFM_INCLUDE_DIR)
  MESSAGE(STATUS "USfM headers not found!")
ENDIF(USFM_INCLUDE_DIR)

GET_FILENAME_COMPONENT(USFM_LIBRARY_DIR "${USFM_LIBRARY}" PATH)

if(USFM_INCLUDE_DIR)
  message(STATUS "USfM include directory: ${USFM_INCLUDE_DIR}")
else()
  message(STATUS "USfM library include not found")
endif()

if(USFM_LIBRARY)
  message(STATUS "USfM libraries found: ${USFM_LIBRARY}")
  message(STATUS "USfM libraries directories: ${USFM_LIBRARY_DIR}")
else()
  message(STATUS "USfM library not found")
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set USFM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(USfM  DEFAULT_MSG
                                  USFM_LIBRARY USFM_INCLUDE_DIR)

# MARK_AS_ADVANCED(USFM_INCLUDE_DIR USFM_LIBRARY)
