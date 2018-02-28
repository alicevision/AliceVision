# Locate the UncertaintyTE libraries.
#
# Defines the following variables:
#
#   UNCERTAINTYTE_FOUND        - TRUE if the UncertaintyTE headers and libs are found
#   UNCERTAINTYTE_INCLUDE_DIRS - The path to UncertaintyTE headers
#
#   UNCERTAINTYTE_LIBRARY      - The opengv library
#   UNCERTAINTYTE_LIBRARY_DIR  - The directory where the libraries are located
#
# Accepts the following variables as input:
#
#   UNCERTAINTYTE_DIR - (as a CMake or environment variable)
#                The root directory of the UncertaintyTE install prefix

MESSAGE(STATUS "Looking for UncertaintyTE.")

FIND_PATH(UNCERTAINTYTE_INCLUDE_DIR uncertaintyTE/uncertainty.h
  HINTS
    $ENV{UNCERTAINTYTE_DIR}
    ${UNCERTAINTYTE_DIR}
  PATH_SUFFIXES
    include
)

FIND_LIBRARY(UNCERTAINTYTE_LIBRARY NAMES uncertaintyTE
  HINTS
    $ENV{UNCERTAINTYTE_DIR}
    ${UNCERTAINTYTE_DIR}
  PATH_SUFFIXES
    lib
)

IF(UNCERTAINTYTE_INCLUDE_DIR)
  MESSAGE(STATUS "UncertaintyTE headers found in ${UNCERTAINTYTE_INCLUDE_DIR}")
  IF(NOT MAGMA_FOUND)
    MESSAGE(STATUS "Looking for MAGMA dependency...")
    FIND_PACKAGE(MAGMA)
    IF(MAGMA_FOUND)
      set(UNCERTAINTYTE_INCLUDE_DIR ${UNCERTAINTYTE_INCLUDE_DIR} ${MAGMA_INCLUDE_DIRS})
      set(UNCERTAINTYTE_LIBRARY ${UNCERTAINTYTE_LIBRARY} ${MAGMA_LIBRARIES})
    ELSE()
      MESSAGE(WARNING "Couldn't find Magma, this is needed for compiling with UncertaintyTE")
      # this is to make the find_package_handle_standard_args  fail
      SET(UNCERTAINTYTE_INCLUDE_DIR "UNCERTAINTYTE_INCLUDE_DIR-NOTFOUND")
    ENDIF(MAGMA_FOUND)
  ELSE(NOT MAGMA_FOUND)
    MESSAGE(STATUS "Magma already found")
  ENDIF(NOT MAGMA_FOUND)
ELSE(UNCERTAINTYTE_INCLUDE_DIR)
  MESSAGE(STATUS "UncertaintyTE headers not found!")
ENDIF(UNCERTAINTYTE_INCLUDE_DIR)


GET_FILENAME_COMPONENT(UNCERTAINTYTE_LIBRARY_DIR "${UNCERTAINTYTE_LIBRARY}" PATH)

if(UNCERTAINTYTE_INCLUDE_DIR)
  message(STATUS "UncertaintyTE include directory: ${UNCERTAINTYTE_INCLUDE_DIR}")
else()
  message(STATUS "UncertaintyTE library include not found")
endif()

if(UNCERTAINTYTE_LIBRARY)
  message(STATUS "UncertaintyTE libraries found: ${UNCERTAINTYTE_LIBRARY}")
  message(STATUS "UncertaintyTE libraries directories: ${UNCERTAINTYTE_LIBRARY_DIR}")
else()
  message(STATUS "UncertaintyTE library not found")
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set UNCERTAINTYTE_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(UncertaintyTE  DEFAULT_MSG
                                  UNCERTAINTYTE_LIBRARY UNCERTAINTYTE_INCLUDE_DIR)

# MARK_AS_ADVANCED(UNCERTAINTYTE_INCLUDE_DIR UNCERTAINTYTE_LIBRARY)
