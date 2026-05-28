# This file re-creates the symbolic links AliceVision creates in the generated
# Darwin bundle
#
# Inputs:
# - AV_BUNDLE_BINARY_DIR: The bin folder in the bundle to search for
#   executables

if(NOT DEFINED AV_BUNDLE_BINARY_DIR)
  message(FATAL_ERROR "Cannot re-create symbolic links in binary folder without AV_BUNDLE_BINARY_DIR being defined!")
endif()

# All files in the folder
file(GLOB BUNDLE_BINARIES LIST_DIRECTORIES OFF "${AV_BUNDLE_BINARY_DIR}/*")

foreach(BUNDLE_BINARY ${BUNDLE_BINARIES})
  if(BUNDLE_BINARY MATCHES "-[0-9]+_[0-9]+$")
    get_filename_component(BINARY_NAME "${BUNDLE_BINARY}" NAME)
    string(REGEX REPLACE "-[0-9]+_[0-9]+$" "" LINK_NAME "${BINARY_NAME}")
    file(CREATE_LINK "${BINARY_NAME}" "${AV_BUNDLE_BINARY_DIR}/${LINK_NAME}" SYMBOLIC)
    message(STATUS "Recreated symlink for ${BINARY_NAME}")
  endif()
endforeach()
