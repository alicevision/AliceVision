project(stlplus C CXX)

file(GLOB LIBSLTPLUS_HPP_FILESYSTEM "${CMAKE_CURRENT_SOURCE_DIR}/filesystemSimplified/*.hpp" )
file(GLOB LIBSLTPLUS_CPP_FILESYSTEM "${CMAKE_CURRENT_SOURCE_DIR}/filesystemSimplified/*.cpp" )

list(APPEND LIBSLTPLUS_SRCS
  ${LIBSLTPLUS_HPP_FILESYSTEM} ${LIBSLTPLUS_CPP_FILESYSTEM})

include_directories(./filesystemSimplified)
add_library(stlplus ${LIBSLTPLUS_SRCS})

set_property(TARGET stlplus
  PROPERTY FOLDER AliceVision/3rdParty
)

install(
  TARGETS stlplus
  DESTINATION lib
  EXPORT openMVG-targets
)

if(STLPLUS_INCLUDE_INSTALL_DIR)
  set(INCLUDE_INSTALL_DIR ${STLPLUS_INCLUDE_INSTALL_DIR})
else()
  set(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include/stlplus")
endif()

install(
  DIRECTORY ./filesystemSimplified
  DESTINATION ${INCLUDE_INSTALL_DIR}
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp"
)

