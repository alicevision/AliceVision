project(easyexif CXX)

cmake_minimum_required(VERSION 2.8)

add_library(easyexif exif.h exif.cpp)

set_property(TARGET easyexif
  PROPERTY FOLDER AliceVision/3rdParty
)

if(EASYEXIF_INCLUDE_INSTALL_DIR)
  set(INCLUDE_INSTALL_DIR ${EASYEXIF_INCLUDE_INSTALL_DIR})
else()
  set(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include/easyexif")
endif()

install(
  DIRECTORY .
  DESTINATION ${INCLUDE_INSTALL_DIR}
  COMPONENT headers
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS easyexif
  DESTINATION lib
  EXPORT openMVG-targets
)

