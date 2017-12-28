# Copyright (c) 2012, 2013 aliceVision authors.

# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. if a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.3)

project(aliceVision C CXX)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed.")
endif()

# Trilean option
function(trilean_option NAME DESCRIPTION DEFAULT_VALUE)
  set(${NAME} ${DEFAULT_VALUE} CACHE STRING ${DESCRIPTION})
  set(TRILEAN_VALUES "AUTO;ON;OFF")

  set_property(CACHE ${NAME} PROPERTY STRINGS "${TRILEAN_VALUES}")
  if("${${NAME}}" IN_LIST TRILEAN_VALUES)
    message(STATUS "** ${NAME}: '${${NAME}}'")
  else()
    message(FATAL_ERROR "A trilean option only accept the values: '${TRILEAN_VALUES}'")
  endif()
endfunction()

# ==============================================================================
# AliceVision build options
# ==============================================================================
option(ALICEVISION_BUILD_SHARED "Build AliceVision shared libs" OFF)
option(ALICEVISION_BUILD_TESTS "Build AliceVision tests" OFF)
option(ALICEVISION_BUILD_EXAMPLES "Build AliceVision samples applications." ON)
option(ALICEVISION_BUILD_COVERAGE "Enable code coverage generation (gcc only)" OFF)

trilean_option(ALICEVISION_BUILD_DOC "Build AliceVision documentation" AUTO)
trilean_option(ALICEVISION_USE_OPENMP "Enable OpenMP parallelization" AUTO)
trilean_option(ALICEVISION_USE_CCTAG "Enable CCTAG markers" AUTO)
trilean_option(ALICEVISION_USE_POPSIFT "Enable GPU SIFT implementation" AUTO)
trilean_option(ALICEVISION_USE_OPENGV "Enable use of OpenGV algorithms" AUTO)
trilean_option(ALICEVISION_USE_ALEMBIC "Enable Alembic I/O" AUTO)

option(ALICEVISION_USE_LOG "Enable logger" ON)

if(ALICEVISION_USE_LOG)
  set(ALICEVISION_WITH_COUT 1)
else()
  set(ALICEVISION_WITH_COUT 0)
endif()

option(ALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE ON)

trilean_option(ALICEVISION_USE_OPENCV "Build opencv+aliceVision samples programs" OFF)

# Since OpenCV 3, SIFT is no longer in the default modules. See
# https://github.com/itseez/opencv_contrib for more informations.
# Enable this to be able to use OpenCV SIFT in main_ComputeFeatures_OpenCV.
option(ALICEVISION_USE_OCVSIFT "Add or not OpenCV SIFT in available features" OFF)

# Default build is in Release mode
if(NOT CMAKE_BUILD_TYPE AND NOT MSVC)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Set build path
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_BUILD_TYPE}")
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_BUILD_TYPE}")

if(WIN32)
  add_definitions(-DNOMINMAX)
  add_definitions(-D_USE_MATH_DEFINES)
  if(MSVC)
    add_definitions(/bigobj)
    add_compile_options(/MP)
  endif()
endif()

# Set shared or static mode
if(ALICEVISION_BUILD_SHARED)
  set(BUILD_SHARED_LIBS ON)
elseif()
  set(BUILD_SHARED_LIBS OFF)
endif()

# ==============================================================================
# Enable cmake UNIT TEST framework
# ==============================================================================
if(ALICEVISION_BUILD_TESTS)
  enable_testing()
endif()

# MACRO to ease UnitTesting
macro(UNIT_TEST NAMESPACE NAME EXTRA_LIBS)
  if(ALICEVISION_BUILD_TESTS)
    add_executable(${NAMESPACE}_test_${NAME} ${NAME}_test.cpp)

    set_property(TARGET ${NAMESPACE}_test_${NAME} PROPERTY FOLDER AliceVision/test)

    target_link_libraries(${NAMESPACE}_test_${NAME}
                          ${EXTRA_LIBS} # Extra libs MUST be first.
                          ${BOOST_LIBRARIES} ${ALICEVISION_LIBRARY_DEPENDENCIES})
    add_test(NAME ${NAMESPACE}_test_${NAME}
             WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:${NAMESPACE}_test_${NAME}> --log_level=all)
  endif()
endmacro()

# ==============================================================================
# Declare src
# ==============================================================================
add_subdirectory(src)
