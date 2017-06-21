# Copyright (c) 2012, 2013 openMVG authors.

# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. if a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.3)

project(openMVG C CXX)

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
    message(STATUS "> ${NAME}: '${${NAME}}'")
  else()
    message(FATAL_ERROR "A trilean option only accept the values: '${TRILEAN_VALUES}'")
  endif()
endfunction()

# ==============================================================================
# OpenMVG build options
# ==============================================================================
option(OPENMVG_BUILD_SHARED "Build OpenMVG shared libs" OFF)
option(OPENMVG_BUILD_TESTS "Build OpenMVG tests" OFF)
option(OPENMVG_BUILD_EXAMPLES "Build OpenMVG samples applications." ON)
option(OPENMVG_BUILD_OPENGL_EXAMPLES "Build OpenMVG openGL examples" OFF)
option(OPENMVG_BUILD_COVERAGE "Enable code coverage generation (gcc only)" OFF)

trilean_option(OPENMVG_BUILD_DOC "Build OpenMVG documentation" ON)
trilean_option(OPENMVG_USE_BOOST "Build code depending on Boost (localization/voctree)" ON)
trilean_option(OPENMVG_USE_OPENMP "Enable OpenMP parallelization" AUTO)
trilean_option(OPENMVG_USE_CCTAG "Enable CCTAG markers" AUTO)
trilean_option(OPENMVG_USE_POPSIFT "Enable GPU SIFT implementation" AUTO)
trilean_option(OPENMVG_USE_OPENGV "Enable use of OpenGV algorithms" AUTO)
trilean_option(OPENMVG_USE_ALEMBIC "Enable Alembic I/O" AUTO)

option(OPENMVG_USE_LOG "Enable logger" ON)

if(OPENMVG_USE_LOG)
  set(OPENMVG_WITH_COUT 1)
else()
  set(OPENMVG_WITH_COUT 0)
endif()

option(OPENMVG_REQUIRE_CERES_WITH_SUITESPARSE ON)

# ==============================================================================
# Opencv is not used by openMVG but some samples show how to use openCV
#  and openMVG simultaneously
# ==============================================================================
trilean_option(OPENMVG_USE_OPENCV "Build or not opencv+openMVG samples programs" OFF)

# ==============================================================================
# Since OpenCV 3, SIFT is no longer in the default modules. See
# https://github.com/itseez/opencv_contrib for more informations.
# Enable this to be able to use OpenCV SIFT in main_ComputeFeatures_OpenCV.
# ==============================================================================
option(OPENMVG_USE_OCVSIFT "Add or not OpenCV SIFT in available features" OFF)

# ==============================================================================
# OpenMVG version
# ==============================================================================
file(STRINGS "openMVG/version.hpp" _OPENMVG_VERSION_HPP_CONTENTS REGEX "#define OPENMVG_VERSION_")
foreach(v MAJOR MINOR REVISION)
  if("${_OPENMVG_VERSION_HPP_CONTENTS}" MATCHES "#define OPENMVG_VERSION_${v} ([0-9]+)")
    set(OPENMVG_VERSION_${v} "${CMAKE_MATCH_1}")
  else()
    message(FATAL_ERROR "Failed to retrieve the OpenMVG version the source code. Missing OPENMVG_VERSION_${v}.")
  endif()
endforeach()
set(OPENMVG_VERSION ${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}.${OPENMVG_VERSION_REVISION})

# Default build is in Release mode
if(NOT CMAKE_BUILD_TYPE AND NOT MSVC)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Set build path
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_BUILD_TYPE}")
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_BUILD_TYPE}")

if(WIN32)
  add_definitions(-DNOMINMAX)
  if(MSVC)
    add_definitions(/bigobj)
    add_compile_options(/MP)
  endif()
endif()

# Set shared or static mode
if(OPENMVG_BUILD_SHARED)
  set(BUILD_SHARED_LIBS ON)
elseif()
  set(BUILD_SHARED_LIBS OFF)
endif()


# ==============================================================================
# Check that submodule have been initialized and updated
# ==============================================================================
if(NOT EXISTS ${PROJECT_SOURCE_DIR}/dependencies/cereal/include)
  message(FATAL_ERROR
    "\n submodule(s) are missing, please update your repository:\n"
    "  > git submodule update -i\n")
endif()

# ==============================================================================
# Additional cmake find modules
# ==============================================================================
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmakeFindModules)
include(OptimizeForArchitecture)
OptimizeForArchitecture()
set(OPENMVG_HAVE_SSE 0)
if(SSE2_FOUND OR TARGET_ARCHITECTURE STREQUAL "native")
  if(MSVC AND NOT ${CMAKE_CL_64})
    add_definitions(/arch:SSE2)
  endif()
  set(OPENMVG_HAVE_SSE 1)
endif()
if(UNIX AND NOT OPENMVG_BUILD_COVERAGE)
  set(CMAKE_C_FLAGS_RELEASE "-O3")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  include(AddCompilerFlag)

  # This flag is useful as not returning from a non-void function is an error with MSVC
  AddCompilerFlag("-Werror=return-type")
  AddCompilerFlag("-Werror=switch")
  AddCompilerFlag("-Werror=return-local-addr")
endif()

# ==============================================================================
# Check C++11 support
# ==============================================================================
include(CXX11)
check_for_cxx11_compiler(CXX11_COMPILER)

if(NOT CXX11_COMPILER)
  message(FATAL_ERROR "The compiler does not support the CXX11 standard.")
endif(NOT CXX11_COMPILER)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# OpenMP detection
# ==============================================================================
if(OPENMVG_USE_OPENMP STREQUAL "OFF")
  set(OPENMVG_HAVE_OPENMP 0)
else() # ON OR AUTO
  find_package(OpenMP)

  if(OPENMP_FOUND)
    set(OPENMVG_HAVE_OPENMP 1)
    message(STATUS "OpenMP found.")
  elseif(OPENMVG_USE_OPENMP STREQUAL "ON")
    set(OPENMVG_HAVE_OPENMP 0)
    message(SEND_ERROR "Failed to find OpenMP.")
  else()
    set(OPENMVG_HAVE_OPENMP 0)
  endif()
endif()

if(OPENMVG_HAVE_OPENMP)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  if(NOT MSVC)
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      # for those using the clang with OpenMP support
      list(APPEND OPENMVG_LIBRARY_DEPENDENCIES iomp)
    else()
    list(APPEND OPENMVG_LIBRARY_DEPENDENCIES gomp)
    endif()
  endif()
endif()

# ==============================================================================
# enable code coverage generation (only with GCC)
# ==============================================================================
if(OPENMVG_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
  message("OPENMVG_BUILD_COVERAGE enabled")
  set(CMAKE_BUILD_TYPE "Debug")
  add_definitions(--coverage -fprofile-arcs -ftest-coverage)
  set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage")
endif()

# ==============================================================================
# IMAGE IO detection
# ==============================================================================
if(NOT APPLE)
  find_package(JPEG QUIET)
  find_package(PNG QUIET)
  find_package(TIFF QUIET)
endif()

# Folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ==============================================================================
# SUBMODULE CONFIGURATION
# ==============================================================================
#- glfw
# ==============================================================================
if(OPENMVG_BUILD_OPENGL_EXAMPLES)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Do not build the GLFW example programs")
  set(GLFW_BUILD_TESTS    OFF CACHE BOOL "Do not build the GLFW tests programs")
  set(GLFW_BUILD_DOCS     OFF CACHE BOOL "Do not build the GLFW documentation")
  set(GLFW_INSTALL        OFF CACHE BOOL "Do not generate the GLFW installation target")
  add_subdirectory(dependencies/glfw)
  include_directories(SYSTEM dependencies/glfw/include)
  set_property(TARGET glfw PROPERTY FOLDER OpenMVG/3rdParty/glfw)
endif()

#- Mosek (linear programming interface)
#===============================================================================
set(OPENMVG_HAVE_MOSEK 0)
find_package(Mosek)
if(MOSEK_FOUND)
  set(OPENMVG_HAVE_MOSEK 1)
  set(LP_INCLUDE_DIRS
    ${MOSEK_INCLUDE}
    ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/Osi/src/OsiMsk/
  )
  include_directories(${LP_INCLUDE_DIRS})
endif()

#Install RULES
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/
  DESTINATION include/openMVG_dependencies
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
# ==============================================================================
# --END-- SUBMODULE CONFIGURATION
# ==============================================================================

# ==============================================================================
# Enable cmake UNIT TEST framework
# ==============================================================================
if(OPENMVG_BUILD_TESTS)
  enable_testing()
endif()

# MACRO to ease UnitTesting
macro(UNIT_TEST NAMESPACE NAME EXTRA_LIBS)
  if(OPENMVG_BUILD_TESTS)
    add_executable(${NAMESPACE}_test_${NAME} ${NAME}_test.cpp)

    set_property(TARGET ${NAMESPACE}_test_${NAME} PROPERTY FOLDER OpenMVG/test)

    target_link_libraries(${NAMESPACE}_test_${NAME}
                          ${EXTRA_LIBS} # Extra libs MUST be first.
                          CppUnitLite ${OPENMVG_LIBRARY_DEPENDENCIES})
    add_test(NAME ${NAMESPACE}_test_${NAME} WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:${NAMESPACE}_test_${NAME}>)
  endif()
endmacro()

# ==============================================================================
# Eigen
# ==============================================================================
# - internal by default,
# - external if EIGEN_INCLUDE_DIR_HINTS is defined
# ==============================================================================
if(NOT DEFINED EIGEN_INCLUDE_DIR_HINTS)
  set(EIGEN_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
  set(OPENMVG_USE_INTERNAL_EIGEN ON)
endif()
find_package(Eigen QUIET)
if(EIGEN_FOUND)
  include_directories(${EIGEN_INCLUDE_DIRS})
endif()

# ==============================================================================
# Ceres
# ==============================================================================
# - internal by default (ceres-solver+cxsparse+miniglog),
# - external if Ceres_DIR and find_package return a valid Ceres setup
# ==============================================================================
message(STATUS "Trying to find package Ceres for openMVG: ${Ceres_DIR}")
if(OPENMVG_REQUIRE_CERES_WITH_SUITESPARSE)
  message(STATUS "By default, Ceres required SuiteSparse to ensure best performances. if you explicitly need to build without it, you can use the option: -DOPENMVG_REQUIRE_CERES_WITH_SUITESPARSE=OFF")
  find_package(Ceres QUIET REQUIRED SuiteSparse CONFIG)
else()
  find_package(Ceres CONFIG QUIET CONFIG)
endif()

if(NOT Ceres_FOUND)
  if(DEFINED OPENMVG_USE_INTERNAL_CERES AND NOT OPENMVG_USE_INTERNAL_CERES)
    message(FATAL_ERROR "External CERES not found. Not found in Ceres_DIR: ${Ceres_DIR}")
  endif()

  message(STATUS "No external CERES detected, using the embedded one.")
  set(OPENMVG_USE_INTERNAL_CERES ON)
  set(CERES_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver/internal/ceres/miniglog
    ${PROJECT_BINARY_DIR}/third_party/ceres-solver/config)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver/VERSION" CERES_CONFIG)
    string(REGEX REPLACE "version ([0-9.]+).*" "\\1" CERES_VERSION ${CERES_CONFIG})
  set(CERES_LIBRARIES ceres cxsparse)
  set(Ceres_FOUND TRUE)
else()
  message(STATUS "Ceres include dirs ${CERES_INCLUDE_DIRS}")
  message(STATUS "Ceres libraries ${CERES_LIBRARIES}")
  include_directories(${CERES_INCLUDE_DIRS})
  set(OPENMVG_USE_INTERNAL_CERES OFF)
endif()

# ==============================================================================
# Flann
# ==============================================================================
# - internal by default (flann),
# - external if FLANN_INCLUDE_DIR_HINTS and a valid Flann setup is found
# ==============================================================================
if(NOT DEFINED FLANN_INCLUDE_DIR_HINTS)
  set(FLANN_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/flann/src/cpp)
  set(OPENMVG_USE_INTERNAL_FLANN ON)
endif()
find_package(Flann QUIET)
if(NOT FLANN_FOUND OR OPENMVG_USE_INTERNAL_FLANN)
  set(FLANN_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/flann/src/cpp)
  set(FLANN_LIBRARY flann_cpp_s)
endif()

# ==============================================================================
# CoinUtils
# ==============================================================================
# - internal by default (CoinUtils),
# - external if COINUTILS_INCLUDE_DIR_HINTS and a valid CoinUtils setup is found
# ==============================================================================
if(NOT DEFINED COINUTILS_INCLUDE_DIR_HINTS)
  set(COINUTILS_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/CoinUtils/src/)
  set(OPENMVG_USE_INTERNAL_COINUTILS ON)
endif()
find_package(CoinUtils QUIET)
if(NOT COINUTILS_FOUND OR OPENMVG_USE_INTERNAL_COINUTILS)
  set(COINUTILS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/CoinUtils/src/)
  set(COINUTILS_LIBRARY lib_CoinUtils)
endif()

## ==============================================================================
## Clp
## ==============================================================================
## - internal by default (Clp),
## - external if CLP_INCLUDE_DIR_HINTS and a valid Clp setup is found
## ==============================================================================
if(NOT DEFINED CLP_INCLUDE_DIR_HINTS)
  set(CLP_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/Clp/src/)
  set(OPENMVG_USE_INTERNAL_CLP ON)
endif()
find_package(Clp QUIET)
if(NOT CLP_FOUND OR OPENMVG_USE_INTERNAL_CLP)
  set(CLP_INCLUDE_DIRS
     ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/Clp/src/
     ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/Clp/src/OsiClp/)
  set(CLP_LIBRARIES lib_clp lib_OsiClpSolver)
endif()

# ==============================================================================
# Osi
# ==============================================================================
# - internal by default (Osi),
# - external if OSI_INCLUDE_DIR_HINTS and a valid Osi setup is found
# ==============================================================================
if(NOT DEFINED OSI_INCLUDE_DIR_HINTS)
  set(OSI_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/Osi/src/)
  set(OPENMVG_USE_INTERNAL_OSI ON)
endif()
find_package(Osi QUIET)
if(NOT OSI_FOUND OR OPENMVG_USE_INTERNAL_OSI)
  set(OSI_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/Osi/src/Osi/)
  set(OSI_LIBRARY lib_Osi)
endif()

# ==============================================================================
# Internal CLP/OSI/COINUTILS libraries:
# ==============================================================================
if(OPENMVG_USE_INTERNAL_OSI AND OPENMVG_USE_INTERNAL_CLP AND OPENMVG_USE_INTERNAL_COINUTILS)
  add_subdirectory(dependencies/osi_clp/)
  set_property(TARGET lib_clp PROPERTY FOLDER OpenMVG/3rdParty/lib_clp)
  set_property(TARGET lib_CoinUtils PROPERTY FOLDER OpenMVG/3rdParty/lib_CoinUtils)
  set_property(TARGET lib_Osi PROPERTY FOLDER OpenMVG/3rdParty/lib_Osi)
  set_property(TARGET lib_OsiClpSolver PROPERTY FOLDER OpenMVG/3rdParty/lib_OsiClpSolver)
endif()

# ==============================================================================
# Lemon
# ==============================================================================
# - internal by default (Lemon),
# - external if LEMON_INCLUDE_DIR_HINTS and a valid Lemon setup is found
# ==============================================================================
if(NOT DEFINED LEMON_INCLUDE_DIR_HINTS)
  set(LEMON_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lemon)
  set(OPENMVG_USE_INTERNAL_LEMON ON)
endif()
find_package(Lemon QUIET)
if(NOT LEMON_FOUND OR OPENMVG_USE_INTERNAL_LEMON)
  set(LEMON_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lemon
    ${PROJECT_BINARY_DIR}/third_party/lemon)
  set(LEMON_LIBRARY lemon)
endif()

# ==============================================================================
# OpenCV
# ==============================================================================
# - only external and enabled only if OPENMVG_USE_OPENCV is set to ON
# ==============================================================================
if(OPENMVG_USE_OPENCV STREQUAL "OFF")
  set(OPENMVG_HAVE_OPENCV 0)
  set(OPENMVG_HAVE_OCVSIFT 0)
else() # ON OR AUTO
  find_package(OpenCV COMPONENTS core imgproc video imgcodecs videoio features2d xfeatures2d)

  if(OpenCV_FOUND)
    set(OPENMVG_HAVE_OPENCV 1)
    message(STATUS "OpenCV found.")
  elseif(OPENMVG_USE_OPENCV STREQUAL "ON")
    set(OPENMVG_HAVE_OPENCV 0)
    message(SEND_ERROR "Failed to find OpenCV.")
  else()
    set(OPENMVG_HAVE_OPENCV 0)
  endif()
endif()

if(OPENMVG_HAVE_OPENCV)
  include_directories(${OpenCV_INCLUDE_DIRS})
  # add a definition that allows the conditional compiling
  if(OPENMVG_USE_OCVSIFT)
    set(OPENMVG_HAVE_OCVSIFT 1)
  else()
    set(OPENMVG_HAVE_OCVSIFT 0)
  endif()
endif()

# ==============================================================================
# Boost
# ==============================================================================
# - optional, it allows to compile the vocabulary tree
# ==============================================================================
if(OPENMVG_USE_BOOST STREQUAL "OFF")
  set(OPENMVG_HAVE_BOOST 0)
else() # ON OR AUTO
  option(BOOST_NO_CXX11 "if Boost is compiled without C++11 support (as it is often the case in OS packages) this must be enabled to avoid symbol conflicts (SCOPED_ENUM)." OFF)
  find_package(Boost 1.53.0 QUIET COMPONENTS system filesystem program_options thread serialization log log_setup)

  if(Boost_FOUND)
    set(OPENMVG_HAVE_BOOST 1)
    message(STATUS "Boost ${Boost_LIB_VERSION} found.")
  elseif(OPENMVG_USE_BOOST STREQUAL "ON")
    set(OPENMVG_HAVE_BOOST 0)
    message(SEND_ERROR "Failed to find Boost.")
  else()
    set(OPENMVG_HAVE_BOOST 0)
  endif()
endif()

if(OPENMVG_HAVE_BOOST)
  if(BOOST_NO_CXX11)
    add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)
  endif()

  if(WIN32)
    add_definitions(-DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB)
  endif()

  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
  add_definitions(${Boost_DEFINITIONS})
  set(LOG_LIB ${Boost_LOG_LIBRARY} ${Boost_LOG_SETUP_LIBRARY})
endif()


# ==============================================================================
# Alembic
# ==============================================================================
# - optional, it allows to use the classes to export data in alembic format
# ==============================================================================
if(OPENMVG_USE_ALEMBIC STREQUAL "OFF")
  set(OPENMVG_HAVE_ALEMBIC 0)
else() # ON OR AUTO
  find_package(Alembic)

  if(ALEMBIC_FOUND)
    set(OPENMVG_HAVE_ALEMBIC 1)
    message(STATUS "Alembic found.")
  elseif(OPENMVG_USE_ALEMBIC STREQUAL "ON")
    set(OPENMVG_HAVE_ALEMBIC 0)
    message(SEND_ERROR "Failed to find Alembic.")
  else()
    set(OPENMVG_HAVE_ALEMBIC 0)
  endif()
endif()

if(OPENMVG_HAVE_ALEMBIC)
  include_directories(${ABC_INCLUDE_DIR})
endif()

# ==============================================================================
# Third-party libraries:
# ==============================================================================
add_subdirectory(third_party)

# ==============================================================================
# CCTag
# ==============================================================================
if(OPENMVG_USE_CCTAG STREQUAL "OFF")
  set(OPENMVG_HAVE_CCTAG 0)
else() # ON OR AUTO
  find_package(CCTag 1.0.0 CONFIG)

  if(CCTag_FOUND)
    set(OPENMVG_HAVE_CCTAG 1)
    message(STATUS "CCTAG ${CCTag_VERSION} found.")
  elseif(OPENMVG_USE_CCTAG STREQUAL "ON")
    set(OPENMVG_HAVE_CCTAG 0)
    message(SEND_ERROR "Failed to find CCTAG.")
  else()
    set(OPENMVG_HAVE_CCTAG 0)
  endif()
endif()

# ==============================================================================
# PopSift
# ==============================================================================
if(OPENMVG_USE_POPSIFT STREQUAL "OFF")
  set(OPENMVG_HAVE_POPSIFT 0)
else() # ON OR AUTO
  find_package(PopSift CONFIG)

  if(PopSift_FOUND)
    set(OPENMVG_HAVE_POPSIFT 1)
    message(STATUS "PopSIFT found.")
  elseif(OPENMVG_USE_POPSIFT STREQUAL "ON")
    set(OPENMVG_HAVE_POPSIFT 0)
    message(SEND_ERROR "Failed to find PopSIFT.")
  else()
    set(OPENMVG_HAVE_POPSIFT 0)
  endif()
endif()

# ==============================================================================
# OpenGV
# ==============================================================================
# - optional, it allows to use the generic camera PnP algorithms for rig localization
# ==============================================================================
if(OPENMVG_USE_OPENGV STREQUAL "OFF")
  set(OPENMVG_HAVE_OPENGV 0)
else() # ON OR AUTO
  find_package(OpenGV)

  if(OPENGV_FOUND)
    set(OPENMVG_HAVE_OPENGV 1)
    message(STATUS "OpenGV found.")
  elseif(OPENMVG_USE_OPENGV STREQUAL "ON")
    set(OPENMVG_HAVE_OPENGV 0)
    message(SEND_ERROR "Failed to find OpenGV.")
  else()
    set(OPENMVG_HAVE_OPENGV 0)
  endif()
endif()

if(OPENMVG_HAVE_OPENGV)
  include_directories(${OPENGV_INCLUDE_DIR})
  link_directories(${OPENGV_LIBRARY_DIR})
endif()

# ==============================================================================
# Include directories
# ==============================================================================
set(generatedDir "${CMAKE_CURRENT_BINARY_DIR}/generated")
message("generatedDir: ${generatedDir}")
set(OPENMVG_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ${generatedDir}
  ${JPEG_INCLUDE_DIR}
  ${PNG_INCLUDE_DIRS}
  ${TIFF_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  ${CMAKE_CURRENT_SOURCE_DIR}/dependencies
  ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/cereal/include
  ${LEMON_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
  ${LP_INCLUDE_DIRS}
  ${COINUTILS_INCLUDE_DIRS}
  ${CLP_INCLUDE_DIRS}
  ${OSI_INCLUDE_DIRS}
)

include_directories(${OPENMVG_INCLUDE_DIRS})

# ==============================================================================
# openMVG modules
# ==============================================================================
# The openMVG library itself
add_subdirectory(openMVG)

# ==============================================================================
# openMVG libraries
# ==============================================================================
set(OPENMVG_LIBRARIES
  openMVG_exif
  openMVG_features
  openMVG_image
#  openMVG_linearProgramming
  openMVG_lInftyComputerVision
  openMVG_matching
  openMVG_kvld
  openMVG_matching_image_collection
  openMVG_multiview
  openMVG_numeric
  openMVG_sfm
  openMVG_system)

if(OPENMVG_HAVE_BOOST)
  list(APPEND OPENMVG_LIBRARIES 
    openMVG_dataio 
    openMVG_voctree 
    openMVG_localization
    openMVG_rig)
  if(OPENMVG_HAVE_OPENCV)
    list(APPEND OPENMVG_LIBRARIES 
      openMVG_calibration)
  endif()
endif()

foreach(omvglib ${OPENMVG_LIBRARIES})
  set_property(TARGET ${omvglib} PROPERTY FOLDER OpenMVG/OpenMVG)
endforeach()

# openMVG tutorial examples
if(OPENMVG_BUILD_EXAMPLES)
  add_subdirectory(openMVG_Samples)
endif()

# Complete software(s) build on openMVG libraries
add_subdirectory(software)

# software(s) under patent or commercial licence
# Included for research purpose only
add_subdirectory(nonFree)

# ==============================================================================
# Documentation
# --------------------------
# Sphinx detection
# ==============================================================================
if(OPENMVG_BUILD_DOC STREQUAL "OFF")
  set(OPENMVG_HAVE_DOC 0)
else()
  find_package(Sphinx)

  if (EXISTS ${SPHINX_EXECUTABLE})
    set(SPHINX_HTML_DIR "${CMAKE_CURRENT_BINARY_DIR}/htmlDoc")

    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/../docs/sphinx/rst/conf.py"
      "${CMAKE_CURRENT_BINARY_DIR}/conf.py"
      @ONLY
    )

    add_custom_target(doc ALL
      ${SPHINX_EXECUTABLE}
      -b html
      "${CMAKE_CURRENT_SOURCE_DIR}/../docs/sphinx/rst"
      "${SPHINX_HTML_DIR}"
      COMMENT "Building HTML documentation with Sphinx")

    set_property(TARGET doc
      PROPERTY FOLDER OpenMVG
    )
    set(OPENMVG_HAVE_DOC 1)
    message(STATUS "Sphinx found.")
  elseif(OPENMVG_BUILD_DOC STREQUAL "ON")
    set(OPENMVG_HAVE_DOC 0)
    message(SEND_ERROR "Failed to find Sphinx.\nSphinx need to be installed to generate the documentation")
  else()
    set(OPENMVG_HAVE_DOC 0)
  endif()
endif()

message("\n")
message("** OpenMVG version: " ${OPENMVG_VERSION})
message("** Build Shared libs: " ${OPENMVG_BUILD_SHARED})
message("** Build OpenMVG tests: " ${OPENMVG_BUILD_TESTS})
message("** Build OpenMVG samples applications: " ${OPENMVG_BUILD_EXAMPLES})
message("** Build OpenMVG openGL examples: " ${OPENMVG_BUILD_OPENGL_EXAMPLES})
message("** Build OpenMVG documentation: " ${OPENMVG_HAVE_DOC})
message("** Build the voctree and localization module: " ${OPENMVG_HAVE_BOOST})
message("** Enable code coverage generation: " ${OPENMVG_BUILD_COVERAGE})
message("** Enable OpenMP parallelization: " ${OPENMVG_HAVE_OPENMP})
message("** Build OpenCV+OpenMVG samples programs: " ${OPENMVG_HAVE_OPENCV})
message("** Use OpenCV SIFT features: " ${OPENMVG_HAVE_OCVSIFT})
message("** Use CCTAG markers: " ${OPENMVG_HAVE_CCTAG})
message("** Build Alembic exporter: " ${OPENMVG_HAVE_ALEMBIC})
message("** Use OpenGV for rig localization: " ${OPENMVG_HAVE_OPENGV})
message("** Use log: " ${OPENMVG_USE_LOG})
message("\n")

if(OPENMVG_USE_INTERNAL_EIGEN)
  message(STATUS "EIGEN: " ${EIGEN_VERSION} " (internal)")
else()
  message(STATUS "EIGEN: " ${EIGEN_VERSION} " (external)")
endif()

if(OPENMVG_USE_INTERNAL_CERES)
  message(STATUS "CERES: " ${CERES_VERSION} " (internal)")
else()
  message(STATUS "CERES: " ${CERES_VERSION} " (external)")
endif()

if(OPENMVG_USE_INTERNAL_FLANN)
  message(STATUS "FLANN: " ${FLANN_VERSION} " (internal)")
else()
  message(STATUS "FLANN: " ${FLANN_VERSION} " (external)")
endif()

if(OPENMVG_USE_INTERNAL_TIFF)
  message(STATUS "LIBTIFF: " ${TIFF_VERSION_STRING} " (internal)")
else()
  message(STATUS "LIBTIFF: " ${TIFF_VERSION_STRING} " (external)")
endif()

if(OPENMVG_USE_INTERNAL_PNG)
  message(STATUS "LIBPNG: " ${PNG_VERSION_STRING} " (internal)")
else()
  message(STATUS "LIBPNG: " ${PNG_VERSION_STRING} " (external)")
endif()

if(OPENMVG_USE_INTERNAL_JPEG)
  message(STATUS "LIBJPEG (internal)")
else()
  message(STATUS "LIBJPEG (external)")
endif()

if(OPENMVG_USE_INTERNAL_CLP)
  message(STATUS "CLP: " ${CLP_VERSION} " (internal)")
else()
  message(STATUS "CLP: " ${CLP_VERSION} " (external)")
endif()

if(OPENMVG_USE_INTERNAL_COINUTILS)
  message(STATUS "COINUTILS: " ${COINUTILS_VERSION} " (internal)")
else()
  message(STATUS "COINUTILS: " ${COINUTILS_VERSION} " (external)")
endif()

if(OPENMVG_USE_INTERNAL_OSI)
  message(STATUS "OSI: " ${OSI_VERSION} " (internal)")
else()
  message(STATUS "OSI: " ${OSI_VERSION} " (external)")
endif()

if(OPENMVG_USE_INTERNAL_LEMON)
  message(STATUS "LEMON: " ${LEMON_VERSION} " (internal)")
else()
  message(STATUS "LEMON: " ${LEMON_VERSION} " (external)")
endif()

message("\n")

# ==============================================================================
# INSTALL RULES
# ==============================================================================

install(EXPORT openMVG-targets DESTINATION lib)
install(EXPORT openMVG-targets
        DESTINATION share/openMVG/cmake FILE OpenMVGTargets.cmake)

#Adapt build include paths to install path
set(OPENMVG_INCLUDE_DIRS
  "${OPENMVG_INCLUDE_DIRS}"
  "${CMAKE_INSTALL_PREFIX}/include/openMVG")

list(REMOVE_ITEM OPENMVG_INCLUDE_DIRS ${generatedDir})
string(REGEX REPLACE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_INSTALL_PREFIX}/include"
  OPENMVG_INCLUDE_DIRS
  "${OPENMVG_INCLUDE_DIRS}"
)

string(REGEX REPLACE
  "third_party"
  "openMVG/third_party"
  OPENMVG_INCLUDE_DIRS
  "${OPENMVG_INCLUDE_DIRS}"
)
string(REGEX REPLACE
  "dependencies"
  "openMVG_dependencies"
  OPENMVG_INCLUDE_DIRS
  "${OPENMVG_INCLUDE_DIRS}"
)

if(OPENMVG_USE_INTERNAL_CERES)
  string(REGEX REPLACE
    "ceres-solver"
    "ceres"
    OPENMVG_INCLUDE_DIRS
    "${OPENMVG_INCLUDE_DIRS}"
  )
  string(REGEX REPLACE
    "internal/ceres/"
    ""
    OPENMVG_INCLUDE_DIRS
    "${OPENMVG_INCLUDE_DIRS}"
  )
endif()

# Create a OpenMVGConfig.cmake file. <name>Config.cmake files are searched by
# find_package() automatically. We configure that file so that we can put any
# information we want in it, e.g. version numbers, include directories, etc.
configure_file("${CMAKE_SOURCE_DIR}/cmakeFindModules/OpenMVGConfig.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/OpenMVGConfig.cmake" @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/OpenMVGConfig.cmake"
        DESTINATION share/openMVG/cmake)

# create the config.hpp file containing all the preprocessor definitions
set(configfile "${generatedDir}/openMVG/config.hpp")
configure_file("${CMAKE_SOURCE_DIR}/cmakeFindModules/config.hpp.in"
               "${configfile}" @ONLY)

install(FILES "${configfile}"
        DESTINATION "include/openMVG")

########### Add uninstall target ###############
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmakeFindModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmakeFindModules/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmakeFindModules/cmake_uninstall.cmake")
