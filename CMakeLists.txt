cmake_minimum_required(VERSION 3.3)

project(aliceVision LANGUAGES C CXX)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed.")
endif()

message("CMake version: ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}")
cmake_minimum_required(VERSION 3.4)

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

# Enable folders in supported IDEs
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# AliceVision build options
# ==============================================================================
option(ALICEVISION_BUILD_SHARED "Build AliceVision shared libs" OFF)
option(ALICEVISION_BUILD_TESTS "Build AliceVision tests" OFF)
option(ALICEVISION_BUILD_EXAMPLES "Build AliceVision samples applications." ON)
option(ALICEVISION_BUILD_COVERAGE "Enable code coverage generation (gcc only)" OFF)
trilean_option(ALICEVISION_BUILD_DOC "Build AliceVision documentation" AUTO)

trilean_option(ALICEVISION_USE_OPENMP "Enable OpenMP parallelization" ON)
trilean_option(ALICEVISION_USE_CCTAG "Enable CCTAG markers" AUTO)
trilean_option(ALICEVISION_USE_POPSIFT "Enable GPU SIFT implementation" AUTO)
trilean_option(ALICEVISION_USE_OPENGV "Enable use of OpenGV algorithms" AUTO)
trilean_option(ALICEVISION_USE_ALEMBIC "Enable Alembic I/O" AUTO)
trilean_option(ALICEVISION_USE_UNCERTAINTYTE "Enable Uncertainty computation" AUTO)
trilean_option(ALICEVISION_USE_CUDA "Enable CUDA" ON)
option(ALICEVISION_USE_MESHSDFILTER "Use MeshSDFilter library (enable MeshDenoising and MeshDecimate)." ON)

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

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Windows specific defines
if(WIN32)
  add_definitions(-DNOMINMAX)
  add_definitions(-D_USE_MATH_DEFINES)
  # avoid 'ERROR' macro clashing on Windows
  add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# There was a bug in FindCUDA fixed in cmake 3.10 that creates empty strings on the nvcc command line.
# The solution merged in cmake 3.10 use the command COMMAND_EXPAND_LISTS only available from cmake >= 3.8
# https://gitlab.kitware.com/cmake/cmake/merge_requests/1008
# So we use another solution compatible with cmake >= 3.4 proposed here:
# https://gitlab.kitware.com/cmake/cmake/issues/16411
# if cmake version < 3.10 use our hacked version of FindCUDA:
if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_LESS 3.10)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/compatibilityCUDA)
endif()

# Set shared or static mode
if(ALICEVISION_BUILD_SHARED)
  set(BUILD_SHARED_LIBS ON)
  if(WIN32)
    # Export all symbols from the dynamic libraries by default (avoid dllexport markup)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  endif()
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


# ZLIB
find_package(ZLIB REQUIRED)

# BOOST
find_package(Boost REQUIRED thread filesystem system date_time graph program_options)

# Disable BOOST autolink
add_definitions(-DBOOST_ALL_NO_LIB)
# Force BOOST to use dynamic libraries (avoid link error with boost program_options)
# https://lists.boost.org/boost-users/2009/11/54015.php
add_definitions(-DBOOST_ALL_DYN_LINK)
# Avoid link errors on boost filesystem copy_file function
# http://stackoverflow.com/questions/35007134/c-boost-undefined-reference-to-boostfilesystemdetailcopy-file
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "4.9.3")
    if(CUDA_VERSION VERSION_LESS "8.0")
      MESSAGE(STATUS "Found gcc >=5 and CUDA <= 7.5, adding workaround C++ flags")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORCE_INLINES -D_MWAITXINTRIN_H_INCLUDED -D__STRICT_ANSI__")
    endif(CUDA_VERSION VERSION_LESS "8.0")
  endif(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "4.9.3")
endif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")

# GEOGRAM
find_package(Geogram REQUIRED)
message(STATUS "Geogram: ${GEOGRAM_LIBRARY}, ${GEOGRAM_INCLUDE_DIR}")

# OpenMP
if(ALICEVISION_USE_OPENMP)
  find_package(OpenMP REQUIRED)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(ALICEVISION_HAVE_OPENMP 1)
else()
  set(ALICEVISION_HAVE_OPENMP 0)
endif()

# OpenImageIO
find_package(OpenImageIO REQUIRED)
if(OPENIMAGEIO_FOUND)
  message(STATUS "OpenImageIO found.")
endif()

# Eigen
find_package(Eigen3 3.3 REQUIRED)
if(EIGEN3_FOUND)
  message("Found external Eigen. include: ${EIGEN3_INCLUDE_DIR}, version: ${EIGEN3_VERSION_STRING}.")
endif()

# CUDA
if(ALICEVISION_USE_CUDA STREQUAL "OFF")
  set(ALICEVISION_HAVE_CUDA 0)
else() # ON OR AUTO
  find_package(CUDA 7.0)
  if(CUDA_FOUND)
    set(ALICEVISION_HAVE_CUDA 1)
    message(STATUS "CUDA found.")
  elseif(ALICEVISION_USE_CUDA STREQUAL "ON")
    set(ALICEVISION_HAVE_CUDA 0)
    message(SEND_ERROR "Failed to find CUDA (>= 7.0).")
  else()
    set(ALICEVISION_HAVE_CUDA 0)
  endif()
endif()

if(ALICEVISION_HAVE_CUDA)
  set(CUDA_SEPARABLE_COMPILATION ON)
  message("Build Mode: ${CMAKE_BUILD_TYPE}")

  set(CUDA_NVCC_FLAGS_DEBUG   "${CUDA_NVCC_FLAGS_DEBUG};-G;-g")
  set(CUDA_NVCC_FLAGS_RELEASE "${CUDA_NVCC_FLAGS_RELEASE};-O3")
  if(CUDA_VERSION VERSION_LESS 9.0) # cuda < 9.0
    set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode;arch=compute_20,code=sm_20")
  endif()
  set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode;arch=compute_30,code=sm_30")
  set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode;arch=compute_35,code=sm_35")
  set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode;arch=compute_50,code=sm_50")
  set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode;arch=compute_52,code=sm_52")
  set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode=arch=compute_52,code=compute_52")

  if(NOT CUDA_VERSION VERSION_LESS 8.0) # cuda >= 8.0
    set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode=arch=compute_60,code=sm_60")
    set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode=arch=compute_61,code=sm_61")
    set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode=arch=compute_62,code=sm_62")
  endif()

  if(NOT CUDA_VERSION VERSION_LESS 7.0) # cuda >= 7.0
    set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-std=c++11")
  endif()

  # default stream legacy implies that the 0 stream synchronizes all streams
  set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};--default-stream;legacy")
  # default stream per-thread implies that each host thread has one non-synchronizing 0-stream
  # set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};--default-stream;per-thread")
  # print local memory usage per kernel: -Xptxas;-v
  # CUDA >= 7.5: -Xptxas;--warn-on-local-memory-usage;-Xptxas;--warn-on-spills
  message(STATUS "CUDA Version is ${CUDA_VERSION}")
  if(NOT CUDA_VERSION VERSION_LESS 7.5) # cuda >= 7.5
    if(UNIX)
      set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-D_FORCE_INLINES")
    endif()
    set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-Xptxas;--warn-on-local-memory-usage")
    set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-Xptxas;--warn-on-spills")
  endif()

  # library required for CUDA dynamic parallelism, forgotten by CMake 3.4
  cuda_find_library_local_first(CUDA_CUDADEVRT_LIBRARY cudadevrt "\"cudadevrt\" library")
endif()

# set the directory where all the generated files (config etc) will be placed
set(generatedDir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# contains the "root" directory from which including all headers
set(ALICEVISION_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src)

if(ALICEVISION_USE_MESHSDFILTER STREQUAL "OFF")
  set(ALICEVISION_HAVE_MESHSDFILTER 0)
else() # ON
  set(ALICEVISION_HAVE_MESHSDFILTER 1)
  add_subdirectory(src/dependencies/MeshSDFilter)
endif()


add_subdirectory(src)

# create the config.hpp file containing all the preprocessor definitions
set(configfile "${generatedDir}/aliceVision/config.hpp")
configure_file("${CMAKE_SOURCE_DIR}/cmake/config.hpp.in"
               "${configfile}" @ONLY)


