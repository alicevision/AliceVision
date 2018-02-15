message("CMake version: ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}")
cmake_minimum_required(VERSION 3.4)
project(aliceVisionMVS LANGUAGES CXX C)

# options
option(ALICEVISION_BUILD_SHARED "Build AliceVision shared libs" OFF)
option(ALICEVISION_USE_OPENMP "Enable OpenMP parallelization" ON)

# C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

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
elseif()
  set(BUILD_SHARED_LIBS OFF)
endif()

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
message(INFO "Geogram: ${GEOGRAM_LIBRARY}, ${GEOGRAM_INCLUDE_DIR}")

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
find_package(OpenImageIO)
if(OPENIMAGEIO_FOUND)
  message(STATUS "OpenImageIO found.")
else()
  message(SEND_ERROR "Failed to find OpenImageIO.")
endif()

# CUDA
find_package(CUDA 7.0)

if(NOT CUDA_FOUND)
  message(FATAL_ERROR "Could not find CUDA >= 7.0")
endif()

set(CUDA_SEPARABLE_COMPILATION ON)

message("Build Mode: ${CMAKE_BUILD_TYPE}")

set(CUDA_NVCC_FLAGS_DEBUG   "${CUDA_NVCC_FLAGS_DEBUG};-G;-g")
set(CUDA_NVCC_FLAGS_RELEASE "${CUDA_NVCC_FLAGS_RELEASE};-O3")

set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-gencode;arch=compute_20,code=sm_20")
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

# set the directory where all the generated files (config etc) will be placed
set(generatedDir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# contains the "root" directory from which including all headers
set(ALICEVISION_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src)

add_subdirectory(src/aliceVision)
add_subdirectory(src/software)
# create the config.hpp file containing all the preprocessor definitions
set(configfile "${generatedDir}/aliceVision/config.hpp")
configure_file("${CMAKE_SOURCE_DIR}/cmake/config.hpp.in"
               "${configfile}" @ONLY)


