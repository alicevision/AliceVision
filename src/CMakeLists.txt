cmake_minimum_required(VERSION 3.12)
message("CMake version: ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}")

# ==============================================================================
# AliceVision version
# ==============================================================================
file(STRINGS "aliceVision/version.hpp" _ALICEVISION_VERSION_HPP_CONTENTS REGEX "#define ALICEVISION_VERSION_")
foreach(v MAJOR MINOR REVISION)
  if("${_ALICEVISION_VERSION_HPP_CONTENTS}" MATCHES "#define ALICEVISION_VERSION_${v} ([0-9]+)")
    set(ALICEVISION_VERSION_${v} "${CMAKE_MATCH_1}")
  else()
    message(FATAL_ERROR "Failed to retrieve the AliceVision version the source code. Missing ALICEVISION_VERSION_${v}.")
  endif()
endforeach()
set(ALICEVISION_VERSION ${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_VERSION_REVISION})

project(aliceVisionSrc LANGUAGES C CXX VERSION ${ALICEVISION_VERSION})

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed.")
endif()

if(NOT ALICEVISION_ROOT)
  message(FATAL_ERROR "Should build from the CMakeLists.txt in the root folder.")
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

# C++17
set(CMAKE_CXX_STANDARD 17 CACHE STRING "The C++ standard used by the project")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD ${CMAKE_CXX_STANDARD} CACHE STRING "The C++ standard used to compile cuda code in the project")
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# ==============================================================================
# AliceVision build options
# ==============================================================================
option(ALICEVISION_BUILD_SFM "Build AliceVision SfM part" ON)
option(ALICEVISION_BUILD_MVS "Build AliceVision MVS part" ON)
option(ALICEVISION_BUILD_HDR "Build AliceVision HDR part" ON)
option(ALICEVISION_BUILD_SEGMENTATION "Build AliceVision Segmentation part" ON)
option(ALICEVISION_BUILD_PHOTOMETRICSTEREO "Build AliceVision Photometric Stereo part" ON)
option(ALICEVISION_BUILD_PANORAMA "Build AliceVision panorama part" ON)
option(ALICEVISION_BUILD_SOFTWARE "Build AliceVision command line tools." ON)
option(ALICEVISION_BUILD_COVERAGE "Enable code coverage generation (gcc only)" OFF)
option(ALICEVISION_BUILD_SWIG_BINDING "Build the Python binding with SWIG." OFF)
trilean_option(ALICEVISION_BUILD_LIDAR "Build AliceVision LIDAR part" AUTO)
trilean_option(ALICEVISION_BUILD_DOC "Build AliceVision documentation" AUTO)

trilean_option(ALICEVISION_USE_OPENMP "Enable OpenMP parallelization" ON)
trilean_option(ALICEVISION_USE_CCTAG "Enable CCTAG markers" AUTO)
trilean_option(ALICEVISION_USE_APRILTAG "Enable AprilTag markers" AUTO)
trilean_option(ALICEVISION_USE_POPSIFT "Enable GPU SIFT implementation" AUTO)
trilean_option(ALICEVISION_USE_OPENGV "Enable use of OpenGV algorithms" AUTO)
trilean_option(ALICEVISION_USE_ALEMBIC "Enable Alembic I/O" AUTO)
trilean_option(ALICEVISION_USE_UNCERTAINTYTE "Enable Uncertainty computation" AUTO)
trilean_option(ALICEVISION_USE_ONNX "Enable ONNX Runtime" AUTO)
option(ALICEVISION_USE_ONNX_GPU "Use CUDA with ONNX Runtime" ON)
trilean_option(ALICEVISION_USE_CUDA "Enable CUDA" ON)
trilean_option(ALICEVISION_USE_OPENCV "Enable use of OpenCV algorithms" OFF)
trilean_option(ALICEVISION_USE_OPENCV_CONTRIB "Enable use of OpenCV algorithms from extra modules" AUTO)
option(ALICEVISION_USE_OCVSIFT "Add or not OpenCV SIFT in available features" OFF)
mark_as_advanced(FORCE ALICEVISION_USE_OCVSIFT)

option(ALICEVISION_USE_MESHSDFILTER "Use MeshSDFilter library (enable MeshDenoising and MeshDecimate)" ON)

option(ALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE "Require Ceres with SuiteSparse (ensure best performances)" ON)

option(ALICEVISION_USE_RPATH "Add RPATH on software with relative paths to libraries" ON)
option(ALICEVISION_REMOVE_ABSOLUTE "Remove absolute paths in dependencies" OFF)
option(ALICEVISION_BUILD_TESTS "Build AliceVision tests" OFF)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)


# Default build is in Release mode
if(NOT CMAKE_BUILD_TYPE AND NOT MSVC)
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(NOT ALICEVISION_BUILD_SFM)
    SET(ALICEVISION_BUILD_MVS OFF)
    SET(ALICEVISION_BUILD_HDR OFF)
    SET(ALICEVISION_BUILD_SEGMENTATION OFF)
    SET(ALICEVISION_BUILD_PHOTOMETRICSTEREO OFF)
    SET(ALICEVISION_BUILD_PANORAMA OFF)
    SET(ALICEVISION_BUILD_LIDAR OFF)
endif()

# ==============================================================================
# Enable cmake UNIT TEST framework
# ==============================================================================
if(ALICEVISION_BUILD_TESTS AND NOT CMAKE_TESTING_ENABLED)
  enable_testing()
endif()

# ==============================================================================
# GNUInstallDirs CMake module
# - Define GNU standard installation directories
# - Provides install directory variables as defined by the GNU Coding Standards.
# ==============================================================================
include(GNUInstallDirs)

if(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "Force CMAKE_INSTALL_DO_STRIP in Release")
    set(CMAKE_INSTALL_DO_STRIP TRUE)
endif()

if(ALICEVISION_USE_RPATH)
  if(APPLE)
    set(CMAKE_MACOSX_RPATH 1)
    set(CMAKE_INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR};@loader_path")
  elseif(UNIX)
    set(CMAKE_INSTALL_RPATH "\\$ORIGIN/../${CMAKE_INSTALL_LIBDIR};\\$ORIGIN")
  endif()
endif()


# Set build path
set(EXECUTABLE_OUTPUT_PATH "${ALICEVISION_ROOT}/${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
set(LIBRARY_OUTPUT_PATH "${ALICEVISION_ROOT}/${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")

# Windows specific defines
if(WIN32)
  add_definitions(-DNOMINMAX)
  add_definitions(-D_USE_MATH_DEFINES)
  if(MSVC)
    add_compile_options(/bigobj)
    add_compile_options(/MP)
  endif()
endif()

# Avoids deprecation warning caused by internals of json_parser. This is properly fixed in
# Boost 1.76.0: https://github.com/boostorg/property_tree/commit/d1c8825a45a0717e1ad79583d3283b0e5e32831e
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS=1)

# Folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "_CMakePredefinedTargets")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(BUILD_SHARED_LIBS)
  if(WIN32)
    # Export all symbols from the dynamic libraries by default (avoid dllexport markup)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  endif()
endif()

# ==============================================================================
# MACRO utility
# ==============================================================================
macro(add_target_properties _target _name)
  set(_properties)
  foreach(_prop ${ARGN})
    set(_properties "${_properties} ${_prop}")
  endforeach(_prop)
  get_target_property(_old_properties ${_target} ${_name})
  if(NOT _old_properties)
    # in case it's NOTFOUND
    set(_old_properties)
  endif(NOT _old_properties)
  set_target_properties(${_target} PROPERTIES ${_name} "${_old_properties} ${_properties}")
endmacro(add_target_properties)

# ==============================================================================
# Check that submodule have been initialized and updated
# ==============================================================================
if(ALICEVISION_USE_MESHSDFILTER AND NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/MeshSDFilter/CMakeLists.txt)
  message(FATAL_ERROR
    "\n submodule(s) are missing, please update your repository:\n"
    "  > git submodule update -i\n")
endif()

# ==============================================================================
# Additional cmake find modules
# ==============================================================================
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(OptimizeForArchitecture)
OptimizeForArchitecture()
set(ALICEVISION_HAVE_SSE 0)
if(SSE2_FOUND OR TARGET_ARCHITECTURE STREQUAL "native")
  if(MSVC AND NOT ${CMAKE_CL_64})
    add_definitions(/arch:SSE2)
  endif()
  set(ALICEVISION_HAVE_SSE 1)
endif()

if(UNIX)
  if (ALICEVISION_BUILD_COVERAGE)
    set(CMAKE_C_FLAGS_RELEASE "-O1")
    set(CMAKE_CXX_FLAGS_RELEASE "-O1")
  else ()
    set(CMAKE_C_FLAGS_RELEASE "-O3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  include(AddCompilerFlag)

  # This flag is useful as not returning from a non-void function is an error with MSVC
  AddCompilerFlag("-Werror=return-type")
  AddCompilerFlag("-Werror=switch")
  AddCompilerFlag("-Werror=return-local-addr")
endif()

# Eigen requires overaligned buffers for maximum efficiency (e.g. on AVX512 buffers may need to
# be aligned to 64 bytes). AliceVision currently does not support this. Fortunately this is fixed
# in C++17. While we can't upgrade to C++17 just yet, some compilers support overaligned
# allocation feature with a separate flag, so use it if alignment is enabled in Eigen.
# See https://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html
if(AV_EIGEN_MEMORY_ALIGNMENT)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 7.1)
    AddCompilerFlag("-faligned-new")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 6.0)
    AddCompilerFlag("-faligned-new")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.0)
    AddCompilerFlag("-faligned-new")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 19.12)
    AddCompilerFlag("/Zc:alignedNew")
  else()
    message(FATAL_ERROR "AV_EIGEN_MEMORY_ALIGNMENT is only supported starting GCC 7.1, Clang 6.0 and MSVC 2017 15.5")
  endif()
endif()

# ==============================================================================
# Enable code coverage generation (only with GCC)
# ==============================================================================
if(ALICEVISION_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
    message("ALICEVISION_BUILD_COVERAGE enabled")
    include(CodeCoverage)
    setup_target_for_coverage_gcovr_xml(
    NAME coverage                 
    EXECUTABLE ctest -j 4)
endif()

# ==============================================================================
# OpenMP
# ==============================================================================
if(ALICEVISION_USE_OPENMP STREQUAL "OFF")
  set(ALICEVISION_HAVE_OPENMP 0)
else() # ON OR AUTO
  find_package(OpenMP)

  if(OPENMP_FOUND)
    set(ALICEVISION_HAVE_OPENMP 1)
    message(STATUS "OpenMP found.")
  elseif(ALICEVISION_USE_OPENMP STREQUAL "ON")
    set(ALICEVISION_HAVE_OPENMP 0)
    message(SEND_ERROR "Failed to find OpenMP.")
  else()
    set(ALICEVISION_HAVE_OPENMP 0)
  endif()
endif()

if(ALICEVISION_HAVE_OPENMP)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  if(NOT MSVC)
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      # for those using the clang with OpenMP support
      list(APPEND ALICEVISION_LIBRARY_DEPENDENCIES omp)
    else()
    list(APPEND ALICEVISION_LIBRARY_DEPENDENCIES gomp)
    endif()
  endif()
endif()

# ==============================================================================
# Boost
# ==============================================================================
option(BOOST_NO_CXX11 "if Boost is compiled without C++11 support (as it is often the case in OS packages) this must be enabled to avoid symbol conflicts (SCOPED_ENUM)." OFF)
set(ALICEVISION_BOOST_COMPONENTS atomic container date_time graph json log log_setup program_options regex serialization system thread timer)
if(ALICEVISION_BUILD_TESTS)
    set(ALICEVISION_BOOST_COMPONENT_UNITTEST unit_test_framework)
endif()
find_package(Boost 1.76.0 QUIET COMPONENTS ${ALICEVISION_BOOST_COMPONENTS} ${ALICEVISION_BOOST_COMPONENT_UNITTEST})

if(Boost_FOUND)
  message(STATUS "Boost ${Boost_LIB_VERSION} found.")
else()
  message(SEND_ERROR "Failed to find Boost.")
  message(SEND_ERROR "${Boost_ERROR_REASON}")
endif()


if(WIN32)
  # Disable BOOST autolink
  add_definitions(-DBOOST_ALL_NO_LIB)

  #To be removed later, a bug to make things work with current vcpkg
  #https://github.com/microsoft/vcpkg/issues/22495
  add_definitions(-DBOOST_USE_WINAPI_VERSION=BOOST_WINAPI_VERSION_WIN10)
endif()

if(BUILD_SHARED_LIBS)
  # Force BOOST to use dynamic libraries (avoid link error with boost program_options)
  # https://lists.boost.org/boost-users/2009/11/54015.php
  add_definitions(-DBOOST_ALL_DYN_LINK)
  add_definitions(-DBOOST_TEST_DYN_LINK)
else()
  set(Boost_USE_STATIC_LIBS ON)
endif()


# ==============================================================================
# OpenEXR >= 2.5
# ==============================================================================
find_package(OpenEXR REQUIRED)
if(TARGET OpenEXR::OpenEXR OR TARGET IlmBase::Half)
  if(OpenEXR_VERSION VERSION_GREATER_EQUAL 2.5)
    message(STATUS "OpenEXR found. (Version ${OpenEXR_VERSION})")
  else()
    message(SEND_ERROR "OpenEXR: Version found is ${OpenEXR_VERSION}, the minimal version required is 2.5.")
  endif()
else()
  message(SEND_ERROR "Failed to find OpenEXR.")
endif()

# ==============================================================================
# OpenImageIO
# ==============================================================================
find_package(OpenImageIO 2.0.9 REQUIRED)
if(OPENIMAGEIO_FOUND OR OpenImageIO_FOUND)
  message(STATUS "OpenImageIO found.")
else()
  message(SEND_ERROR "Failed to find OpenImageIO.")
endif()

# ==============================================================================
# Expat
# ==============================================================================
find_package(expat CONFIG REQUIRED)
if(EXPAT_FOUND OR expat_FOUND OR TARGET expat::expat)
  message(STATUS "Expat found.")
else()
  message(SEND_ERROR "Failed to find Expat.")
endif()

# ==============================================================================
# Mosek (linear programming interface)
# ==============================================================================
set(ALICEVISION_HAVE_MOSEK 0)

if(ALICEVISION_BUILD_SFM)
  find_package(Mosek)
  if(MOSEK_FOUND)
    find_package(Coin::OsiMsk)
  endif()

  #Install RULES
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/
    DESTINATION include/aliceVision_dependencies
    COMPONENT headers
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  )
endif()

# ==============================================================================
# Eigen
# ==============================================================================
find_package(Eigen3 3.3 REQUIRED)
if(Eigen3_FOUND OR EIGEN3_FOUND)
  # message(STATUS "EIGEN_INCLUDE_DIR: ${EIGEN_INCLUDE_DIR}")
  if(AV_EIGEN_MEMORY_ALIGNMENT)
    set(AV_EIGEN_DEFINITIONS -DALICEVISION_EIGEN_REQUIRE_ALIGNMENT=1)
  else()
    set(AV_EIGEN_DEFINITIONS -DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0)
  endif()
else()
  message(FATAL_ERROR " EIGEN NOT FOUND. EIGEN_INCLUDE_DIR: ${EIGEN_INCLUDE_DIR}")
endif()

# ==============================================================================
# onnxruntime
# ==============================================================================
set(ALICEVISION_HAVE_ONNX 0)
set(ALICEVISION_HAVE_ONNX_GPU 1)

if(NOT ALICEVISION_USE_ONNX STREQUAL "OFF")
  find_package(ONNXRuntime)

  if(TARGET ONNXRuntime::ONNXRuntime)
    set(ALICEVISION_HAVE_ONNX 1)
    message(STATUS "ONNXRuntime FOUND: ${ONNXRuntime_LIBRARY}")
  elseif(ALICEVISION_USE_ONNX STREQUAL "ON")
    message(SEND_ERROR "Failed to find ONNX Runtime.")
  endif()
endif()

if(ALICEVISION_USE_ONNX_GPU STREQUAL "OFF" OR ALICEVISION_USE_CUDA STREQUAL "OFF")
  set(ALICEVISION_HAVE_ONNX_GPU 0)
endif()

# ==============================================================================
# Ceres
# ==============================================================================
# - rely on Ceres_DIR
# ==============================================================================
if(ALICEVISION_BUILD_SFM)
  message(STATUS "Trying to find package Ceres for aliceVision: ${Ceres_DIR}")
  if(ALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE)
    message(STATUS "By default, Ceres required SuiteSparse to ensure best performances. if you explicitly need to build without it, you can use the option: -DALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE=OFF")
    find_package(Ceres QUIET REQUIRED COMPONENTS SuiteSparse CONFIG)
  else()
    find_package(Ceres CONFIG QUIET CONFIG)
  endif()

  if(Ceres_FOUND)
    message(STATUS "Ceres include dirs ${CERES_INCLUDE_DIRS}")
    message(STATUS "Ceres libraries ${CERES_LIBRARIES}")
    if(ALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE)
        # Ceres export include dirs but doesn't export suitesparse lib dependencies in CeresConfig.cmake
        # So here is a workaround:
        find_package(SuiteSparse)
        message(STATUS "SUITESPARSE_LIBRARIES: ${SUITESPARSE_LIBRARIES}")
        if(SUITESPARSE_LIBRARIES)
            list(APPEND CERES_LIBRARIES ${SUITESPARSE_LIBRARIES})
        endif()
    endif()
    message(STATUS "CERES_LIBRARIES: ${CERES_LIBRARIES}")
    message(STATUS "CERES_INCLUDE_DIRS: ${CERES_INCLUDE_DIRS}")
    if(WIN32)
        # avoid 'ERROR' macro clashing on Windows
        add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)
    endif()
    include_directories(${CERES_INCLUDE_DIRS})
  else()
    message(FATAL_ERROR "External CERES not found. Not found in Ceres_DIR: ${Ceres_DIR}")
  endif()
endif()

# ==============================================================================
# Flann
# ==============================================================================
if(ALICEVISION_BUILD_SFM)
  find_package(lz4 REQUIRED)
  find_package(flann REQUIRED)

  if (TARGET lz4::lz4)
    set(FLANN_LIBRARIES flann::flann_cpp lz4::lz4)
  elseif (TARGET LZ4::lz4_static)
    set(FLANN_LIBRARIES flann::flann_cpp LZ4::lz4_static)
  elseif (TARGET LZ4::lz4_shared)
    set(FLANN_LIBRARIES flann::flann_cpp LZ4::lz4_shared)
  else()
    message(FATAL_ERROR "FLANN can not be found")
  endif()
endif()

# ==============================================================================
# CoinUtils, Clp, Osi
# ==============================================================================
if(ALICEVISION_BUILD_SFM)
  find_package(CoinUtils REQUIRED)
  find_package(Clp REQUIRED)
  find_package(Osi REQUIRED)
endif()

# ==============================================================================
# Lemon
# ==============================================================================
if(ALICEVISION_BUILD_SFM)
  find_package(LEMON REQUIRED)
endif()

# ==============================================================================
# libE57Format
# ==============================================================================
if(NOT ALICEVISION_BUILD_LIDAR STREQUAL "OFF")
  find_package(E57Format)

  if(E57Format_FOUND)
    message(STATUS "E57Format found")
  elseif(ALICEVISION_BUILD_LIDAR STREQUAL "ON")
    message(SEND_ERROR "Failed to find E57Format")
  endif()
endif()


# ==============================================================================
# Assimp
# ==============================================================================
if(ALICEVISION_BUILD_MVS)
  find_package(assimp REQUIRED)
  message(STATUS "Assimp: ${ASSIMP_LIBRARIES}, ${ASSIMP_INCLUDE_DIRS}, ${ASSIMP_LIBRARY_DIRS}")
endif()

# ==============================================================================
# OpenCV
# ==============================================================================
# - optional, only external and enabled only if ALICEVISION_USE_OPENCV is ON
# ==============================================================================
set(ALICEVISION_HAVE_OPENCV 0)
set(ALICEVISION_HAVE_OCVSIFT 0)
set(ALICEVISION_HAVE_OPENCV_CONTRIB 0)

if(ALICEVISION_BUILD_SFM)
  if(NOT ALICEVISION_USE_OPENCV STREQUAL "OFF")
    find_package(OpenCV COMPONENTS core imgproc video imgcodecs videoio features2d optflow photo)

    if(OpenCV_FOUND)
      # We do not set the minimal version directly in find_package
      # due to this issue in opencv: https://github.com/opencv/opencv/issues/5931
      # which considers major version as exact requirement.
      if(OpenCV_VERSION VERSION_LESS "3.4.11")
        if(ALICEVISION_USE_OPENCV STREQUAL "ON")
          message(SEND_ERROR "Failed to find OpenCV 3.4.11+")
        endif()
      else()
        set(ALICEVISION_HAVE_OPENCV 1)
        message(STATUS "OpenCV found.")
      endif()

    elseif(ALICEVISION_USE_OPENCV STREQUAL "ON")
      message(SEND_ERROR "Failed to find OpenCV.")
    endif()
  endif()

  if(ALICEVISION_HAVE_OPENCV)
    if(NOT ALICEVISION_USE_OPENCV_CONTRIB STREQUAL "OFF")
      find_package(OpenCV COMPONENTS mcc)
      if(OpenCV_FOUND)
        set(ALICEVISION_HAVE_OPENCV_CONTRIB 1)
        message(STATUS "OpenCV contrib found.")
      elseif(ALICEVISION_USE_OPENCV_CONTRIB STREQUAL "ON")
        message(SEND_ERROR "Failed to find OpenCV.")
      endif()
    endif()

    include_directories(${OpenCV_INCLUDE_DIRS})
    # add a definition that allows the conditional compiling
    if(ALICEVISION_USE_OCVSIFT)
      set(ALICEVISION_HAVE_OCVSIFT 1)
    endif()
  endif()
endif()

# ==============================================================================
# Alembic
# ==============================================================================
# - optional, it allows to use the classes to export data in alembic format
# ==============================================================================
set(ALICEVISION_HAVE_ALEMBIC 0)

if(ALICEVISION_BUILD_SFM) # or ALICEVISION_BUILD_MVS
  if(NOT ALICEVISION_USE_ALEMBIC STREQUAL "OFF")
    find_package(Alembic 1.7.0 CONFIG)

    if(Alembic_FOUND)
      set(ALICEVISION_HAVE_ALEMBIC 1)
      message(STATUS "Alembic version ${Alembic_VERSION} found.")
      # There is a missing include dependency in Alembic cmake export.
      add_target_properties(Alembic::Alembic PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ILMBASE_INCLUDE_DIR}"
      )
    elseif(ALICEVISION_USE_ALEMBIC STREQUAL "ON")
      message(SEND_ERROR "Failed to find Alembic.")
    endif()
  endif()
endif()

# ==============================================================================
# USD
# ==============================================================================
# - optional, it allows to export the geometry and textures as USD
# ==============================================================================
set(ALICEVISION_HAVE_USD 0)

if(NOT ALICEVISION_USE_USD STREQUAL "OFF")
  find_package(pxr CONFIG)

  if(pxr_FOUND)
    set(ALICEVISION_HAVE_USD 1)
    message(STATUS "USD version ${PXR_VERSION} found.")
  elseif(ALICEVISION_USE_USD STREQUAL "ON")
    message(SEND_ERROR "Failed to find USD.")
  endif()
endif()

# ==============================================================================
# OpenGV
# ==============================================================================
# - optional, it allows to use the generic camera PnP algorithms for rig localization
# ==============================================================================
set(ALICEVISION_HAVE_OPENGV 0)

if(ALICEVISION_BUILD_SFM)
  if(NOT ALICEVISION_USE_OPENGV STREQUAL "OFF")
    find_package(opengv 1.0 CONFIG)
    if(opengv_FOUND)
      set(ALICEVISION_HAVE_OPENGV 1)
      message(STATUS "OpenGV ${opengv_VERSION} found.")
    elseif(ALICEVISION_USE_OPENGV STREQUAL "ON")
      message(SEND_ERROR "Failed to find OpenGV.")
    endif()
  endif()

endif()

# ==============================================================================
# UncertaintyTE
# ==============================================================================
# - optional, only external and enabled only if ALICEVISION_USE_UNCERTAINTYTE is ON
# ==============================================================================
set(ALICEVISION_HAVE_UNCERTAINTYTE 0)

if(ALICEVISION_BUILD_SFM)
  if(NOT ALICEVISION_USE_UNCERTAINTYTE STREQUAL "OFF")
    find_package(UncertaintyTE)

    if(UNCERTAINTYTE_FOUND)
      set(ALICEVISION_HAVE_UNCERTAINTYTE 1)
      message(STATUS "UncertaintyTE found.")
    elseif(ALICEVISION_USE_UNCERTAINTYTE STREQUAL "ON")
      message(SEND_ERROR "Failed to find UncertaintyTE.")
    endif()
  endif()

  if(ALICEVISION_HAVE_UNCERTAINTYTE)
    include_directories(${UNCERTAINTYTE_INCLUDE_DIR})
    link_directories(${UNCERTAINTYTE_LIBRARY_DIR})
  endif()
endif()

# ==============================================================================
# ZLIB
# ==============================================================================
if(ALICEVISION_BUILD_MVS)
  find_package(ZLIB REQUIRED)
endif()

# ==============================================================================
# GEOGRAM
# ==============================================================================
if(ALICEVISION_BUILD_MVS)
  find_package(Geogram REQUIRED)
  message(STATUS "Geogram: ${GEOGRAM_LIBRARY}, ${GEOGRAM_INCLUDE_DIR}")
endif()

# ==============================================================================
# MeshSDFilter
# ==============================================================================
# - optional, only internal and enabled only if ALICEVISION_USE_MESHSDFILTER is ON
# ==============================================================================
set(ALICEVISION_HAVE_MESHSDFILTER 0)

if(ALICEVISION_BUILD_MVS)
  find_package(OpenMesh REQUIRED)

  if(ALICEVISION_USE_MESHSDFILTER STREQUAL "ON")
    set(ALICEVISION_HAVE_MESHSDFILTER 1)
    add_subdirectory(dependencies/MeshSDFilter)
  endif()
endif()

# ==============================================================================
# CUDA
# ==============================================================================
option(ALICEVISION_USE_NVTX_PROFILING "Use CUDA NVTX for profiling." OFF)
option(ALICEVISION_NVCC_WARNINGS      "Switch on several additional warnings for CUDA nvcc." OFF)

set(ALICEVISION_HAVE_CUDA 0)

if(NOT ALICEVISION_USE_CUDA STREQUAL "OFF")

  if(BUILD_SHARED_LIBS)
    message(STATUS "BUILD_SHARED_LIBS ON")
    # Need to declare CUDA_USE_STATIC_CUDA_RUNTIME as an option to ensure that it is not overwritten in FindCUDA.
    option(CUDA_USE_STATIC_CUDA_RUNTIME "Use the static version of the CUDA runtime library if available" OFF)
    set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
  else()
    message(STATUS "BUILD_SHARED_LIBS OFF")
    option(CUDA_USE_STATIC_CUDA_RUNTIME "Use the static version of the CUDA runtime library if available" ON)
    set(CUDA_USE_STATIC_CUDA_RUNTIME ON)
  endif()

  find_package(CUDA 11.0)

  if(CUDA_FOUND)
    set(ALICEVISION_HAVE_CUDA 1)
    message(STATUS "CUDA found.")
  elseif(ALICEVISION_USE_CUDA STREQUAL "ON")
    message(SEND_ERROR "Failed to find CUDA (>= 11.0).")
  endif()
endif()

if(ALICEVISION_HAVE_CUDA)
  set(CUDA_SEPARABLE_COMPILATION ON)
  message("Build Mode: ${CMAKE_BUILD_TYPE}")

  set(CUDA_NVCC_FLAGS_DEBUG   "${CUDA_NVCC_FLAGS_DEBUG};-G;-g")
  # set(CUDA_NVCC_FLAGS_RELEASE "${CUDA_NVCC_FLAGS_RELEASE};-O3")
  if(CUDA_VERSION_MAJOR VERSION_GREATER_EQUAL 12)
    set(ALICEVISION_CUDA_CC_LIST_BASIC 50 52 60 61 62 70 72 75 80 86 87 89 90)
  elseif(CUDA_VERSION VERSION_GREATER_EQUAL 11.8)
    set(ALICEVISION_CUDA_CC_LIST_BASIC 35 50 52 60 61 62 70 72 75 80 86 87 89 90)
  elseif(CUDA_VERSION VERSION_GREATER_EQUAL 11.5)
    set(ALICEVISION_CUDA_CC_LIST_BASIC 35 50 52 60 61 62 70 72 75 80 86 87)
  elseif(CUDA_VERSION VERSION_GREATER_EQUAL 11.1)
    set(ALICEVISION_CUDA_CC_LIST_BASIC 35 50 52 60 61 62 70 72 75 80 86)
  elseif(CUDA_VERSION_MAJOR GREATER_EQUAL 11)
    set(ALICEVISION_CUDA_CC_LIST_BASIC 35 50 52 60 61 62 70 72 75 80)
  else()
    message(SEND_ERROR "Requires CUDA >= 11.0.")
  endif()
  set(ALICEVISION_CUDA_CC_LIST ${ALICEVISION_CUDA_CC_LIST_BASIC}
      CACHE STRING "CUDA CC versions to compile")

  # Add all requested CUDA CCs to the command line for offline compilation
  list(SORT ALICEVISION_CUDA_CC_LIST)
  foreach(ALICEVISION_CC_VERSION ${ALICEVISION_CUDA_CC_LIST})
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-gencode;arch=compute_${ALICEVISION_CC_VERSION},code=sm_${ALICEVISION_CC_VERSION}")
  endforeach()

  # Use the highest request CUDA CC for CUDA JIT compilation
  list(LENGTH ALICEVISION_CUDA_CC_LIST ALICEVISION_CC_LIST_LEN)
  MATH(EXPR ALICEVISION_CC_LIST_LEN "${ALICEVISION_CC_LIST_LEN}-1")
  list(GET ALICEVISION_CUDA_CC_LIST ${ALICEVISION_CC_LIST_LEN} ALICEVISION_CUDA_CC_LIST_LAST)
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-gencode;arch=compute_${ALICEVISION_CUDA_CC_LIST_LAST},code=compute_${ALICEVISION_CUDA_CC_LIST_LAST}")

  set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-std=c++17")

  # default stream legacy implies that the 0 stream synchronizes all streams
  set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};--default-stream;legacy")
  # default stream per-thread implies that each host thread has one non-synchronizing 0-stream
  # set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};--default-stream;per-thread")
  # print local memory usage per kernel: -Xptxas;-v
  #   -Xptxas;--warn-on-local-memory-usage;-Xptxas;--warn-on-spills
  message(STATUS "CUDA Version is ${CUDA_VERSION}")
  if(UNIX)
    set(CUDA_NVCC_FLAGS         "${CUDA_NVCC_FLAGS};-D_FORCE_INLINES")
  endif()
  if(ALICEVISION_NVCC_WARNINGS)
    set(CUDA_NVCC_FLAGS_RELEASE "${CUDA_NVCC_FLAGS_RELEASE};-Xptxas;-warn-lmem-usage")
    set(CUDA_NVCC_FLAGS_RELEASE "${CUDA_NVCC_FLAGS_RELEASE};-Xptxas;-warn-spills")
    set(CUDA_NVCC_FLAGS_RELEASE "${CUDA_NVCC_FLAGS_RELEASE};-Xptxas;--warn-on-local-memory-usage")
    set(CUDA_NVCC_FLAGS_RELEASE "${CUDA_NVCC_FLAGS_RELEASE};-Xptxas;--warn-on-spills")
  endif()

  # library required for CUDA dynamic parallelism, forgotten by CMake 3.4
  cuda_find_library_local_first(CUDA_CUDADEVRT_LIBRARY cudadevrt "\"cudadevrt\" library")

  # If user activates NVTX profiling, add library and flags
  if(ALICEVISION_USE_NVTX_PROFILING)
    message(STATUS "PROFILING CPU/GPU CODE: NVTX is in use")
    cuda_find_library_local_first(CUDA_NVTX_LIBRARY nvToolsExt "NVTX library")
    add_definitions(-DALICEVISION_USE_NVTX)
    include_directories(${CUDA_INCLUDE_DIRS})
    set(ALICEVISION_NVTX_LIBRARY ${CUDA_NVTX_LIBRARY})
  endif()
endif()

# ==============================================================================
# SWIG
# ==============================================================================
if(ALICEVISION_BUILD_SWIG_BINDING)
  find_package(SWIG REQUIRED)  # SWIG dependency
  if(SWIG_FOUND)
    include(UseSWIG)
    message(STATUS "SWIG found.")
  else()
    message(SEND_ERROR "Failed to find SWIG.")
  endif()


  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)  # Python dependencies
endif()


# ==============================================================================
# CCTag
# ==============================================================================
# - optional, only external and enabled only if ALICEVISION_USE_CCTAG is ON
# ==============================================================================
set(ALICEVISION_HAVE_CCTAG 0)

if(ALICEVISION_BUILD_SFM)
  if(NOT ALICEVISION_USE_CCTAG STREQUAL "OFF")
    if(ALICEVISION_HAVE_OPENCV)
      find_package(CCTag 1.0.0 CONFIG)

      if(CCTag_FOUND)
        set(ALICEVISION_HAVE_CCTAG 1)
        message(STATUS "CCTAG ${CCTag_VERSION} found.")
      elseif(ALICEVISION_USE_CCTAG STREQUAL "ON")
        message(SEND_ERROR "Failed to find CCTAG.")
      endif()
    elseif(ALICEVISION_USE_CCTAG STREQUAL "ON")
      message(SEND_ERROR "Can't use CCTAG without OPENCV.")
    endif()
  endif()
endif()

# ==============================================================================
# AprilTag
# ==============================================================================
# - optional, only external and enabled only if ALICEVISION_USE_APRILTAG is ON
# ==============================================================================
set(ALICEVISION_HAVE_APRILTAG 0)

if(ALICEVISION_BUILD_SFM)
  if(NOT ALICEVISION_USE_APRILTAG STREQUAL "OFF")
    find_package(apriltag CONFIG)

    if(apriltag_FOUND)
      set(ALICEVISION_HAVE_APRILTAG 1)
      message(STATUS "AprilTag found.")
    elseif(ALICEVISION_USE_APRILTAG STREQUAL "ON")
      message(SEND_ERROR "Failed to find AprilTag.")
    endif()
  endif()
endif()

# ==============================================================================
# PopSift
# ==============================================================================
# - optional, only external and enabled only if ALICEVISION_USE_POPSIFT is ON
# ==============================================================================
set(ALICEVISION_HAVE_POPSIFT 0)

if(ALICEVISION_BUILD_SFM)
  if(NOT ALICEVISION_USE_POPSIFT STREQUAL "OFF")
    find_package(PopSift CONFIG)

    if(PopSift_FOUND)
      set(ALICEVISION_HAVE_POPSIFT 1)
      message(STATUS "PopSIFT found.")
    elseif(ALICEVISION_USE_POPSIFT STREQUAL "ON")
      message(SEND_ERROR "Failed to find PopSIFT.")
    endif()
  endif()
endif()


# ==============================================================================
# Include directories
# ==============================================================================
# set the directory where all the generated files (config etc) will be placed
# ==============================================================================
set(generatedDir "${CMAKE_CURRENT_BINARY_DIR}/generated")
message("generatedDir: ${generatedDir}")

# contains the "root" directory from which including all headers
set(ALICEVISION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(ALICEVISION_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${generatedDir}
        ${CMAKE_CURRENT_SOURCE_DIR}/dependencies
        ${LEMON_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${UNCERTAINTYTE_INCLUDE_DIRS}
        ${MAGMA_INCLUDE_DIRS}
        ${FLANN_INCLUDE_DIRS}
        ${LP_INCLUDE_DIRS}
        ${COINUTILS_INCLUDE_DIRS}
        ${CLP_INCLUDE_DIRS}
        ${OSI_INCLUDE_DIRS}
        )

include_directories(${ALICEVISION_INCLUDE_DIRS})

# ==============================================================================
# Documentation
# --------------------------
# Sphinx and Doxygen detection
# ==============================================================================
if(ALICEVISION_BUILD_DOC STREQUAL "OFF")
  set(ALICEVISION_HAVE_DOC 0)
else()
  # try to find the packages
  find_package(Sphinx)
  find_package(Doxygen)

   # if neither one is found
  if (NOT EXISTS ${SPHINX_EXECUTABLE} AND NOT DOXYGEN_FOUND)
    # generate error if build doc was required
    if(ALICEVISION_BUILD_DOC STREQUAL "ON")
      set(ALICEVISION_HAVE_DOC 0)
      message(SEND_ERROR "Failed to find Sphinx.\nSphinx need to be installed to generate the documentation")
    else()
      # otherwise quietly fail and set the variable to off
      set(ALICEVISION_HAVE_DOC 0)
    endif()
  else()
    # set the variable to true
    set(ALICEVISION_HAVE_DOC 1)

    # sphinx stuff
    if(EXISTS ${SPHINX_EXECUTABLE})
      message(STATUS "Sphinx found.")
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
        PROPERTY FOLDER AliceVision
      )
    endif()

    # doxygen stuff
    if(DOXYGEN_FOUND)
      message(STATUS "Doxygen found.")

      # set input and output files
      set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/../docs/doxygen/Doxyfile.in)
      set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

      # request to configure the file
      configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

      add_custom_target( doc_doxygen
              COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
              WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
              COMMENT "Generating API documentation with Doxygen"
              VERBATIM )

      set_property(TARGET doc_doxygen
              PROPERTY FOLDER AliceVision
              )

      if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc)
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc  DESTINATION ${CMAKE_INSTALL_DOCDIR})
      endif()

    endif()
  endif()
endif()

# ==============================================================================
# Information print
# ==============================================================================
message("\n")
message("** AliceVision version: " ${ALICEVISION_VERSION})
message("** Target architecture: " ${TARGET_ARCHITECTURE})
message("** Build Shared libs: " ${BUILD_SHARED_LIBS})
message("** Build SfM part: " ${ALICEVISION_BUILD_SFM})
message("** Build MVS part: " ${ALICEVISION_BUILD_MVS})
message("** Build LIDAR part: " ${ALICEVISION_BUILD_LIDAR})
message("** Build AliceVision tests: " ${ALICEVISION_BUILD_TESTS})
message("** Build AliceVision documentation: " ${ALICEVISION_HAVE_DOC})
message("** Build AliceVision+OpenCV samples programs: " ${ALICEVISION_HAVE_OPENCV})
message("** Build UncertaintyTE: " ${ALICEVISION_HAVE_UNCERTAINTYTE})
message("** Build MeshSDFilter: " ${ALICEVISION_HAVE_MESHSDFILTER})
message("** Build Alembic exporter: " ${ALICEVISION_HAVE_ALEMBIC})
message("** Build SWIG Python binding: " ${ALICEVISION_BUILD_SWIG_BINDING})
message("** Enable code coverage generation: " ${ALICEVISION_BUILD_COVERAGE})
message("** Enable OpenMP parallelization: " ${ALICEVISION_HAVE_OPENMP})
message("** Use CUDA: " ${ALICEVISION_HAVE_CUDA})
message("** Use OpenCV SIFT features: " ${ALICEVISION_HAVE_OCVSIFT})
message("** Use PopSift feature extractor: " ${ALICEVISION_HAVE_POPSIFT})
message("** Use CCTAG markers: " ${ALICEVISION_HAVE_CCTAG})
message("** Use AprilTag markers: " ${ALICEVISION_HAVE_APRILTAG})
message("** Use OpenGV for rig localization: " ${ALICEVISION_HAVE_OPENGV})
message("** Use ONNX: " ${ALICEVISION_HAVE_ONNX})
message("** Use ONNX with CUDA: " ${ALICEVISION_HAVE_ONNX_GPU})
message("\n")

message(STATUS "EIGEN: " ${EIGEN_VERSION} "")

if(ALICEVISION_BUILD_SFM)
  message(STATUS "CERES: " ${CERES_VERSION} "")
  message(STATUS "FLANN: " ${FLANN_VERSION} "")
  message(STATUS "CLP: " ${Clp_VERSION} "")
  message(STATUS "COINUTILS: " ${CoinUtils_VERSION} "")
  message(STATUS "OSI: " ${Osi_VERSION} "")
  message(STATUS "LEMON: " ${LEMON_VERSION} "")
endif()

if(NOT ALICEVISION_BUILD_LIDAR STREQUAL "OFF")
  message(STATUS "E57FORMAT: " ${E57Format} "")
endif()

if(ALICEVISION_BUILD_SWIG_BINDING)
  message("\n")
  message(STATUS "SWIG: " ${SWIG_VERSION} "")
endif()

message("\n")

# ==============================================================================
# AliceVision CMake Helpers
# ==============================================================================
include(Helpers)

# ==============================================================================
# Internal libraries dependencies
# ==============================================================================
add_subdirectory(dependencies)

# ==============================================================================
# AliceVision modules
# ==============================================================================

# software(s) under patent or commercial licence
# Included for research purpose only
if(ALICEVISION_BUILD_SFM)
  add_subdirectory(nonFree)
endif()

# The aliceVision library itself
add_subdirectory(aliceVision)

# Complete software(s) build on aliceVision libraries
if(ALICEVISION_BUILD_SOFTWARE)
  add_subdirectory(software)
endif()

# ==============================================================================
# Install rules
# ==============================================================================

# Include module with fuction 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

# name of the exported project (cannot use PROJECT_NAME as it is AliceVisionSrc
set(exportedProjectName "AliceVision")
# the generated cmake config file
set(cmakeVersionConfig "${generatedDir}/${exportedProjectName}ConfigVersion.cmake")
# the generated cmake version file
set(cmakeProjectConfig "${generatedDir}/${exportedProjectName}Config.cmake")
# place to install the cmake-related files
set(cmakeConfigInstallDir "${CMAKE_INSTALL_DATADIR}/aliceVision/cmake")
# the name of the cmake Target file to export
set(targetsExportName "${exportedProjectName}Targets")

# generate '<PROJECT-NAME>ConfigVersion.cmake'
# Note: major version number must be the same as requested
write_basic_package_version_file("${cmakeVersionConfig}" COMPATIBILITY SameMajorVersion)

# Configure '<PROJECT-NAME>Config.cmake'
configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/AliceVisionConfig.cmake.in"
        "${cmakeProjectConfig}"
        INSTALL_DESTINATION "${cmakeConfigInstallDir}")

# this exports the targets only to be used for the in-tree compilation, do not export this
export(EXPORT aliceVision-targets FILE "${generatedDir}/${targetsExportName}.cmake")

# generate and install the target.cmake file to be used when the library is installed.
install(EXPORT aliceVision-targets DESTINATION ${cmakeConfigInstallDir} FILE ${targetsExportName}.cmake)

# install ${PROJECT_NAME}ConfigVersion.cmake and ${PROJECT_NAME}Config.cmake
install(FILES "${cmakeProjectConfig}" "${cmakeVersionConfig}"
        DESTINATION "${cmakeConfigInstallDir}")


# create the config.hpp file containing all the preprocessor definitions
set(configfile "${generatedDir}/aliceVision/config.hpp")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.hpp.in"
               "${configfile}" @ONLY)

install(FILES "${configfile}"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/aliceVision")


# Add uninstall target
set(cmakeUninstallFile "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in" ${cmakeUninstallFile}
  IMMEDIATE @ONLY
)

add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${cmakeUninstallFile}")
