
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

# ==============================================================================
# Check that submodule have been initialized and updated
# ==============================================================================
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/cereal/include)
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
set(ALICEVISION_HAVE_SSE 0)
if(SSE2_FOUND OR TARGET_ARCHITECTURE STREQUAL "native")
  if(MSVC AND NOT ${CMAKE_CL_64})
    add_definitions(/arch:SSE2)
  endif()
  set(ALICEVISION_HAVE_SSE 1)
endif()
if(UNIX AND NOT ALICEVISION_BUILD_COVERAGE)
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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  if(NOT MSVC)
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      # for those using the clang with OpenMP support
      list(APPEND ALICEVISION_LIBRARY_DEPENDENCIES iomp)
    else()
    list(APPEND ALICEVISION_LIBRARY_DEPENDENCIES gomp)
    endif()
  endif()
endif()

# ==============================================================================
# Enable code coverage generation (only with GCC)
# ==============================================================================
if(ALICEVISION_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
  message("ALICEVISION_BUILD_COVERAGE enabled")
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
#- Mosek (linear programming interface)
#===============================================================================
set(ALICEVISION_HAVE_MOSEK 0)
find_package(Mosek)
if(MOSEK_FOUND)
  set(ALICEVISION_HAVE_MOSEK 1)
  set(LP_INCLUDE_DIRS
    ${MOSEK_INCLUDE}
    ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/Osi/src/OsiMsk/
  )
  include_directories(${LP_INCLUDE_DIRS})
endif()

#Install RULES
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/
  DESTINATION include/aliceVision_dependencies
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# ==============================================================================
# Eigen
# ==============================================================================
# - internal by default,
# - external if EIGEN_INCLUDE_DIR_HINTS is defined
# ==============================================================================
if(NOT DEFINED EIGEN_INCLUDE_DIR_HINTS)
  set(EIGEN_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/eigen)
  set(ALICEVISION_USE_INTERNAL_EIGEN ON)
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
message(STATUS "Trying to find package Ceres for aliceVision: ${Ceres_DIR}")
if(ALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE)
  message(STATUS "By default, Ceres required SuiteSparse to ensure best performances. if you explicitly need to build without it, you can use the option: -DALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE=OFF")
  find_package(Ceres QUIET REQUIRED SuiteSparse CONFIG)
else()
  find_package(Ceres CONFIG QUIET CONFIG)
endif()

if(NOT Ceres_FOUND)
  if(DEFINED ALICEVISION_USE_INTERNAL_CERES AND NOT ALICEVISION_USE_INTERNAL_CERES)
    message(FATAL_ERROR "External CERES not found. Not found in Ceres_DIR: ${Ceres_DIR}")
  endif()

  message(STATUS "No external CERES detected, using the embedded one.")
  set(ALICEVISION_USE_INTERNAL_CERES ON)
  set(CERES_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/ceres-solver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/ceres-solver/internal/ceres/miniglog
    ${CMAKE_CURRENT_BINARY_DIR}/dependencies/ceres-solver/config)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/ceres-solver/VERSION" CERES_CONFIG)
    string(REGEX REPLACE "version ([0-9.]+).*" "\\1" CERES_VERSION ${CERES_CONFIG})
  set(CERES_LIBRARIES ceres cxsparse)
  set(Ceres_FOUND TRUE)
else()
  message(STATUS "Ceres include dirs ${CERES_INCLUDE_DIRS}")
  message(STATUS "Ceres libraries ${CERES_LIBRARIES}")
  include_directories(${CERES_INCLUDE_DIRS})
  set(ALICEVISION_USE_INTERNAL_CERES OFF)
endif()

# ==============================================================================
# Flann
# ==============================================================================
# - internal by default (flann),
# - external if FLANN_INCLUDE_DIR_HINTS and a valid Flann setup is found
# ==============================================================================
if(NOT DEFINED FLANN_INCLUDE_DIR_HINTS)
  set(FLANN_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/flann/src/cpp)
  set(ALICEVISION_USE_INTERNAL_FLANN ON)
endif()
find_package(Flann QUIET)
if(NOT FLANN_FOUND OR ALICEVISION_USE_INTERNAL_FLANN)
  set(FLANN_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/flann/src/cpp)
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
  set(ALICEVISION_USE_INTERNAL_COINUTILS ON)
endif()
find_package(CoinUtils QUIET)
if(NOT COINUTILS_FOUND OR ALICEVISION_USE_INTERNAL_COINUTILS)
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
  set(ALICEVISION_USE_INTERNAL_CLP ON)
endif()
find_package(Clp QUIET)
if(NOT CLP_FOUND OR ALICEVISION_USE_INTERNAL_CLP)
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
  set(ALICEVISION_USE_INTERNAL_OSI ON)
endif()
find_package(Osi QUIET)
if(NOT OSI_FOUND OR ALICEVISION_USE_INTERNAL_OSI)
  set(OSI_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/osi_clp/Osi/src/Osi/)
  set(OSI_LIBRARY lib_Osi)
endif()

# ==============================================================================
# Internal CLP/OSI/COINUTILS libraries:
# ==============================================================================
if(ALICEVISION_USE_INTERNAL_OSI AND ALICEVISION_USE_INTERNAL_CLP AND ALICEVISION_USE_INTERNAL_COINUTILS)
  add_subdirectory(dependencies/osi_clp/)
  set_property(TARGET lib_clp PROPERTY FOLDER AliceVision/Dependencies/Lib_clp)
  set_property(TARGET lib_CoinUtils PROPERTY FOLDER AliceVision/Dependencies/Lib_CoinUtils)
  set_property(TARGET lib_Osi PROPERTY FOLDER AliceVision/Dependencies/Lib_Osi)
  set_property(TARGET lib_OsiClpSolver PROPERTY FOLDER AliceVision/Dependencies/Lib_OsiClpSolver)
endif()

# ==============================================================================
# Lemon
# ==============================================================================
# - internal by default (Lemon),
# - external if LEMON_INCLUDE_DIR_HINTS and a valid Lemon setup is found
# ==============================================================================
if(NOT DEFINED LEMON_INCLUDE_DIR_HINTS)
  set(LEMON_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/lemon)
  set(ALICEVISION_USE_INTERNAL_LEMON ON)
endif()
find_package(Lemon QUIET)
if(NOT LEMON_FOUND OR ALICEVISION_USE_INTERNAL_LEMON)
  set(LEMON_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/lemon
    ${CMAKE_CURRENT_BINARY_DIR}/dependencies/lemon)
  set(LEMON_LIBRARY lemon)
endif()

# ==============================================================================
# OpenCV
# ==============================================================================
# - only external and enabled only if ALICEVISION_USE_OPENCV is set to ON
# ==============================================================================
if(ALICEVISION_USE_OPENCV STREQUAL "OFF")
  set(ALICEVISION_HAVE_OPENCV 0)
  set(ALICEVISION_HAVE_OCVSIFT 0)
else() # ON OR AUTO
  find_package(OpenCV COMPONENTS core imgproc video imgcodecs videoio features2d xfeatures2d)

  if(OpenCV_FOUND)
    set(ALICEVISION_HAVE_OPENCV 1)
    message(STATUS "OpenCV found.")
  elseif(ALICEVISION_USE_OPENCV STREQUAL "ON")
    set(ALICEVISION_HAVE_OPENCV 0)
    message(SEND_ERROR "Failed to find OpenCV.")
  else()
    set(ALICEVISION_HAVE_OPENCV 0)
  endif()
endif()

if(ALICEVISION_HAVE_OPENCV)
  include_directories(${OpenCV_INCLUDE_DIRS})
  # add a definition that allows the conditional compiling
  if(ALICEVISION_USE_OCVSIFT)
    set(ALICEVISION_HAVE_OCVSIFT 1)
  else()
    set(ALICEVISION_HAVE_OCVSIFT 0)
  endif()
endif()

# ==============================================================================
# Boost
# ==============================================================================
option(BOOST_NO_CXX11 "if Boost is compiled without C++11 support (as it is often the case in OS packages) this must be enabled to avoid symbol conflicts (SCOPED_ENUM)." OFF)
find_package(Boost 1.53.0 QUIET COMPONENTS system filesystem program_options thread serialization log log_setup)

if(Boost_FOUND)
  message(STATUS "Boost ${Boost_LIB_VERSION} found.")
else()
  message(SEND_ERROR "Failed to find Boost.")
endif()

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

# ==============================================================================
# Alembic
# ==============================================================================
# - optional, it allows to use the classes to export data in alembic format
# ==============================================================================
if(ALICEVISION_USE_ALEMBIC STREQUAL "OFF")
  set(ALICEVISION_HAVE_ALEMBIC 0)
else() # ON OR AUTO
  find_package(Alembic)

  if(ALEMBIC_FOUND)
    set(ALICEVISION_HAVE_ALEMBIC 1)
    message(STATUS "Alembic found.")
  elseif(ALICEVISION_USE_ALEMBIC STREQUAL "ON")
    set(ALICEVISION_HAVE_ALEMBIC 0)
    message(SEND_ERROR "Failed to find Alembic.")
  else()
    set(ALICEVISION_HAVE_ALEMBIC 0)
  endif()
endif()

if(ALICEVISION_HAVE_ALEMBIC)
  include_directories(${ABC_INCLUDE_DIR})
endif()

# ==============================================================================
# Dependencies libraries:
# ==============================================================================
add_subdirectory(dependencies)

# ==============================================================================
# CCTag
# ==============================================================================
if(ALICEVISION_USE_CCTAG STREQUAL "OFF")
  set(ALICEVISION_HAVE_CCTAG 0)
else() # ON OR AUTO
  if(ALICEVISION_HAVE_OPENCV) #
    find_package(CCTag 1.0.0 CONFIG)

    if(CCTag_FOUND)
      set(ALICEVISION_HAVE_CCTAG 1)
      message(STATUS "CCTAG ${CCTag_VERSION} found.")
    elseif(ALICEVISION_USE_CCTAG STREQUAL "ON")
      set(ALICEVISION_HAVE_CCTAG 0)
      message(SEND_ERROR "Failed to find CCTAG.")
    else()
      set(ALICEVISION_HAVE_CCTAG 0)
    endif()
  elseif(ALICEVISION_USE_CCTAG STREQUAL "ON")
    set(ALICEVISION_HAVE_CCTAG 0)
    message(SEND_ERROR "Can't use CCTAG without OPENCV.")
  else()
    set(ALICEVISION_HAVE_CCTAG 0)
  endif()
endif()

# ==============================================================================
# PopSift
# ==============================================================================
if(ALICEVISION_USE_POPSIFT STREQUAL "OFF")
  set(ALICEVISION_HAVE_POPSIFT 0)
else() # ON OR AUTO
  find_package(PopSift CONFIG)

  if(PopSift_FOUND)
    set(ALICEVISION_HAVE_POPSIFT 1)
    message(STATUS "PopSIFT found.")
  elseif(ALICEVISION_USE_POPSIFT STREQUAL "ON")
    set(ALICEVISION_HAVE_POPSIFT 0)
    message(SEND_ERROR "Failed to find PopSIFT.")
  else()
    set(ALICEVISION_HAVE_POPSIFT 0)
  endif()
endif()

# ==============================================================================
# OpenGV
# ==============================================================================
# - optional, it allows to use the generic camera PnP algorithms for rig localization
# ==============================================================================
if(ALICEVISION_USE_OPENGV STREQUAL "OFF")
  set(ALICEVISION_HAVE_OPENGV 0)
else() # ON OR AUTO
  find_package(OpenGV)

  if(OPENGV_FOUND)
    set(ALICEVISION_HAVE_OPENGV 1)
    message(STATUS "OpenGV found.")
  elseif(ALICEVISION_USE_OPENGV STREQUAL "ON")
    set(ALICEVISION_HAVE_OPENGV 0)
    message(SEND_ERROR "Failed to find OpenGV.")
  else()
    set(ALICEVISION_HAVE_OPENGV 0)
  endif()
endif()

if(ALICEVISION_HAVE_OPENGV)
  include_directories(${OPENGV_INCLUDE_DIR})
  link_directories(${OPENGV_LIBRARY_DIR})
endif()

# ==============================================================================
# OpenImageIO
# ==============================================================================
# - optional, it allows to use keyframeSelection
# ==============================================================================
if(ALICEVISION_USE_OIIO STREQUAL "OFF")
  set(ALICEVISION_HAVE_OIIO 0)
else() # ON OR AUTO
  find_package(OpenImageIO)
  if(OPENIMAGEIO_FOUND)
    set(ALICEVISION_HAVE_OIIO 1)
    message(STATUS "OpenImageIO found.")
  elseif(ALICEVISION_USE_OIIO STREQUAL "ON")
    set(ALICEVISION_HAVE_OIIO 0)
    message(SEND_ERROR "Failed to find OpenImageIO.")
  else()
    set(ALICEVISION_HAVE_OIIO 0)
  endif()
endif()

# ==============================================================================
# Include directories
# ==============================================================================
set(generatedDir "${CMAKE_CURRENT_BINARY_DIR}/generated")
message("generatedDir: ${generatedDir}")
set(ALICEVISION_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ${generatedDir}
  ${JPEG_INCLUDE_DIR}
  ${PNG_INCLUDE_DIRS}
  ${TIFF_INCLUDE_DIR}
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

include_directories(${ALICEVISION_INCLUDE_DIRS})

# ==============================================================================
# aliceVision modules
# ==============================================================================
# The aliceVision library itself
add_subdirectory(aliceVision)

# ==============================================================================
# aliceVision libraries
# ==============================================================================
set(ALICEVISION_LIBRARIES
  aliceVision_dataio
  aliceVision_exif
  aliceVision_feature
  aliceVision_image
  aliceVision_kvld
#  aliceVision_linearProgramming
  aliceVision_lInftyComputerVision
  aliceVision_localization
  aliceVision_matching
  aliceVision_matchingImageCollection
  aliceVision_multiview
  aliceVision_numeric
  aliceVision_rig
  aliceVision_sfm
  aliceVision_system
  aliceVision_voctree
)

if(ALICEVISION_HAVE_OPENCV)
  list(APPEND ALICEVISION_LIBRARIES aliceVision_calibration)
endif()

foreach(omvglib ${ALICEVISION_LIBRARIES})
  set_property(TARGET ${omvglib} PROPERTY FOLDER AliceVision/AliceVision)
endforeach()

# aliceVision tutorial examples
if(ALICEVISION_BUILD_EXAMPLES)
  add_subdirectory(samples)
endif()

# Complete software(s) build on aliceVision libraries
add_subdirectory(software)

# software(s) under patent or commercial licence
# Included for research purpose only
add_subdirectory(nonFree)

# ==============================================================================
# Documentation
# --------------------------
# Sphinx detection
# ==============================================================================
if(ALICEVISION_BUILD_DOC STREQUAL "OFF")
  set(ALICEVISION_HAVE_DOC 0)
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
      PROPERTY FOLDER AliceVision
    )
    set(ALICEVISION_HAVE_DOC 1)
    message(STATUS "Sphinx found.")
  elseif(ALICEVISION_BUILD_DOC STREQUAL "ON")
    set(ALICEVISION_HAVE_DOC 0)
    message(SEND_ERROR "Failed to find Sphinx.\nSphinx need to be installed to generate the documentation")
  else()
    set(ALICEVISION_HAVE_DOC 0)
  endif()
endif()

message("\n")
message("** AliceVision version: " ${ALICEVISION_VERSION})
message("** Build Shared libs: " ${ALICEVISION_BUILD_SHARED})
message("** Build AliceVision tests: " ${ALICEVISION_BUILD_TESTS})
message("** Build AliceVision samples applications: " ${ALICEVISION_BUILD_EXAMPLES})
message("** Build AliceVision documentation: " ${ALICEVISION_HAVE_DOC})
message("** Build the keyframeSelection module: " ${ALICEVISION_HAVE_OIIO})
message("** Enable code coverage generation: " ${ALICEVISION_BUILD_COVERAGE})
message("** Enable OpenMP parallelization: " ${ALICEVISION_HAVE_OPENMP})
message("** Build OpenCV+AliceVision samples programs: " ${ALICEVISION_HAVE_OPENCV})
message("** Use OpenCV SIFT features: " ${ALICEVISION_HAVE_OCVSIFT})
message("** Use CCTAG markers: " ${ALICEVISION_HAVE_CCTAG})
message("** Build Alembic exporter: " ${ALICEVISION_HAVE_ALEMBIC})
message("** Use OpenGV for rig localization: " ${ALICEVISION_HAVE_OPENGV})
message("** Use log: " ${ALICEVISION_USE_LOG})
message("\n")

if(ALICEVISION_USE_INTERNAL_EIGEN)
  message(STATUS "EIGEN: " ${EIGEN_VERSION} " (internal)")
else()
  message(STATUS "EIGEN: " ${EIGEN_VERSION} " (external)")
endif()

if(ALICEVISION_USE_INTERNAL_CERES)
  message(STATUS "CERES: " ${CERES_VERSION} " (internal)")
else()
  message(STATUS "CERES: " ${CERES_VERSION} " (external)")
endif()

if(ALICEVISION_USE_INTERNAL_FLANN)
  message(STATUS "FLANN: " ${FLANN_VERSION} " (internal)")
else()
  message(STATUS "FLANN: " ${FLANN_VERSION} " (external)")
endif()

if(ALICEVISION_USE_INTERNAL_TIFF)
  message(STATUS "LIBTIFF: " ${TIFF_VERSION_STRING} " (internal)")
else()
  message(STATUS "LIBTIFF: " ${TIFF_VERSION_STRING} " (external)")
endif()

if(ALICEVISION_USE_INTERNAL_PNG)
  message(STATUS "LIBPNG: " ${PNG_VERSION_STRING} " (internal)")
else()
  message(STATUS "LIBPNG: " ${PNG_VERSION_STRING} " (external)")
endif()

if(ALICEVISION_USE_INTERNAL_JPEG)
  message(STATUS "LIBJPEG (internal)")
else()
  message(STATUS "LIBJPEG (external)")
endif()

if(ALICEVISION_USE_INTERNAL_CLP)
  message(STATUS "CLP: " ${CLP_VERSION} " (internal)")
else()
  message(STATUS "CLP: " ${CLP_VERSION} " (external)")
endif()

if(ALICEVISION_USE_INTERNAL_COINUTILS)
  message(STATUS "COINUTILS: " ${COINUTILS_VERSION} " (internal)")
else()
  message(STATUS "COINUTILS: " ${COINUTILS_VERSION} " (external)")
endif()

if(ALICEVISION_USE_INTERNAL_OSI)
  message(STATUS "OSI: " ${OSI_VERSION} " (internal)")
else()
  message(STATUS "OSI: " ${OSI_VERSION} " (external)")
endif()

if(ALICEVISION_USE_INTERNAL_LEMON)
  message(STATUS "LEMON: " ${LEMON_VERSION} " (internal)")
else()
  message(STATUS "LEMON: " ${LEMON_VERSION} " (external)")
endif()

message("\n")

# ==============================================================================
# INSTALL RULES
# ==============================================================================

install(EXPORT aliceVision-targets DESTINATION lib)
install(EXPORT aliceVision-targets
        DESTINATION share/aliceVision/cmake FILE AliceVisionTargets.cmake)

#Adapt build include paths to install path
set(ALICEVISION_INCLUDE_DIRS
  "${ALICEVISION_INCLUDE_DIRS}"
  "${CMAKE_INSTALL_PREFIX}/include/aliceVision")

list(REMOVE_ITEM ALICEVISION_INCLUDE_DIRS ${generatedDir})
string(REGEX REPLACE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_INSTALL_PREFIX}/include"
  ALICEVISION_INCLUDE_DIRS
  "${ALICEVISION_INCLUDE_DIRS}"
)

string(REGEX REPLACE
  "dependencies"
  "aliceVision_dependencies"
  ALICEVISION_INCLUDE_DIRS
  "${ALICEVISION_INCLUDE_DIRS}"
)

if(ALICEVISION_USE_INTERNAL_CERES)
  string(REGEX REPLACE
    "ceres-solver"
    "ceres"
    ALICEVISION_INCLUDE_DIRS
    "${ALICEVISION_INCLUDE_DIRS}"
  )
  string(REGEX REPLACE
    "internal/ceres/"
    ""
    ALICEVISION_INCLUDE_DIRS
    "${ALICEVISION_INCLUDE_DIRS}"
  )
endif()

# Create a AliceVisionConfig.cmake file. <name>Config.cmake files are searched by
# find_package() automatically. We configure that file so that we can put any
# information we want in it, e.g. version numbers, include directories, etc.
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmakeFindModules/AliceVisionConfig.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/AliceVisionConfig.cmake" @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/AliceVisionConfig.cmake"
        DESTINATION share/aliceVision/cmake)

# create the config.hpp file containing all the preprocessor definitions
set(configfile "${generatedDir}/aliceVision/config.hpp")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmakeFindModules/config.hpp.in"
               "${configfile}" @ONLY)

install(FILES "${configfile}"
        DESTINATION "include/aliceVision")

########### Add uninstall target ###############
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmakeFindModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmakeFindModules/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmakeFindModules/cmake_uninstall.cmake")
