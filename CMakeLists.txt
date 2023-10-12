cmake_minimum_required(VERSION 3.12)
project(aliceVision LANGUAGES C CXX)

option(ALICEVISION_BUILD_DEPENDENCIES "Build all AliceVision dependencies" OFF)
option(AV_BUILD_ALICEVISION "Enable building of AliceVision" ON)
option(AV_EIGEN_MEMORY_ALIGNMENT "Enable Eigen memory alignment" ON)
option(ALICEVISION_BUILD_TESTS "Build AliceVision tests" OFF)
option(AV_USE_CUDA "Enable CUDA" ON)
option(AV_USE_OPENMP "Enable OpenMP" $<$<CXX_COMPILER_ID:"AppleClang">,OFF,ON>) # disable by default for AppleClang
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type for AliceVision" FORCE)
endif()

# set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type for AliceVision")
set(DEPS_CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type for all external libraries (only used if ALICEVISION_BUILD_DEPENDENCIES=ON)")
string(TOLOWER ${DEPS_CMAKE_BUILD_TYPE} DEPS_CMAKE_BUILD_TYPE_LOWERCASE)
set(ALICEVISION_BUNDLE_PREFIX "${CMAKE_INSTALL_PREFIX}/bundle" CACHE STRING "Path for bundle installation")
set(ALICEVISION_ROOT ${PROJECT_BINARY_DIR})

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

message(STATUS "----------------------------------------")
message(STATUS "ALICEVISION_BUILD_DEPENDENCIES: ${ALICEVISION_BUILD_DEPENDENCIES}")
message(STATUS "ALICEVISION_BUILD_TESTS: ${ALICEVISION_BUILD_TESTS}")
message(STATUS "")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "DEPS_CMAKE_BUILD_TYPE: ${DEPS_CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_INSTALL_RPATH: ${CMAKE_INSTALL_RPATH}")
message(STATUS "CMAKE_INSTALL_DO_STRIP: ${CMAKE_INSTALL_DO_STRIP}")
message(STATUS "----------------------------------------")

# Eigen requires overaligned buffers for maximum efficiency (e.g. on AVX512 buffers may need to
# be aligned to 64 bytes). Fortunately this is fixed in C++17.
# See https://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html
# Only change this option to disable it for older compilers or for debug purposes.
if(NOT AV_EIGEN_MEMORY_ALIGNMENT)
    set(EIGEN_CMAKE_ALIGNMENT_FLAGS
        "-DCMAKE_CXX_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0"
        "-DCMAKE_C_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0")
endif()


# ==============================================================================
# Enable cmake UNIT TEST framework
# ==============================================================================
if(ALICEVISION_BUILD_TESTS)
    enable_testing()
endif()


if(ALICEVISION_BUILD_DEPENDENCIES)
    include(src/cmake/Dependencies.cmake)
else()

# ==============================================================================
# Standard cmake project without internal support for dependencies
# ==============================================================================

# Add sources
add_subdirectory(src)

install(
    FILES 
        LICENSE-MPL2.md 
        LICENSE-MIT-libmv.md 
        COPYING.md 
        CONTRIBUTORS.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision
)

endif()

# Bundle target (see src/cmake/MakeBundle.cmake)
# Note: require that the install rule has been executed
# Include VCPKG installed dir for runtime dependencies lookup
set(BUNDLE_LIBS_PATHS "")
if(_VCPKG_INSTALLED_DIR)
    set(BUNDLE_LIBS_PATHS ${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}$<$<CONFIG:Debug>:/debug>/bin)
endif()

add_custom_target(bundle
    ${CMAKE_COMMAND}
    -DBUNDLE_INSTALL_PREFIX=${ALICEVISION_BUNDLE_PREFIX}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DBUNDLE_LIBS_PATHS=${BUNDLE_LIBS_PATHS}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/MakeBundle.cmake
)

set_target_properties(bundle PROPERTIES FOLDER Packaging)
