cmake_minimum_required(VERSION 3.30)
project(aliceVision LANGUAGES C CXX)

include(CMakeDependentOption)

# Initialize CMAKE_OSX_ARCHITECTURES, if not specified on the command line.
if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES)
    message(STATUS "Host processor: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
    if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64")
        set(CMAKE_OSX_ARCHITECTURES "arm64")
    elseif(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(CMAKE_OSX_ARCHITECTURES "x86_64")
    else()
        message(FATAL_ERROR "CMAKE_HOST_SYSTEM_PROCESSOR was neither arm64 nor x86_64 on an Apple platform and CMAKE_OSX_ARCHITECTURES was not specified!")
    endif()
endif()

option(ALICEVISION_BUILD_DEPENDENCIES "Build all AliceVision dependencies" OFF)
option(ALICEVISION_USE_RPATH "Add RPATH on software with relative paths to libraries" ON)
option(AV_BUILD_ALICEVISION "Enable building of AliceVision" ON)
option(AV_EIGEN_MEMORY_ALIGNMENT "Enable Eigen memory alignment" ON)
option(ALICEVISION_BUILD_TESTS "Build AliceVision tests" OFF)
option(AV_USE_OPENMP "Enable OpenMP" ON) # AppleClang now supports OpenMP, if installed as an external dependency (Homebrew, MacPorts, ...)
cmake_dependent_option(AV_USE_CUDA  "Enable CUDA support" ON  "NOT APPLE" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
if(APPLE AND BUILD_SHARED_LIBS)
    option(BUILD_APPLE_FRAMEWORKS "Create Frameworks instead of plain dynamic libraries on macOS" ON)
endif()
option(ALICEVISION_INSTALL_MESHROOM_PLUGIN "Install AliceVision's plugin for Meshroom" ON)

# Global policy section
cmake_policy(SET CMP0135 NEW)    # Improve timestamp handling in ExternalProject_Add()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type for AliceVision" FORCE)
endif()

# Currently no universal binaries are supported. Fail early.
# FIXME: Enable universal builds by adapting dependency building accordingly.
if(APPLE AND CMAKE_OSX_ARCHITECTURES MATCHES "arm64" AND CMAKE_OSX_ARCHITECTURES MATCHES "x86_64")
    message(FATAL_ERROR "Building universal binaries is currently not supported. Please set 'CMAKE_OSX_ARCHITECTURES' to either arm64 (Apple Silicon) or x86_64 (Intel)!")
endif()

# set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type for AliceVision")
set(DEPS_CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type for all external libraries (only used if ALICEVISION_BUILD_DEPENDENCIES=ON)")
string(TOLOWER ${DEPS_CMAKE_BUILD_TYPE} DEPS_CMAKE_BUILD_TYPE_LOWERCASE)
set(ALICEVISION_BUNDLE_PREFIX "${CMAKE_INSTALL_PREFIX}/bundle" CACHE STRING "Path for bundle installation")
set(ALICEVISION_ROOT ${PROJECT_BINARY_DIR})
option(ALICEVISION_BUNDLE_SEARCH_LIBS_PATHS "Extra paths to search for external libraries to create the bundle installation" "")

# ==============================================================================
# GNUInstallDirs CMake module
# - Define GNU standard installation directories
# - Provides install directory variables as defined by the GNU Coding Standards.
# ==============================================================================
include(GNUInstallDirs)

if (CMAKE_BUILD_TYPE MATCHES Release)
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
if (NOT AV_EIGEN_MEMORY_ALIGNMENT)
    set(EIGEN_CMAKE_ALIGNMENT_FLAGS
        "-DCMAKE_CXX_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0"
        "-DCMAKE_C_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0")
endif()


# ==============================================================================
# Enable cmake UNIT TEST framework
# ==============================================================================
if (ALICEVISION_BUILD_TESTS)
    include(CTest)
    enable_testing()
endif()


if (ALICEVISION_BUILD_DEPENDENCIES)
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

if (ALICEVISION_INSTALL_MESHROOM_PLUGIN)
    install(
        DIRECTORY meshroom
        DESTINATION ${CMAKE_INSTALL_DATADIR}
    )
endif()

# Bundle target (see src/cmake/MakeBundle.cmake)
# Note: require that the install rule has been executed
# Include VCPKG installed dir for runtime dependencies lookup
set(BUNDLE_LIBS_PATHS "")

if(_VCPKG_INSTALLED_DIR)
    set(_vcpkg_bin_dir_release "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin")
    set(_vcpkg_bin_dir_debug   "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/debug/bin")

    # Generator expressions are not evaluated at install/bundle time,
    # so we resolve the config explicitly.
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        list(APPEND BUNDLE_LIBS_PATHS "${_vcpkg_bin_dir_debug}")
    else()
        list(APPEND BUNDLE_LIBS_PATHS "${_vcpkg_bin_dir_release}")
    endif()
endif()

if(ALICEVISION_BUNDLE_SEARCH_LIBS_PATHS)
    list(APPEND BUNDLE_LIBS_PATHS ${ALICEVISION_BUNDLE_SEARCH_LIBS_PATHS})
endif()

if(CMAKE_PREFIX_PATH)
    foreach(_prefix IN LISTS CMAKE_PREFIX_PATH)
        foreach(_libdir IN ITEMS bin lib lib64)
            set(_candidate "${_prefix}/${_libdir}")
            if(EXISTS "${_candidate}")
                list(APPEND BUNDLE_LIBS_PATHS "${_candidate}")
            endif()
        endforeach()
    endforeach()
endif()

add_custom_target(bundle
    ${CMAKE_COMMAND}
    -DBUNDLE_INSTALL_PREFIX=${ALICEVISION_BUNDLE_PREFIX}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DBUNDLE_LIBS_PATHS=${BUNDLE_LIBS_PATHS}
    -DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/MakeBundle.cmake
)

set_target_properties(bundle PROPERTIES FOLDER Packaging)
