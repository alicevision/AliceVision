# =============================================================================
# Dependencies.cmake — AliceVision superbuild orchestrator
#
# Each external dependency lives in its own file under cmake/deps/.
# Files are included in dependency order: a lib is always included after
# all libs it depends on.
#
# Variable conventions (defined in cmake/DependenciesVersions.cmake):
#   DEP_<LIB>_VERSION   — version string
#   DEP_<LIB>_URL       — full download URL
#   DEP_<LIB>_HASH      — checksum as <ALGO>=<value>
#   DEP_<LIB>_GIT_REPO  — git remote URL
#   DEP_<LIB>_GIT_TAG   — git tag / commit
# =============================================================================

include(${CMAKE_SOURCE_DIR}/src/cmake/ConditionalOption.cmake)

include(CMakeDependentOption)
include(ExternalProject)

# ── Build options ─────────────────────────────────────────────────────────────

set(AV_BUILD_DEPENDENCIES_PARALLEL 1
    CACHE STRING "Number of cores to use when building dependencies (0 - use the number of cores of the processor)"
)

if(AV_BUILD_DEPENDENCIES_PARALLEL EQUAL 0)
    cmake_host_system_information(RESULT AV_BUILD_DEPENDENCIES_PARALLEL QUERY NUMBER_OF_LOGICAL_CORES)
endif()

set(AV_ONNX_APPLE_ARCH "${CMAKE_OSX_ARCHITECTURES}" CACHE STRING "Version to download OFF Apple [arm64, x86_64]")

# Core
option(AV_BUILD_ZLIB        "Build zlib"            ON)
option(AV_BUILD_TBB         "Build TBB"             ON)
option(AV_BUILD_EIGEN       "Build Eigen"           ON)
option(AV_BUILD_EXPAT       "Build Expat"           ON)
option(AV_BUILD_BOOST       "Build Boost"           ON)
option(AV_BUILD_PYBIND11    "Build pybind11"        ON)
option(AV_BUILD_SWIG        "Build SWIG"            ON)
av_conditional_option(AV_BUILD_OPENMP "Build LLVM OpenMP" IF APPLE THEN ON ELSE OFF)

# CUDA
cmake_dependent_option(AV_BUILD_CUDA  "Build/install CUDA toolkit"  OFF  "NOT APPLE"  OFF)

# Image codecs
option(AV_BUILD_TIFF        "Build libtiff"        ON)
option(AV_BUILD_PNG         "Build libpng"         ON)
option(AV_BUILD_JPEG        "Build libjpeg-turbo"  ON)
option(AV_BUILD_LIBRAW      "Build LibRaw"         ON)
option(AV_BUILD_OPENEXR     "Build OpenEXR"        ON)

# Video
option(AV_BUILD_VPX         "Build libvpx (ffmpeg codec)" ON)
option(AV_BUILD_FFMPEG      "Build FFmpeg"                ON)

# Color / image processing
option(AV_BUILD_ONNXRUNTIME  "Build ONNX Runtime"   ON)
option(AV_BUILD_OPENCOLORIO  "Build OpenColorIO"    ON)
option(AV_BUILD_OPENIMAGEIO  "Build OpenImageIO"    ON)
option(AV_BUILD_OPENCV       "Build OpenCV"         ON)

# Math / solvers
av_conditional_option(AV_BUILD_LAPACK      "Build LAPACK/BLAS" IF "NOT APPLE" THEN ON ELSE OFF)
av_conditional_option(AV_BUILD_SUITESPARSE "Build SuiteSparse" IF "NOT APPLE" THEN ON ELSE OFF)
option(AV_BUILD_CERES       "Build Ceres"         ON)
option(AV_BUILD_FLANN       "Build FLANN (+lz4)"  ON)
option(AV_BUILD_NANOFLANN   "Build nanoflann"     ON)
option(AV_BUILD_COINUTILS   "Build CoinUtils"     ON)
option(AV_BUILD_OSI         "Build Osi"           ON)
option(AV_BUILD_CLP         "Build Clp"           ON)
option(AV_BUILD_LEMON       "Build LEMON"         ON)

# 3D / Geometry
option(AV_BUILD_GEOGRAM     "Build Geogram"   ON)
option(AV_BUILD_ASSIMP      "Build Assimp"    ON)
option(AV_BUILD_ALEMBIC     "Build Alembic"   ON)
option(AV_BUILD_E57FORMAT   "Build E57Format" ON)
option(AV_BUILD_OPENMESH    "Build OpenMesh"  ON)
option(AV_BUILD_PCL         "Build PCL"       OFF)
option(AV_BUILD_OPENSUBDIV  "Build OpenSubdiv" ON)
option(AV_BUILD_USD         "Build USD"        ON)

# Feature detectors
cmake_dependent_option(AV_BUILD_POPSIFT       "Build PopSift" ON  "NOT APPLE" OFF)
option(AV_BUILD_CCTAG       "Build CCTag"     ON)
option(AV_BUILD_APRILTAG    "Build AprilTag"  ON)

# ── Log enabled dependencies ──────────────────────────────────────────────────

set(_all_av_options
    AV_BUILD_ZLIB AV_BUILD_TBB AV_BUILD_EIGEN AV_BUILD_EXPAT
    AV_BUILD_BOOST AV_BUILD_PYBIND11 AV_BUILD_SWIG AV_BUILD_OPENMP
    AV_BUILD_CUDA
    AV_BUILD_TIFF AV_BUILD_PNG AV_BUILD_JPEG AV_BUILD_LIBRAW AV_BUILD_OPENEXR
    AV_BUILD_VPX AV_BUILD_FFMPEG
    AV_BUILD_ONNXRUNTIME AV_BUILD_OPENCOLORIO AV_BUILD_OPENIMAGEIO AV_BUILD_OPENCV
    AV_BUILD_LAPACK AV_BUILD_SUITESPARSE AV_BUILD_CERES
    AV_BUILD_FLANN AV_BUILD_NANOFLANN
    AV_BUILD_COINUTILS AV_BUILD_OSI AV_BUILD_CLP AV_BUILD_LEMON
    AV_BUILD_GEOGRAM AV_BUILD_ASSIMP AV_BUILD_ALEMBIC AV_BUILD_E57FORMAT
    AV_BUILD_OPENMESH AV_BUILD_PCL AV_BUILD_USD AV_BUILD_OPENSUBDIV
    AV_BUILD_POPSIFT AV_BUILD_CCTAG AV_BUILD_APRILTAG
)

message(STATUS "")
message(STATUS "=== AliceVision dependency build options ===")
foreach(_opt ${_all_av_options})
    if(${_opt})
        message(STATUS "  [ON]  ${_opt}")
    else()
        message(STATUS "  [OFF] ${_opt}")
    endif()
endforeach()
message(STATUS "============================================")
message(STATUS "")

# ── Common setup ──────────────────────────────────────────────────────────────

set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/external")

string(TOLOWER "${DEPS_CMAKE_BUILD_TYPE}" DEPS_CMAKE_BUILD_TYPE_LOWERCASE)

set(CMAKE_CORE_BUILD_FLAGS
    -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_STANDARD=20
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
)

set(DEP_DEPS "")

include(${CMAKE_CURRENT_LIST_DIR}/DependenciesVersions.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/Helpers.cmake)

# ── Dependencies (included in dependency order) ───────────────────────────────

# Core — no prerequisites
include(${CMAKE_CURRENT_LIST_DIR}/deps/zlib.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/deps/tbb.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/deps/eigen.cmake)     # depends: openmp
include(${CMAKE_CURRENT_LIST_DIR}/deps/expat.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/deps/boost.cmake)      # depends: zlib
include(${CMAKE_CURRENT_LIST_DIR}/deps/pybind11.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/deps/swig.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/deps/openmp.cmake)

# CUDA — no prerequisites
include(${CMAKE_CURRENT_LIST_DIR}/deps/cuda.cmake)

# Image codecs — depend on zlib
include(${CMAKE_CURRENT_LIST_DIR}/deps/tiff.cmake)       # depends: zlib
include(${CMAKE_CURRENT_LIST_DIR}/deps/png.cmake)        # depends: zlib
include(${CMAKE_CURRENT_LIST_DIR}/deps/jpeg.cmake)       # depends: zlib
include(${CMAKE_CURRENT_LIST_DIR}/deps/libraw.cmake)     # depends: zlib
include(${CMAKE_CURRENT_LIST_DIR}/deps/openexr.cmake)    # depends: zlib

# Video — vpx before ffmpeg
include(${CMAKE_CURRENT_LIST_DIR}/deps/vpx.cmake)        # depends: (none)
include(${CMAKE_CURRENT_LIST_DIR}/deps/ffmpeg.cmake)     # depends: vpx

# Color / image processing
include(${CMAKE_CURRENT_LIST_DIR}/deps/onnxruntime.cmake)  # depends: (none)
include(${CMAKE_CURRENT_LIST_DIR}/deps/opencolorio.cmake)  # depends: expat
include(${CMAKE_CURRENT_LIST_DIR}/deps/openimageio.cmake)  # depends: boost, openexr, tiff, png, jpeg, libraw, zlib, ffmpeg, pybind11, expat, opencolorio
include(${CMAKE_CURRENT_LIST_DIR}/deps/opencv.cmake)       # depends: tbb, zlib, openexr, tiff, png, jpeg, libraw, ffmpeg, openmp

# Math / solvers
include(${CMAKE_CURRENT_LIST_DIR}/deps/lapack.cmake)       # depends: tbb
include(${CMAKE_CURRENT_LIST_DIR}/deps/suitesparse.cmake)  # depends: lapack (includes gmp + mpfr inline)
include(${CMAKE_CURRENT_LIST_DIR}/deps/ceres.cmake)        # depends: eigen, suitesparse
include(${CMAKE_CURRENT_LIST_DIR}/deps/lz4.cmake)          # depends: (none)
include(${CMAKE_CURRENT_LIST_DIR}/deps/flann.cmake)        # depends: lz4, openmp
include(${CMAKE_CURRENT_LIST_DIR}/deps/nanoflann.cmake)    # depends: (none)
include(${CMAKE_CURRENT_LIST_DIR}/deps/coinutils.cmake)    # depends: (none)
include(${CMAKE_CURRENT_LIST_DIR}/deps/osi.cmake)          # depends: coinutils
include(${CMAKE_CURRENT_LIST_DIR}/deps/clp.cmake)          # depends: coinutils, osi
include(${CMAKE_CURRENT_LIST_DIR}/deps/lemon.cmake)        # depends: (none)

# 3D / Geometry
include(${CMAKE_CURRENT_LIST_DIR}/deps/geogram.cmake)      # depends: zlib
include(${CMAKE_CURRENT_LIST_DIR}/deps/assimp.cmake)       # depends: zlib
include(${CMAKE_CURRENT_LIST_DIR}/deps/alembic.cmake)      # depends: boost, openexr, zlib
include(${CMAKE_CURRENT_LIST_DIR}/deps/e57format.cmake)    # depends: (none)
include(${CMAKE_CURRENT_LIST_DIR}/deps/openmesh.cmake)     # depends: (none)
include(${CMAKE_CURRENT_LIST_DIR}/deps/pcl.cmake)          # depends: eigen, boost, png, flann, lz4, zlib, cuda, openmp
include(${CMAKE_CURRENT_LIST_DIR}/deps/opensubdiv.cmake)   # depends: openmp
include(${CMAKE_CURRENT_LIST_DIR}/deps/usd.cmake)          # depends: (opensubdiv)

# Feature detectors
include(${CMAKE_CURRENT_LIST_DIR}/deps/popsift.cmake)      # depends: boost, cuda
include(${CMAKE_CURRENT_LIST_DIR}/deps/cctag.cmake)        # depends: boost, cuda, opencv, eigen, tbb
include(${CMAKE_CURRENT_LIST_DIR}/deps/apriltag.cmake)     # depends: (none)

# ── AliceVision (main project) ────────────────────────────────────────────────

if(AV_BUILD_ALICEVISION)
    ExternalProject_Add(aliceVision
        SOURCE_DIR        ${CMAKE_CURRENT_SOURCE_DIR}
        BINARY_DIR        ${CMAKE_CURRENT_BINARY_DIR}/aliceVision_build
        INSTALL_DIR       ${CMAKE_INSTALL_PREFIX}
        BUILD_ALWAYS      1
        CMAKE_ARGS
            ${CMAKE_CORE_BUILD_FLAGS}
            ${ZLIB_CMAKE_FLAGS}
            ${TBB_CMAKE_FLAGS}
            ${EIGEN_CMAKE_FLAGS}
            ${BOOST_CMAKE_FLAGS}
            ${SWIG_CMAKE_FLAGS}
            ${TIFF_CMAKE_FLAGS}
            ${PNG_CMAKE_FLAGS}
            ${JPEG_CMAKE_FLAGS}
            ${LIBRAW_CMAKE_FLAGS}
            ${OPENEXR_CMAKE_FLAGS}
            ${ILMBASE_CMAKE_FLAGS}
            ${FFMPEG_CMAKE_FLAGS}
            ${OPENIMAGEIO_CMAKE_FLAGS}
            ${OPENCOLORIO_CMAKE_FLAGS}
            ${OPENCV_CMAKE_FLAGS}
            ${LAPACK_CMAKE_FLAGS}
            ${SUITESPARSE_CMAKE_FLAGS}
            ${CERES_CMAKE_FLAGS}
            ${FLANN_CMAKE_FLAGS}
            ${LZ4_CMAKE_FLAGS}
            ${NANOFLANN_CMAKE_FLAGS}
            ${COINUTILS_CMAKE_FLAGS}
            ${OSI_CMAKE_FLAGS}
            ${CLP_CMAKE_FLAGS}
            ${LEMON_CMAKE_FLAGS}
            ${GEOGRAM_CMAKE_FLAGS}
            ${ASSIMP_CMAKE_FLAGS}
            ${ALEMBIC_CMAKE_FLAGS}
            ${E57FORMAT_CMAKE_FLAGS}
            ${OPENMESH_CMAKE_FLAGS}
            ${PCL_CMAKE_FLAGS}
            ${OPENSUBDIV_CMAKE_FLAGS}
            ${USD_CMAKE_FLAGS}
            ${POPSIFT_CMAKE_FLAGS}
            ${CCTAG_CMAKE_FLAGS}
            ${APRILTAG_CMAKE_FLAGS}
            ${CUDA_CMAKE_FLAGS}
            -DALICEVISION_USE_OPENCV:BOOL=${AV_BUILD_OPENCV}
            -DALICEVISION_USE_CUDA:BOOL=${AV_USE_CUDA}
            -DALICEVISION_BUILD_SWIG_BINDING:BOOL=${AV_BUILD_SWIG}
        DEPENDS ${AV_DEPS}
    )
endif()
