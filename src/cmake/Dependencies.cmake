# =============================================================================
# Dependencies.cmake — AliceVision superbuild orchestrator
#
# Layout:
#   cmake/
#   ├── Dependencies.cmake          ← this file
#   ├── versions.cmake              ← all versions / URLs / hashes (DEP_*_VERSION,
#   │                                 DEP_*_URL, DEP_*_HASH, DEP_*_GIT_REPO, DEP_*_GIT_TAG)
#   ├── macros.cmake                ← av_add_cmake_dep / av_register_dep
#   └── deps/
#       ├── core.cmake              ← zlib, tbb, eigen, expat, boost, pybind11, swig
#       ├── cuda.cmake              ← cuda toolkit
#       ├── image_codecs.cmake      ← tiff, png, jpeg, libraw, openexr
#       ├── video.cmake             ← vpx, ffmpeg
#       ├── color_image.cmake       ← onnxruntime, opencolorio, openimageio, opencv
#       ├── math_solvers.cmake      ← lapack, suitesparse, ceres, flann/lz4,
#       │                             nanoflann, coin/osi/clp, lemon
#       ├── geometry.cmake          ← geogram, assimp, alembic, e57format,
#       │                             openmesh, pcl, usd
#       └── feature_detectors.cmake ← popsift, cctag, apriltag
#
# Include strategy:
#   Each deps/*.cmake is included conditionally — only when at least one of its
#   AV_BUILD_* options is ON. Each file still guards individual targets with
#   its own if(AV_BUILD_*), so fine-grained control is preserved within a group.
#
# Variable naming (versions.cmake):
#   DEP_<LIB>_VERSION  — version string
#   DEP_<LIB>_URL      — full archive URL
#   DEP_<LIB>_HASH     — checksum (ALGO=value)
#   DEP_<LIB>_GIT_REPO — git remote
#   DEP_<LIB>_GIT_TAG  — git tag / commit
#
# To upgrade a library: edit cmake/versions.cmake only.
# =============================================================================

include(ExternalProject)

# -----------------------------------------------------------------------------
# Parallel build degree
# -----------------------------------------------------------------------------

set(AV_BUILD_DEPENDENCIES_PARALLEL 1
    CACHE STRING "Parallel jobs for dependency builds (0 = auto-detect)"
)

if(AV_BUILD_DEPENDENCIES_PARALLEL EQUAL 0)
    cmake_host_system_information(
        RESULT AV_BUILD_DEPENDENCIES_PARALLEL
        QUERY  NUMBER_OF_LOGICAL_CORES
    )
endif()

# -----------------------------------------------------------------------------
# Feature flags
# -----------------------------------------------------------------------------

set(AV_ONNX_APPLE_ARCH "arm64" CACHE STRING "ONNX Runtime arch on Apple [arm64, x86_64]")

# Core
option(AV_BUILD_CUDA     "Build embedded CUDA toolkit"   OFF)
option(AV_BUILD_ZLIB     "Build embedded zlib"           OFF)
option(AV_BUILD_TBB      "Build embedded TBB"            ON)
option(AV_BUILD_EIGEN    "Build embedded Eigen"          ON)
option(AV_BUILD_EXPAT    "Build embedded Expat"          ON)
option(AV_BUILD_BOOST    "Build embedded Boost"          ON)
option(AV_BUILD_SWIG     "Build embedded SWIG"           ON)
option(AV_BUILD_PYBIND11 "Build pybind11"                OFF)

# Image codecs
option(AV_BUILD_TIFF    "Build embedded libtiff"         ON)
option(AV_BUILD_PNG     "Build embedded libpng"          ON)
option(AV_BUILD_JPEG    "Build embedded libjpeg-turbo"   ON)
option(AV_BUILD_LIBRAW  "Build embedded libraw"          ON)
option(AV_BUILD_OPENEXR "Build embedded OpenEXR"         ON)

# Video
option(AV_BUILD_VPX    "Build embedded libvpx"           ON)
option(AV_BUILD_FFMPEG "Build embedded FFmpeg"           ON)

# Color / Image processing
option(AV_BUILD_ONNXRUNTIME  "Build embedded ONNX Runtime"    ON)
option(AV_BUILD_OPENCOLORIO  "Build embedded OpenColorIO"      ON)
option(AV_BUILD_OPENIMAGEIO  "Build embedded OpenImageIO"      ON)
option(AV_BUILD_OPENCV       "Build embedded OpenCV"           ON)

# Math / Solvers
option(AV_BUILD_LAPACK      "Build embedded LAPACK"           ON)
option(AV_BUILD_SUITESPARSE "Build embedded SuiteSparse"      ON)
option(AV_BUILD_CERES       "Build embedded Ceres"            ON)
option(AV_BUILD_FLANN       "Build embedded FLANN (+ lz4)"    ON)
option(AV_BUILD_NANOFLANN   "Build embedded NanoFLANN"        ON)
option(AV_BUILD_COINUTILS   "Build embedded CoinUtils"        ON)
option(AV_BUILD_OSI         "Build embedded Osi"              ON)
option(AV_BUILD_CLP         "Build embedded Clp"              ON)
option(AV_BUILD_LEMON       "Build embedded LEMON"            ON)

# 3D / Geometry
option(AV_BUILD_GEOGRAM   "Build embedded Geogram"            ON)
option(AV_BUILD_ASSIMP    "Build embedded Assimp"             ON)
option(AV_BUILD_ALEMBIC   "Build embedded Alembic"            ON)
option(AV_BUILD_E57FORMAT "Build embedded libE57Format"       ON)
option(AV_BUILD_OPENMESH  "Build embedded OpenMesh"           ON)
option(AV_BUILD_PCL       "Build embedded PCL"                OFF)
option(AV_BUILD_USD       "Build embedded USD"                OFF)

# Feature detectors
option(AV_BUILD_POPSIFT  "Build embedded PopSift"             ON)
option(AV_BUILD_CCTAG    "Build embedded CCTag"               ON)
option(AV_BUILD_APRILTAG "Build embedded AprilTag"            ON)

# -----------------------------------------------------------------------------
# Logging
# -----------------------------------------------------------------------------

message(STATUS "")
message(STATUS "=== AliceVision dependency build options ===")
foreach(_opt
    AV_BUILD_CUDA     AV_BUILD_ZLIB      AV_BUILD_TBB       AV_BUILD_EIGEN
    AV_BUILD_EXPAT    AV_BUILD_BOOST     AV_BUILD_SWIG      AV_BUILD_PYBIND11
    AV_BUILD_TIFF     AV_BUILD_PNG       AV_BUILD_JPEG      AV_BUILD_LIBRAW
    AV_BUILD_OPENEXR
    AV_BUILD_VPX      AV_BUILD_FFMPEG
    AV_BUILD_ONNXRUNTIME AV_BUILD_OPENCOLORIO AV_BUILD_OPENIMAGEIO AV_BUILD_OPENCV
    AV_BUILD_LAPACK   AV_BUILD_SUITESPARSE AV_BUILD_CERES
    AV_BUILD_FLANN    AV_BUILD_NANOFLANN
    AV_BUILD_COINUTILS AV_BUILD_OSI AV_BUILD_CLP AV_BUILD_LEMON
    AV_BUILD_GEOGRAM  AV_BUILD_ASSIMP    AV_BUILD_ALEMBIC
    AV_BUILD_E57FORMAT AV_BUILD_OPENMESH AV_BUILD_PCL AV_BUILD_USD
    AV_BUILD_POPSIFT  AV_BUILD_CCTAG     AV_BUILD_APRILTAG
    AV_USE_CUDA       AV_USE_OPENMP
    AV_BUILD_DEPENDENCIES_PARALLEL
)
    message(STATUS "  ${_opt}: ${${_opt}}")
endforeach()
if(APPLE)
    message(STATUS "  AV_ONNX_APPLE_ARCH: ${AV_ONNX_APPLE_ARCH}")
endif()
message(STATUS "")

# -----------------------------------------------------------------------------
# Build paths shared by all sub-projects
# -----------------------------------------------------------------------------

set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/external")

set(CMAKE_CORE_BUILD_FLAGS
    -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE}
    -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
    -DCMAKE_INSTALL_DO_STRIP:BOOL=${CMAKE_INSTALL_DO_STRIP}
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_STANDARD=20
)

# Accumulates all built targets; consumed by the aliceVision ExternalProject.
set(AV_DEPS "")

# -----------------------------------------------------------------------------
# Load versions / URLs / hashes — single source of truth
# -----------------------------------------------------------------------------

include(${CMAKE_CURRENT_LIST_DIR}/DepsVersions.cmake)

# -----------------------------------------------------------------------------
# Load shared macros
# -----------------------------------------------------------------------------

include(${CMAKE_CURRENT_LIST_DIR}/Helpers.cmake)

# -----------------------------------------------------------------------------
# Build dependency groups (order matters — later groups depend on earlier ones)
# -----------------------------------------------------------------------------

# ── Core ─────────────────────────────────────────────────────────────────────
if(AV_BUILD_ZLIB OR AV_BUILD_TBB OR AV_BUILD_EIGEN OR AV_BUILD_EXPAT
        OR AV_BUILD_BOOST OR AV_BUILD_SWIG OR AV_BUILD_PYBIND11)
    include(${CMAKE_CURRENT_LIST_DIR}/deps/core.cmake)
endif()

# ── CUDA ─────────────────────────────────────────────────────────────────────
if(AV_USE_CUDA)
    include(${CMAKE_CURRENT_LIST_DIR}/deps/cuda.cmake)
endif()

# ── Image codecs ─────────────────────────────────────────────────────────────
if(AV_BUILD_TIFF OR AV_BUILD_PNG OR AV_BUILD_JPEG
        OR AV_BUILD_LIBRAW OR AV_BUILD_OPENEXR)
    include(${CMAKE_CURRENT_LIST_DIR}/deps/image_codecs.cmake)
endif()

# ── Video ─────────────────────────────────────────────────────────────────────
if(AV_BUILD_VPX OR AV_BUILD_FFMPEG)
    include(${CMAKE_CURRENT_LIST_DIR}/deps/video.cmake)
endif()

# ── Color / Image processing ─────────────────────────────────────────────────
if(AV_BUILD_ONNXRUNTIME OR AV_BUILD_OPENCOLORIO
        OR AV_BUILD_OPENIMAGEIO OR AV_BUILD_OPENCV)
    include(${CMAKE_CURRENT_LIST_DIR}/deps/color_image.cmake)
endif()

# ── Math / Solvers ────────────────────────────────────────────────────────────
if(AV_BUILD_LAPACK OR AV_BUILD_SUITESPARSE OR AV_BUILD_CERES
        OR AV_BUILD_FLANN OR AV_BUILD_NANOFLANN
        OR AV_BUILD_COINUTILS OR AV_BUILD_OSI OR AV_BUILD_CLP
        OR AV_BUILD_LEMON)
    include(${CMAKE_CURRENT_LIST_DIR}/deps/math_solvers.cmake)
endif()

# ── 3D / Geometry ─────────────────────────────────────────────────────────────
if(AV_BUILD_GEOGRAM OR AV_BUILD_ASSIMP OR AV_BUILD_ALEMBIC
        OR AV_BUILD_E57FORMAT OR AV_BUILD_OPENMESH
        OR AV_BUILD_PCL OR AV_BUILD_USD)
    include(${CMAKE_CURRENT_LIST_DIR}/deps/geometry.cmake)
endif()

# ── Feature detectors ────────────────────────────────────────────────────────
if(AV_BUILD_POPSIFT OR AV_BUILD_CCTAG OR AV_BUILD_APRILTAG)
    include(${CMAKE_CURRENT_LIST_DIR}/deps/feature_detectors.cmake)
endif()

# -----------------------------------------------------------------------------
# AliceVision main build (superbuild mode)
# -----------------------------------------------------------------------------

if(AV_BUILD_ALICEVISION)
    ExternalProject_Add(aliceVision
        PREFIX          ${CMAKE_CURRENT_SOURCE_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    1
        SOURCE_DIR      ${CMAKE_CURRENT_SOURCE_DIR}/src
        BINARY_DIR      ${BUILD_DIR}/aliceVision_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DBUILD_SHARED_LIBS:BOOL=ON
            -DTARGET_ARCHITECTURE=core
            -DALICEVISION_ROOT=${ALICEVISION_ROOT}
            -DALICEVISION_USE_ALEMBIC=ON
            -DMINIGLOG=ON
            -DALICEVISION_USE_CCTAG=${AV_BUILD_CCTAG}
            -DALICEVISION_USE_APRILTAG=${AV_BUILD_APRILTAG}
            -DALICEVISION_USE_OPENCV=${AV_BUILD_OPENCV}
            -DALICEVISION_USE_POPSIFT=${AV_BUILD_POPSIFT}
            -DALICEVISION_USE_CUDA=${AV_USE_CUDA}
            -DALICEVISION_BUILD_SWIG_BINDING=${AV_USE_SWIG}
            -DALICEVISION_BUILD_DOC=OFF
            ${ZLIB_CMAKE_FLAGS}      ${ASSIMP_CMAKE_FLAGS}    ${EIGEN_CMAKE_FLAGS}
            ${OPENIMAGEIO_CMAKE_FLAGS} ${OPENEXR_CMAKE_FLAGS} ${BOOST_CMAKE_FLAGS}
            ${ALEMBIC_CMAKE_FLAGS}   ${GEOGRAM_CMAKE_FLAGS}   ${LAPACK_CMAKE_FLAGS}
            ${CERES_CMAKE_FLAGS}     ${CUDA_CMAKE_FLAGS}      ${POPSIFT_CMAKE_FLAGS}
            ${OPENCV_CMAKE_FLAGS}    ${CCTAG_CMAKE_FLAGS}     ${APRILTAG_CMAKE_FLAGS}
            ${EXPAT_CMAKE_FLAGS}     ${COINUTILS_CMAKE_FLAGS} ${OSI_CMAKE_FLAGS}
            ${CLP_CMAKE_FLAGS}       ${LZ4_CMAKE_FLAGS}       ${FLANN_CMAKE_FLAGS}
            ${NANOFLANN_CMAKE_FLAGS} ${PCL_CMAKE_FLAGS}       ${USD_CMAKE_FLAGS}
            ${SWIG_CMAKE_FLAGS}      ${E57FORMAT_CMAKE_FLAGS} ${OPENMESH_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        DEPENDS ${AV_DEPS}
    )
endif()
