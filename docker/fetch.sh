#!/bin/bash
# Download as many external dependencies as possible to minimise
# the amount of repeated downloads when building Docker images.
#
# Downloaded files are stored in 'dl/' at the top of the AliceVision
# source tree and copied into the Docker environments at image build time.
set -e

test -e docker/fetch.sh || {
    echo "This script must be run from the top level of the AliceVision tree"
    exit 1
}

# ---------------------------------------------------------------------------
# Static assets
# ---------------------------------------------------------------------------
test -d dl || mkdir dl

test -f dl/vlfeat_K80L3.SIFT.tree || \
    wget https://gitlab.com/alicevision/trainedVocabularyTreeData/raw/master/vlfeat_K80L3.SIFT.tree \
         -O dl/vlfeat_K80L3.SIFT.tree

test -f dl/sphereDetection_Mask-RCNN.onnx || \
    wget https://gitlab.com/alicevision/SphereDetectionModel/-/raw/main/sphereDetection_Mask-RCNN.onnx \
         -O dl/sphereDetection_Mask-RCNN.onnx

test -f dl/fcn_resnet50.onnx || \
    wget https://gitlab.com/alicevision/semanticSegmentationModel/-/raw/main/fcn_resnet50.onnx \
         -O dl/fcn_resnet50.onnx

if [ ! -d dl/ColorChartDetectionModel ]; then
    git clone --depth=1 --branch=main \
        https://gitlab.com/alicevision/ColorchartDetectionModel.git \
        dl/ColorChartDetectionModel
    rm -f  dl/ColorChartDetectionModel/README.md
    rm -rf dl/ColorChartDetectionModel/.git
fi

# CMake tarball (used inside the Docker image build)
#CMAKE_VERSION=3.26.0
#CMAKE_VERSION_MM=3.26
#test -f dl/cmake-${CMAKE_VERSION}.tar.gz || \
#    wget https://cmake.org/files/v${CMAKE_VERSION_MM}/cmake-${CMAKE_VERSION}.tar.gz \
#         -O dl/cmake-${CMAKE_VERSION}.tar.gz

# ---------------------------------------------------------------------------
# Dependency pre-fetch
# ---------------------------------------------------------------------------
test -d dl/deps || mkdir dl/deps

# Build dir for the configure-only pass
FETCH_BUILD=build-fetch

if [ ! -d "${FETCH_BUILD}" ]; then
    mkdir -p "${FETCH_BUILD}/external"
    # Symlink download cache into the expected ExternalProject download dir
    ln -s "$(pwd)/dl/deps" "${FETCH_BUILD}/external/download"
fi

pushd "${FETCH_BUILD}"

# Configure only — no build, just generate the stamp/download scripts
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DALICEVISION_BUILD_DEPENDENCIES:BOOL=ON \
    -DAV_BUILD_ALICEVISION:BOOL=OFF

# ---------------------------------------------------------------------------
# URL-based deps → download-<dep>.cmake  (under external/src/<dep>-stamp/)
# ---------------------------------------------------------------------------
# NOTE: stamp dir is now external/src/<dep>-stamp/ (aligned with BUILD_DIR
# prefix in av_add_cmake_dep).  Git-based deps do NOT have a download-*.cmake
# — they use gitclone-*.cmake instead and are handled separately below.
# ---------------------------------------------------------------------------

for dep in \
    alembic \
    assimp \
    boost \
    eigen \
    ffmpeg \
    geogram \
    gmp \
    lapack \
    mpfr \
    openexr \
    openimageio \
    png \
    suitesparse \
    tbb \
    turbojpeg \
    onnxruntime
do
    STAMP="external/src/${dep}-stamp/download-${dep}.cmake"
    if [ -f "${STAMP}" ]; then
        cmake -P "${STAMP}"
    else
        echo "WARNING: ${STAMP} not found — skipping ${dep}"
    fi
done

# OpenCV lives under a different prefix (legacy layout kept as-is)
cmake -P "opencv-prefix/src/opencv-stamp/download-opencv.cmake"
cmake -P "opencv_contrib-prefix/src/opencv_contrib-stamp/download-opencv_contrib.cmake"

# ---------------------------------------------------------------------------
# Git-based deps → gitclone-<dep>.cmake  (no download-*.cmake for these)
# ---------------------------------------------------------------------------
# tiff, ceres, expat, pybind11, lemon, coinutils, osi, clp, flann,
# nanoflann, libraw, vpx, opencolorio, e57format, popsift, cctag, apriltag
# These are cloned (shallow) at build time; nothing to pre-fetch here unless
# you want to mirror them — comment in as needed.
# ---------------------------------------------------------------------------
# cmake -P "external/src/tiff-stamp/gitclone-tiff.cmake"
# cmake -P "external/src/ceres-stamp/gitclone-ceres.cmake"

popd

rm -rf "${FETCH_BUILD}"
