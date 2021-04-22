#!/bin/bash

# Download as many external dependancies as possible to minimise
# the amount of repeated downloads when building Docker images
#
# The downloaded files are stored in the 'dl' directory in the
# top of the AliceVision source, and copied into the Docker enviroments
# when the images are built

set -e

test -e docker/fetch.sh || {
	echo This script must be run from the top level of the AliceVision tree
	exit 1
}

test -d dl || \
        mkdir dl
test -f dl/vlfeat_K80L3.SIFT.tree || \
        wget https://gitlab.com/alicevision/trainedVocabularyTreeData/raw/master/vlfeat_K80L3.SIFT.tree -O dl/vlfeat_K80L3.SIFT.tree
test -f dl/cmake-3.16.3.tar.gz || \
        wget https://cmake.org/files/v3.16/cmake-3.16.3.tar.gz -O dl/cmake-3.16.3.tar.gz
test -d  dl/deps || \
	mkdir dl/deps

test -d build-fetch || {
	mkdir -p build-fetch/external
	ln -s ../../dl/deps build-fetch/external/download
}

pushd build-fetch
cmake .. \
     -DCMAKE_BUILD_TYPE=Release \
     -DALICEVISION_BUILD_DEPENDENCIES:BOOL=ON \
     -DAV_BUILD_ALICEVISION:BOOL=OFF

cmake -P "opencv-prefix/src/opencv-stamp/download-opencv.cmake"
cmake -P "opencv_contrib-prefix/src/opencv_contrib-stamp/download-opencv_contrib.cmake"

cmake -P "external/src/alembic-stamp/download-alembic.cmake"
cmake -P "external/src/boost-stamp/download-boost.cmake"
# cmake -P "external/src/ceres-stamp/download-ceres.cmake"
cmake -P "external/src/eigen-stamp/download-eigen.cmake"
cmake -P "external/src/ffmpeg-stamp/download-ffmpeg.cmake"
cmake -P "external/src/geogram-stamp/download-geogram.cmake"
cmake -P "external/src/gmp-stamp/download-gmp.cmake"
cmake -P "external/src/lapack-stamp/download-lapack.cmake"
cmake -P "external/src/mpfr-stamp/download-mpfr.cmake"
cmake -P "external/src/openexr-stamp/download-openexr.cmake"
# cmake -P "external/src/opengv-stamp/download-opengv.cmake"
cmake -P "external/src/openimageio-stamp/download-openimageio.cmake"
cmake -P "external/src/png-stamp/download-png.cmake"
# cmake -P "external/src/suitesparse-stamp/download-suitesparse.cmake"
cmake -P "external/src/tbb-stamp/download-tbb.cmake"
cmake -P "external/src/tiff-stamp/download-tiff.cmake"
cmake -P "external/src/turbojpeg-stamp/download-turbojpeg.cmake"
cmake -P "external/src/zlib-stamp/download-zlib.cmake"

popd

rm -rf build-fetch
