ARG CUDA_TAG=9.2
ARG OS_TAG=18.04
ARG NPROC=1
FROM nvidia/cuda:${CUDA_TAG}-devel-ubuntu${OS_TAG}
LABEL maintainer="AliceVision Team alicevision-team@googlegroups.com"

# use CUDA_TAG to select the image version to use
# see https://hub.docker.com/r/nvidia/cuda/
#
# CUDA_TAG=8.0-devel
# docker build --build-arg CUDA_TAG=$CUDA_TAG --tag alicevision:$CUDA_TAG .
#
# then execute with nvidia docker (https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0))
# docker run -it --runtime=nvidia alicevision


# OS/Version (FILE): cat /etc/issue.net
# Cuda version (ENV): $CUDA_VERSION

# Install all compilation tools
RUN apt-get clean && \
    apt-get update
RUN apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        wget \
        unzip \
        yasm \
        pkg-config \
        libtool \
        nasm \
        automake \
        gfortran

# rm -rf /var/lib/apt/lists/*

ENV AV_DEV=/opt/AliceVision_git \
    AV_BUILD=/tmp/AliceVision_build \
    AV_INSTALL=/opt/AliceVision_install \
    AV_BUNDLE=/opt/AliceVision_bundle \
    PATH="${PATH}:${AV_BUNDLE}"

COPY . "${AV_DEV}"


# Cannot get rig of this error on lapack build on Ubuntu, so use system libraries for lapack/suitesparse:
#CMake Error at BLAS/SRC/cmake_install.cmake:53 (file):
#  file INSTALL cannot find
#  "/tmp/AliceVision_build/external/lapack_build/lib/libblas.so.3.8.0".
#Call Stack (most recent call first):
#  BLAS/cmake_install.cmake:46 (include)
#  cmake_install.cmake:72 (include)
RUN apt-get  install -y --no-install-recommends liblapack-dev libsuitesparse-dev

WORKDIR "${AV_BUILD}"
RUN cmake "${AV_DEV}" -DCMAKE_BUILD_TYPE=Release -DALICEVISION_BUILD_DEPENDENCIES:BOOL=ON -DAV_BUILD_LAPACK:BOOL=OFF -DAV_BUILD_SUITESPARSE:BOOL=OFF -DCMAKE_INSTALL_PREFIX="${AV_INSTALL}" -DALICEVISION_BUNDLE_PREFIX="${AV_BUNDLE}"

WORKDIR "${AV_BUILD}"
RUN make -j${NPROC} install && make bundle
# && cd /opt && rm -rf "${AV_BUILD}"

WORKDIR "${AV_BUNDLE}/share/aliceVision"
RUN wget https://gitlab.com/alicevision/trainedVocabularyTreeData/raw/master/vlfeat_K80L3.SIFT.tree

