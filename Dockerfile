ARG CUDA_TAG=7.0
ARG OS_TAG=7
ARG NPROC=1
FROM nvidia/cuda:${CUDA_TAG}-devel-centos${OS_TAG}
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
# - file and openssl are needed for cmake
RUN yum -y install \
        file \
        build-essential \
        make \
        git \
        wget \
        unzip \
        yasm \
        pkg-config \
        libtool \
        nasm \
        automake \
        openssl-devel \
        gcc-gfortran

# Manually install cmake 3.11
WORKDIR /opt
RUN wget https://cmake.org/files/v3.11/cmake-3.11.0.tar.gz && tar zxvf cmake-3.11.0.tar.gz && cd cmake-3.11.0 && ./bootstrap --prefix=/usr/local  -- -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_USE_OPENSSL:BOOL=ON && make -j8 && make install

ENV AV_DEV=/opt/AliceVision_git \
    AV_BUILD=/tmp/AliceVision_build \
    AV_INSTALL=/opt/AliceVision_install \
    AV_BUNDLE=/opt/AliceVision_bundle \
    PATH="${PATH}:${AV_BUNDLE}"

COPY . "${AV_DEV}"

WORKDIR "${AV_BUILD}"
RUN cmake "${AV_DEV}" -DCMAKE_BUILD_TYPE=Release -DALICEVISION_BUILD_DEPENDENCIES:BOOL=ON -DINSTALL_DEPS_BUILD:BOOL=ON -DCMAKE_INSTALL_PREFIX="${AV_INSTALL}" -DALICEVISION_BUNDLE_PREFIX="${AV_BUNDLE}"

WORKDIR "${AV_BUILD}"
RUN make -j${NPROC} install && make bundle
# && cd /opt && rm -rf "${AV_BUILD}"

WORKDIR "${AV_BUNDLE}/share/aliceVision"
RUN wget https://gitlab.com/alicevision/trainedVocabularyTreeData/raw/master/vlfeat_K80L3.SIFT.tree

