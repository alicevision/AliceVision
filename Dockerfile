ARG CUDA_TAG=9.2
ARG OS_TAG=7
FROM nvidia/cuda:${CUDA_TAG}-devel-centos${OS_TAG}
LABEL maintainer="AliceVision Team alicevision-team@googlegroups.com"

# Install all compilation tools
# file: needed by cmake
RUN yum -y install file build-essential make git wget unzip yasm pkg-config libtool nasm automake openssl-devel gcc-gfortran

# Manually install cmake 3.11
RUN cd /opt && wget https://cmake.org/files/v3.11/cmake-3.11.0.tar.gz && tar zxvf cmake-3.11.0.tar.gz && cd cmake-3.11.0 && ./bootstrap --prefix=/usr/local  -- -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_USE_OPENSSL:BOOL=ON && make -j8 && make install

ENV AV_DEV=/opt/AliceVision_git \
    AV_BUILD=/tmp/AliceVision_build \
    AV_INSTALL=/opt/AliceVision_install \
    AV_BUNDLE=/opt/AliceVision_bundle \
    PATH=${PATH}:${AV_BUNDLE}

COPY . ${AV_DEV}

RUN cd ${AV_DEV} && git submodule update -i

RUN mkdir ${AV_BUILD} && cd ${AV_BUILD} && cmake ${AV_DEV} -DCMAKE_BUILD_TYPE=Release -DALICEVISION_BUILD_DEPENDENCIES:BOOL=ON -DCMAKE_INSTALL_PREFIX=${AV_INSTALL} -DALICEVISION_BUNDLE_PREFIX=${AV_BUNDLE}

RUN cd ${AV_BUILD} && make -j8 install
RUN cd ${AV_BUILD} && make bundle
RUN cd /opt && rm -rf ${AV_BUILD}


