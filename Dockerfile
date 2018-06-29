ARG CUDA_TAG=9.2-devel
FROM nvidia/cuda:$CUDA_TAG
LABEL maintainer="AliceVision Team alicevision@googlegroups.com"

# use CUDA_TAG to select the image version to use
# see https://hub.docker.com/r/nvidia/cuda/
#
# For example, to create a ubuntu 16.04 with cuda 8.0 for development, use
# docker build --build-arg CUDA_TAG=8.0-devel --tag alicevision .
#
# then execute with nvidia docker (https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0))
# docker run -it --runtime=nvidia alicevision


# OS/Version (FILE): cat /etc/issue.net
# Cuda version (ENV): $CUDA_VERSION

# System update
RUN apt-get clean && apt-get update && apt-get install -y --no-install-recommends\
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
				zlib1g-dev \
 && rm -rf /var/lib/apt/lists/*

COPY . /opt/alicevision
WORKDIR /opt/alicevision/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release -DALICEVISION_BUILD_DEPENDENCIES:BOOL=ON && make aliceVision -j
# temporary fix, there is maybe something to do with the rpath in cmake
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
