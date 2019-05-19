#!/bin/bash
set -e
CURRDIR="$( cd "$( dirname "$( readlink -f "${BASH_SOURCE[0]}" )" )" && pwd )"
. "${CURRDIR}/env.sh"


# downloadFromAliceVisionDependencies TARGET_FULL_NAME INSTALL_PATH
downloadFromAliceVisionDependencies()
{
    download_files_from_tar "https://github.com/alicevision/AliceVisionDependencies/releases/download/$1/$1.tgz" $2
    return 0
}

set -x

downloadFromAliceVisionDependencies boost-1.66.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies eigen-3.3.7 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies ceres-1.14.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies opencv-3.4.2 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies opengv-2019.04.25 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies openexr-2.3.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies openimageio-1.8.9 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies alembic-1.7.10 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies geogram-1.6.11 ${DEPS_INSTALL_PATH}


CUDA_VERSION_MAJOR="8"
CUDA_VERSION_MINOR="0"
CUDA_PKG_LONGVERSION="${CUDA_VERSION_MAJOR}.${CUDA_VERSION_MINOR}.61-1"
CUDA_PKG_VERSION="${CUDA_VERSION_MAJOR}-${CUDA_VERSION_MINOR}"

if [ "${ALICEVISION_USE_CUDA}" = "ON" ]; then
  CUDA_REPO_PKG=cuda-repo-ubuntu1404_${CUDA_PKG_LONGVERSION}_amd64.deb
  wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/$CUDA_REPO_PKG
  sudo dpkg -i $CUDA_REPO_PKG
  rm $CUDA_REPO_PKG
  sudo apt-get -y update
  sudo apt-get install -y --no-install-recommends  cuda-core-$CUDA_PKG_VERSION  cuda-cudart-dev-$CUDA_PKG_VERSION  cuda-cublas-dev-$CUDA_PKG_VERSION cuda-curand-dev-$CUDA_PKG_VERSION
  sudo ln -s /usr/local/cuda-${CUDA_VERSION_MAJOR}.${CUDA_VERSION_MINOR} /usr/local/cuda
fi
