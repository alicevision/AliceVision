export CUDA_VERSION_MAJOR="8"
export CUDA_VERSION_MINOR="0"
export CUDA_VERSION_MICRO="61-1"
export CUDA_PKG_LONGVERSION="${CUDA_VERSION_MAJOR}.${CUDA_VERSION_MINOR}.${CUDA_VERSION_MICRO}"
export CUDA_PKG_VERSION="${CUDA_VERSION_MAJOR}-${CUDA_VERSION_MINOR}"
export CUDA_REPO_PKG=cuda-repo-ubuntu1404_${CUDA_PKG_LONGVERSION}_amd64.deb

wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/${CUDA_REPO_PKG}
sudo dpkg -i ${CUDA_REPO_PKG}
rm ${CUDA_REPO_PKG}
sudo apt-get -y update
sudo apt-get install -y --no-install-recommends  cuda-core-$CUDA_PKG_VERSION  cuda-cudart-dev-$CUDA_PKG_VERSION  cuda-cublas-dev-$CUDA_PKG_VERSION cuda-curand-dev-$CUDA_PKG_VERSION
sudo ln -s /usr/local/cuda-${CUDA_VERSION_MAJOR}.${CUDA_VERSION_MINOR} /usr/local/cuda
