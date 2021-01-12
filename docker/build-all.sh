#!/bin/sh

# Build all supported Docker images

set -e

test -e docker/fetch.sh || {
	echo This script must bu run from the top level of the AliceVision tree
	exit 1
}

CUDA_VERSION=11.0 UBUNTU_VERSION=20.04 docker/build-ubuntu.sh
CUDA_VERSION=11.0 UBUNTU_VERSION=18.04 docker/build-ubuntu.sh
CUDA_VERSION=10.2 UBUNTU_VERSION=18.04 docker/build-ubuntu.sh
CUDA_VERSION=9.2 UBUNTU_VERSION=18.04 docker/build-ubuntu.sh

CUDA_VERSION=10.2 CENTOS_VERSION=7 docker/build-centos.sh
CUDA_VERSION=9.2 CENTOS_VERSION=7 docker/build-centos.sh
