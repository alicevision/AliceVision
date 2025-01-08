#!/bin/sh

# Build all supported Docker images

set -e

test -e docker/fetch.sh || {
	echo This script must bu run from the top level of the AliceVision tree
	exit 1
}

CUDA_VERSION=12.1.1 UBUNTU_VERSION=22.04 docker/build-ubuntu.sh
CUDA_VERSION=12.1.1 ROCKY_VERSION=9 docker/build-rocky.sh
