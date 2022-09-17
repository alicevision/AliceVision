#!/bin/bash
set -e

test -e docker/fetch.sh || {
	echo This script must be run from the top level of the AliceVision tree
	exit 1
}

test -z "$AV_VERSION" && AV_VERSION="$(git rev-parse --abbrev-ref HEAD)-$(git rev-parse --short HEAD)"
test -z "$CUDA_VERSION" && CUDA_VERSION=11.0
test -z "$UBUNTU_VERSION" && UBUNTU_VERSION=20.04

./docker/fetch.sh

## DEPENDENCIES
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
	--tag alicevision/alicevision-deps:${AV_VERSION}-ubuntu${UBUNTU_VERSION}-cuda${CUDA_VERSION} \
	-f docker/Dockerfile_ubuntu_deps .

## ALICEVISION
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
	--build-arg AV_VERSION=${AV_VERSION} \
	--tag alicevision/alicevision:${AV_VERSION}-ubuntu${UBUNTU_VERSION}-cuda${CUDA_VERSION} \
	-f docker/Dockerfile_ubuntu .
