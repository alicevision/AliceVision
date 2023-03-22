#!/bin/bash
set -e

test -e docker/fetch.sh || {
	echo This script must be run from the top level of the AliceVision tree
	exit 1
}

test -z "$AV_DEPS_VERSION" && AV_DEPS_VERSION=2023.03.20
test -z "$AV_VERSION" && AV_VERSION="$(git rev-parse --abbrev-ref HEAD)-$(git rev-parse --short HEAD)"
test -z "$CUDA_VERSION" && CUDA_VERSION=11.3.1
test -z "$UBUNTU_VERSION" && UBUNTU_VERSION=20.04
test -z "$REPO_OWNER" && REPO_OWNER=alicevision
test -z "$DOCKER_REGISTRY" && DOCKER_REGISTRY=docker.io

./docker/fetch.sh

DEPS_DOCKER_TAG=${REPO_OWNER}/alicevision-deps:${AV_DEPS_VERSION}-centos${CENTOS_VERSION}-cuda${CUDA_VERSION}

## DEPENDENCIES
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
	--tag alicevision/alicevision-deps:${AV_VERSION}-ubuntu${UBUNTU_VERSION}-cuda${CUDA_VERSION} \
	-f docker/Dockerfile_ubuntu_deps .

echo ""
echo "  To upload results:"
echo "docker push ${DEPS_DOCKER_TAG}"
echo ""

DOCKER_TAG=${REPO_OWNER}/alicevision:${AV_VERSION}-centos${CENTOS_VERSION}-cuda${CUDA_VERSION}

## ALICEVISION
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
	--build-arg AV_VERSION=${AV_VERSION} \
	--tag alicevision/alicevision:${AV_VERSION}-ubuntu${UBUNTU_VERSION}-cuda${CUDA_VERSION} \
	-f docker/Dockerfile_ubuntu .

echo ""
echo "  To upload results:"
echo ""
echo "docker push ${DEPS_DOCKER_TAG}"
echo "docker push ${DOCKER_TAG}"
echo ""
