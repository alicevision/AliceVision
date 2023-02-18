#!/bin/bash
set -e

test -e docker/fetch.sh || {
	echo This script must be run from the top level of the AliceVision tree
	exit 1
}

test -z "$AV_VERSION" && AV_VERSION="$(git rev-parse --abbrev-ref HEAD)-$(git rev-parse --short HEAD)"
test -z "$CUDA_VERSION" && CUDA_VERSION=11.3.1
test -z "$UBUNTU_VERSION" && UBUNTU_VERSION=20.04
test -z "$REPO_OWNER" && REPO_OWNER=alicevision

./docker/fetch.sh

## DEPENDENCIES
DOCKER_TAG=${REPO_OWNER}/alicevision-deps:${AV_VERSION}-ubuntu${UBUNTU_VERSION}-cuda${CUDA_VERSION}
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
	--tag ${DOCKER_TAG} \
	-f docker/Dockerfile_ubuntu_deps .

if [ ! -z "$DOCKER_PUSH" ]; then
	docker push ${DOCKER_TAG}
fi

## ALICEVISION
DOCKER_TAG=${REPO_OWNER}/alicevision:${AV_VERSION}-ubuntu${UBUNTU_VERSION}-cuda${CUDA_VERSION}
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
	--build-arg AV_VERSION=${AV_VERSION} \
	--build-arg REPO_OWNER=${REPO_OWNER} \
	--tag ${DOCKER_TAG} \
	-f docker/Dockerfile_ubuntu .

if [ ! -z "$DOCKER_PUSH" ]; then
	docker push ${DOCKER_TAG}
fi
