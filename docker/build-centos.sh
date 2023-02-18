#!/bin/bash

set -ex

test -e docker/fetch.sh || {
	echo This script must be run from the top level of the AliceVision tree
	exit 1
}

test -z "$AV_VERSION" && AV_VERSION="$(git rev-parse --abbrev-ref HEAD)-$(git rev-parse --short HEAD)"
test -z "$CUDA_VERSION" && CUDA_VERSION=11.3.1
test -z "$CENTOS_VERSION" && CENTOS_VERSION=7
test -z "$REPO_OWNER" && REPO_OWNER=alicevision
 
echo "AV_VERSION: $AV_VERSION"
echo "CUDA_VERSION: $CUDA_VERSION"
echo "CENTOS_VERSION: $CENTOS_VERSION"

docker/fetch.sh

## DEPENDENCIES
DOCKER_TAG=${REPO_OWNER}/alicevision-deps:${AV_VERSION}-centos${CENTOS_VERSION}-cuda${CUDA_VERSION} \
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg CENTOS_VERSION=${CENTOS_VERSION} \
	--tag ${DOCKER_TAG} \
	-f docker/Dockerfile_centos_deps .

if [ ! -z "$DOCKER_PUSH" ]; then
	docker push ${DOCKER_TAG}
fi

## ALICEVISION
DOCKER_TAG=${REPO_OWNER}/alicevision:${AV_VERSION}-centos${CENTOS_VERSION}-cuda${CUDA_VERSION} \
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg CENTOS_VERSION=${CENTOS_VERSION} \
	--build-arg AV_VERSION=${AV_VERSION} \
	--tag ${DOCKER_TAG} \
	-f docker/Dockerfile_centos .

if [ ! -z "$DOCKER_PUSH" ]; then
	docker push ${DOCKER_TAG}
fi
