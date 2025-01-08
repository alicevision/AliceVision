#!/bin/bash
set -e

test -e docker/fetch.sh || {
	echo This script must be run from the top level of the AliceVision tree
	exit 1
}

test -z "$AV_DEPS_VERSION" && AV_DEPS_VERSION=2025.01.06
test -z "$AV_VERSION" && AV_VERSION="$(git rev-parse --abbrev-ref HEAD)-$(git rev-parse --short HEAD)"
test -z "$CUDA_VERSION" && CUDA_VERSION=12.1.1
test -z "$ROCKY_VERSION" && ROCKY_VERSION=9
test -z "$REPO_OWNER" && REPO_OWNER=alicevision
test -z "$DOCKER_REGISTRY" && DOCKER_REGISTRY=docker.io

echo "AV_VERSION: $AV_VERSION"
echo "AV_DEPS_VERSION: $AV_DEPS_VERSION"
echo "CUDA_VERSION: $CUDA_VERSION"
echo "ROCKY_VERSION: $ROCKY_VERSION"

echo "--== FETCH DEPENDENCIES ==--"

./docker/fetch.sh

DEPS_DOCKER_TAG=${REPO_OWNER}/alicevision-deps:${AV_DEPS_VERSION}-rocky${ROCKY_VERSION}-cuda${CUDA_VERSION}

echo "--== BUILD DEPENDENCIES ==--"

## DEPENDENCIES
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg ROCKY_VERSION=${ROCKY_VERSION} \
	--tag ${DEPS_DOCKER_TAG} \
	-f docker/Dockerfile_rocky_deps .

echo ""
echo "  To upload results:"
echo "docker push ${DEPS_DOCKER_TAG}"
echo ""


DOCKER_TAG=${REPO_OWNER}/alicevision:${AV_VERSION}-rocky${ROCKY_VERSION}-cuda${CUDA_VERSION}

echo "--== BUILD ALICEVISION ==--"

## ALICEVISION
docker build \
	--rm \
	--build-arg CUDA_VERSION=${CUDA_VERSION} \
	--build-arg ROCKY_VERSION=${ROCKY_VERSION} \
	--build-arg AV_DEPS_VERSION=${AV_DEPS_VERSION} \
	--build-arg AV_VERSION=${AV_VERSION} \
	--tag ${DOCKER_TAG} \
	-f docker/Dockerfile_rocky .

echo ""
echo "  To upload results:"
echo ""
echo "docker push ${DEPS_DOCKER_TAG}"
echo "docker push ${DOCKER_TAG}"
echo ""
