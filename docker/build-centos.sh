#!/bin/bash

set -ex

test -e docker/fetch.sh || {
    echo This script must be run from the top level of the AliceVision tree
    exit 1
}

test -z "$AV_DEPS_VERSION" && AV_DEPS_VERSION=2024.02.08
test -z "$AV_VERSION" && AV_VERSION="$(git rev-parse --abbrev-ref HEAD)-$(git rev-parse --short HEAD)"
test -z "$CUDA_VERSION" && CUDA_VERSION=11.3.1
test -z "$CENTOS_VERSION" && CENTOS_VERSION=7
test -z "$REPO_OWNER" && REPO_OWNER=alicevision
test -z "$DOCKER_REGISTRY" && DOCKER_REGISTRY=docker.io


echo "AV_VERSION: $AV_VERSION"
echo "AV_DEPS_VERSION: $AV_DEPS_VERSION"
echo "CUDA_VERSION: $CUDA_VERSION"
echo "CENTOS_VERSION: $CENTOS_VERSION"

echo "--== FETCH DEPENDENCIES ==--"

docker/fetch.sh

DEPS_DOCKER_TAG=${REPO_OWNER}/alicevision-deps:${AV_DEPS_VERSION}-centos${CENTOS_VERSION}-cuda${CUDA_VERSION}

echo "--== BUILD DEPENDENCIES ==--"

## DEPENDENCIES
docker build \
    --progress plain \
    --rm \
    --build-arg CUDA_VERSION=${CUDA_VERSION} \
    --build-arg CENTOS_VERSION=${CENTOS_VERSION} \
    --tag ${DEPS_DOCKER_TAG} \
    -f docker/Dockerfile_centos_deps .

echo ""
echo "  To upload results:"
echo "docker push ${DEPS_DOCKER_TAG}"
echo ""


DOCKER_TAG=${REPO_OWNER}/alicevision:${AV_VERSION}-centos${CENTOS_VERSION}-cuda${CUDA_VERSION}

echo "--== BUILD ALICEVISION ==--"

## ALICEVISION
docker build \
    --progress plain \
    --rm \
    --build-arg CUDA_VERSION=${CUDA_VERSION} \
    --build-arg CENTOS_VERSION=${CENTOS_VERSION} \
    --build-arg AV_DEPS_VERSION=${AV_DEPS_VERSION} \
    --build-arg AV_VERSION=${AV_VERSION} \
    --tag ${DOCKER_TAG} \
    -f docker/Dockerfile_centos .

echo ""
echo "  To upload results:"
echo ""
echo "docker push ${DEPS_DOCKER_TAG}"
echo "docker push ${DOCKER_TAG}"
echo ""

