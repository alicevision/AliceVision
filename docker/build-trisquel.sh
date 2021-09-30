#!/bin/bash

set -ex

test -e docker/fetch.sh || {
	echo This script must be run from the top level of the AliceVision tree
	exit 1
}

test -z "$AV_VERSION" && AV_VERSION="$(git rev-parse --abbrev-ref HEAD)-$(git rev-parse --short HEAD)"
test -z "$TRISQUEL_VERSION" && TRISQUEL_VERSION=9
test -z "$UBUNTU_CODENAME" && UBUNTU_CODENAME=bionic
 
echo "AV_VERSION: $AV_VERSION"
echo "TRISQUEL_VERSION: $TRISQUEL_VERSION"
echo "UBUNTU_CODENAME: $UBUNTU_CODENAME"

docker/fetch.sh

## DEPENDENCIES
docker build \
    --rm \
    --build-arg TRISQUEL_VERSION=${TRISQUEL_VERSION} \
    --build-arg UBUNTU_CODENAME=${UBUNTU_CODENAME} \
    --tag alicevision/alicevision-deps:${AV_VERSION}-trisquel${TRISQUEL_VERSION} \
    -f docker/Dockerfile_trisquel_deps .

## ALICEVISION
docker build \
    --rm \
    --build-arg TRISQUEL_VERSION=${TRISQUEL_VERSION} \
	--build-arg AV_VERSION=${AV_VERSION} \
    --tag alicevision/alicevision:${AV_VERSION}-trisquel${TRISQUEL_VERSION} \
    -f docker/Dockerfile_trisquel .


