#!/bin/bash
set -ex

export AVDEPS_VERSION=2.2.8.develop
export AV_VERSION=2.2.8.develop

## DEPENDENCIES
sudo docker build --tag alicevision/alicevision-deps:${AV_VERSION}-centos7-cuda9.0 -f Dockerfile_deps .

## ALICEVISION
sudo docker build --tag alicevision/alicevision:${AV_VERSION}-centos7-cuda9.0 --build-arg AVDEPS_VERSION=${AVDEPS_VERSION} .


