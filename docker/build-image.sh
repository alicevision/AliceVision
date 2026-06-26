#!/usr/bin/env bash
set -euo pipefail

# Generic AliceVision Docker image builder, shared by build-rocky.sh and
# build-ubuntu.sh. It builds the dependencies image, then the AliceVision image.
#
# Required environment:
#   OS          rocky | ubuntu
#   OS_VERSION  e.g. 9 (rocky) or 22.04 (ubuntu)
# Optional overrides:
#   CUDA_VERSION     (default 12.1.1)
#   AV_DEPS_VERSION  (default: set by the wrapper script)
#   AV_VERSION       (default: <branch>-<short-sha>)
#   REPO_OWNER       (default alicevision)
#   DEPS_NO_CACHE    set to a non-empty value to build the deps image with --no-cache

: "${OS:?OS must be set (rocky|ubuntu)}"
: "${OS_VERSION:?OS_VERSION must be set}"

test -e "docker/Dockerfile_${OS}_deps" || {
  echo "This script must be run from the top level of the AliceVision tree" >&2
  exit 1
}

# shellcheck source=docker/container-engine.sh
. docker/container-engine.sh

: "${CUDA_VERSION:=12.1.1}"
: "${AV_DEPS_VERSION:?AV_DEPS_VERSION must be set}"
: "${REPO_OWNER:=alicevision}"

if [ -z "${AV_VERSION:-}" ]; then
  # '/' (e.g. in "feat/..." branch names) is invalid in a Docker image tag.
  av_branch=$(git rev-parse --abbrev-ref HEAD)
  AV_VERSION="${av_branch//\//-}-$(git rev-parse --short HEAD)"
fi

OS_VERSION_ARG="${OS^^}_VERSION=${OS_VERSION}"
DEPS_DOCKER_TAG="${REPO_OWNER}/alicevision-deps:${AV_DEPS_VERSION}-${OS}${OS_VERSION}-cuda${CUDA_VERSION}"
DOCKER_TAG="${REPO_OWNER}/alicevision:${AV_VERSION}-${OS}${OS_VERSION}-cuda${CUDA_VERSION}"

echo "OS:              ${OS}${OS_VERSION}"
echo "CUDA_VERSION:    ${CUDA_VERSION}"
echo "AV_VERSION:      ${AV_VERSION}"
echo "AV_DEPS_VERSION: ${AV_DEPS_VERSION}"

print_push() {
  echo ""
  echo "  To upload results:"
  for _tag in "$@"; do
    echo "  ${CONTAINER_ENGINE} push ${_tag}"
  done
  echo ""
}

deps_no_cache=()
test -n "${DEPS_NO_CACHE:-}" && deps_no_cache=(--no-cache)

echo "--== BUILD DEPENDENCIES ==--"
"${CONTAINER_ENGINE}" build "${deps_no_cache[@]}" \
  --rm \
  --progress=plain \
  --build-arg CUDA_VERSION="${CUDA_VERSION}" \
  --build-arg "${OS_VERSION_ARG}" \
  --tag "${DEPS_DOCKER_TAG}" \
  -f "docker/Dockerfile_${OS}_deps" .

print_push "${DEPS_DOCKER_TAG}"

echo "--== BUILD ALICEVISION ==--"
"${CONTAINER_ENGINE}" build \
  --rm \
  --progress=plain \
  --build-arg DEPS_IMAGE="${DEPS_DOCKER_TAG}" \
  --tag "${DOCKER_TAG}" \
  -f docker/Dockerfile .

print_push "${DEPS_DOCKER_TAG}" "${DOCKER_TAG}"
