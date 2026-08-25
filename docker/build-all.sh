#!/usr/bin/env bash
set -euo pipefail

# Build all supported Docker images.

test -e docker/build-image.sh || {
  echo "This script must be run from the top level of the AliceVision tree" >&2
  exit 1
}

CUDA_VERSION=12.1.1 UBUNTU_VERSION=22.04 docker/build-ubuntu.sh
CUDA_VERSION=12.1.1 ROCKY_VERSION=9 docker/build-rocky.sh
