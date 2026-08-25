#!/usr/bin/env bash
set -euo pipefail

# Build the Ubuntu AliceVision images (deps + alicevision).
# See docker/build-image.sh for the build logic and overridable variables.

export OS=ubuntu
export OS_VERSION="${UBUNTU_VERSION:-22.04}"
export AV_DEPS_VERSION="${AV_DEPS_VERSION:-2025.09.12}"

exec docker/build-image.sh
