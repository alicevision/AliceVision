#!/usr/bin/env bash
set -euo pipefail

# Build the Rocky Linux AliceVision images (deps + alicevision).
# See docker/build-image.sh for the build logic and overridable variables.

export OS=rocky
export OS_VERSION="${ROCKY_VERSION:-9}"
export AV_DEPS_VERSION="${AV_DEPS_VERSION:-2026.03.30}"

exec docker/build-image.sh
