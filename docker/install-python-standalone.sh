#!/bin/bash
# Install the relocatable release Python runtime (python-build-standalone) into a prefix.
# Baseline x86_64 GNU/Linux, glibc >= 2.17.
#
# This is the interpreter the dependencies and AliceVision are built against, and the one
# embedded in the bundle. Windows uses the same release through vcpkg/overlay_ports/python3.
# Pin version, release and checksum together; never resolve "latest" at build time.
# Use the full archive to retain the upstream licensing texts and metadata.
# This release omits LICENSE.zlib-ng.txt referenced by its metadata; fetch the
# license from the exact dependency tag in its downloads.json, checksum-pinned.
# Keep the installation intact; build packages belong ONLY in a separate venv.
#
# Usage: install-python-standalone.sh [prefix]   (default: /opt/python)
set -euo pipefail

PYTHON_VERSION="3.11.17"
PBS_RELEASE="20261001"
PBS_ARCHIVE="cpython-${PYTHON_VERSION}+${PBS_RELEASE}-x86_64-unknown-linux-gnu-pgo+lto-full.tar.zst"
PBS_SHA256="a98646d6a9b5366be6af3f8cfb1f15abc1f7841472edf326328342ac2dc23b0d"
ZLIB_NG_LICENSE_TAG="zlib-ng-2.2.4"
ZLIB_NG_LICENSE_SHA256="6c9f0d975b41afaa34d22f55bb8986ce69e5cb7ad327cb2b28820cd425edf5ee"

prefix="${1:-/opt/python}"

test "$(uname -m)" = x86_64

work=$(mktemp -d)
trap 'rm -rf "${work}"' EXIT

wget -q "https://github.com/astral-sh/python-build-standalone/releases/download/${PBS_RELEASE}/${PBS_ARCHIVE}" \
    -O "${work}/python.tar.zst"
echo "${PBS_SHA256}  ${work}/python.tar.zst" | sha256sum -c -
tar --zstd -xf "${work}/python.tar.zst" -C "${work}"

mkdir -p "${prefix}"
cp -a "${work}/python/install/." "${prefix}/"
cp -a "${work}/python/licenses" "${prefix}/licenses"
cp -a "${work}/python/PYTHON.json" "${prefix}/PYTHON.json"

wget -q "https://raw.githubusercontent.com/python/cpython-source-deps/${ZLIB_NG_LICENSE_TAG}/LICENSE.md" \
    -O "${prefix}/licenses/LICENSE.zlib-ng.txt"
echo "${ZLIB_NG_LICENSE_SHA256}  ${prefix}/licenses/LICENSE.zlib-ng.txt" | sha256sum -c -
