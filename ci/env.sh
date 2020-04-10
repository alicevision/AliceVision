#!/bin/bash
set -e

[ -z "${TRAVIS_BUILD_DIR}" ] && echo "No TRAVIS_BUILD_DIR env variable, use current folder." && export TRAVIS_BUILD_DIR=$PWD

# Same as travis_retry:
# https://raw.githubusercontent.com/tomjaguarpaw/neil/master/travis.sh
retry() {
  local result=0
  local count=1
  while [ $count -le 3 ]; do
    [ $result -ne 0 ] && {
      echo -e "\n${ANSI_RED}The command \"$@\" failed. Retrying, $count of 3.${ANSI_RESET}\n" >&2
    }
    "$@"
    result=$?
    [ $result -eq 0 ] && break
    count=$(($count + 1))
    sleep 1
  done

  [ $count -gt 3 ] && {
    echo -e "\n${ANSI_RED}The command \"$@\" failed 3 times.${ANSI_RESET}\n" >&2
  }

  return $result
}

# Check if the folder exists and is non empty
folder_not_empty()
{
    if [ -d "$1" ] && [ "$(ls -A $1)" ]; then
        # The folder exist and is non empty
        return 0
    fi
    return 1
}

# Check if the folder doesn't exist or is empty
folder_empty()
{
    if [ -d "$1" ] && [ "$(ls $1)" ]; then
        return 1
    fi
    # The folder doesn't exist or is empty
    return 0
}

# download_files_from_tar http://path/to/archive.tar.gz /path/to/source
download_files_from_tar()
{
    mkdir --parent "$2"
    retry wget --no-check-certificate --quiet -O - "$1" | tar --strip-components=1 -xz -C "$2"
    return 0
}

export ALICEVISION_SOURCE="${TRAVIS_BUILD_DIR}"
export ALICEVISION_BUILD="${TRAVIS_BUILD_DIR}/build"
export ALICEVISION_INSTALL="${TRAVIS_BUILD_DIR}/install"
export ALICEVISION_BUILD_AS3RDPARTY="${TRAVIS_BUILD_DIR}/buildAs3rdparty"
export ALICEVISION_SAMPLE_AS3RDPARTY="${ALICEVISION_SOURCE}/src/samples/aliceVisionAs3rdParty/"
# GT datasets for tests
export GT_TEST_ROOT="${TRAVIS_BUILD_DIR}/gt_test"
export GT_TEST_SOURCE="${GT_TEST_ROOT}/gt_source"
export GT_TEST_RESULTS="${GT_TEST_ROOT}/result.json"
export GT_TEST_OUTPUT="${GT_TEST_ROOT}/gt_output"
# CMAKE
export CMAKE_VERSION_SHORT=3.11
export CMAKE_VERSION=3.11.4
export CMAKE_URL="https://cmake.org/files/v${CMAKE_VERSION_SHORT}/cmake-${CMAKE_VERSION}-Linux-x86_64.tar.gz"
export CMAKE_ROOT="${TRAVIS_BUILD_DIR}/cmake-${CMAKE_VERSION}"
export CMAKE_INSTALL="${CMAKE_ROOT}/install"

export PATH="${CMAKE_INSTALL}/bin:${PATH}"
