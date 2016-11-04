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
    if folder_empty "$2"; then
        mkdir --parent "$2"
        retry wget --no-check-certificate --quiet -O - "$1" | tar --strip-components=1 -xz -C "$2"
    fi
    return 0
}

export OPENMVG_SOURCE="${TRAVIS_BUILD_DIR}/src"
export OPENMVG_BUILD="${TRAVIS_BUILD_DIR}/build"
export OPENMVG_INSTALL="${TRAVIS_BUILD_DIR}/install"
# GT datasets for tests
export GT_TEST_ROOT="${TRAVIS_BUILD_DIR}/gt_test"
export GT_TEST_SOURCE="${GT_TEST_ROOT}/gt_source"
export GT_TEST_RESULTS="${GT_TEST_ROOT}/result.json"
export GT_TEST_OUTPUT="${GT_TEST_ROOT}/gt_output"
# CMAKE
export CMAKE_VERSION_SHORT=3.4
export CMAKE_VERSION=3.4.1
export CMAKE_URL="https://cmake.org/files/v${CMAKE_VERSION_SHORT}/cmake-${CMAKE_VERSION}-Linux-x86_64.tar.gz"
export CMAKE_ROOT="${TRAVIS_BUILD_DIR}/cmake-${CMAKE_VERSION}"
export CMAKE_INSTALL="${CMAKE_ROOT}/install"
# OPENCV
export OPENCV_VERSION=3.0.0
export OPENCV_ROOT="${TRAVIS_BUILD_DIR}/opencv-${OPENCV_VERSION}"
export OPENCV_SOURCE="${OPENCV_ROOT}/source"
export OPENCV_CONTRIB="${OPENCV_ROOT}/contrib"
export OPENCV_BUILD="${OPENCV_ROOT}/build"
export OPENCV_INSTALL="${OPENCV_ROOT}/install"
# OPENGV
export OPENGV_VERSION=dev
export OPENGV_ROOT="${TRAVIS_BUILD_DIR}/opengv-${OPENGV_VERSION}"
export OPENGV_SOURCE="${OPENGV_ROOT}/source"
export OPENGV_BUILD="${OPENGV_ROOT}/build"
export OPENGV_INSTALL="${OPENGV_ROOT}/install"
# EIGEN
export EIGEN_VERSION=3.2.8
export EIGEN_ROOT="${TRAVIS_BUILD_DIR}/eigen-${EIGEN_VERSION}"
export EIGEN_INSTALL="${EIGEN_ROOT}/install"
# BOOST
export BOOST_VERSION=1.55.0
export BOOST_VERSION_FILENAME=1_55_0
export BOOST_ROOT="${TRAVIS_BUILD_DIR}/boost-${BOOST_VERSION}"
export BOOST_SOURCE="${BOOST_ROOT}/source"
export BOOST_INSTALL="${BOOST_ROOT}/install"
## SUITESPARSE		
# export SS_VERSION=4.5.3		
# export SS_ROOT="${TRAVIS_BUILD_DIR}/suitesparse-${SS_VERSION}"		
# export SS_SOURCE="${SS_ROOT}/source"		
# export SS_INSTALL="${SS_ROOT}/install"
# CERES
export CERES_VERSION=1.11.0
export CERES_ROOT="${TRAVIS_BUILD_DIR}/ceres-${CERES_VERSION}"
export CERES_SOURCE="${CERES_ROOT}/source"
export CERES_BUILD="${CERES_ROOT}/build"
export CERES_INSTALL="${CERES_ROOT}/install"

export PATH="${CMAKE_INSTALL}/bin:${PATH}"
