#!/bin/bash
set -e
CURRDIR="$( cd "$( dirname "$( readlink -f "${BASH_SOURCE[0]}" )" )" && pwd )"
. "${CURRDIR}/env.sh"


# downloadFromAliceVisionDependencies TARGET_FULL_NAME INSTALL_PATH
downloadFromAliceVisionDependencies()
{
    download_files_from_tar "https://github.com/alicevision/AliceVisionDependencies/releases/download/$1/$1.tgz" $2
    return 0
}

set -x

downloadFromAliceVisionDependencies boost-1.66.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies eigen-3.3.7 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies ceres-1.14.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies opencv-3.4.2 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies opengv-2019.04.25 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies openexr-2.3.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies openimageio-2.2.11.1 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies alembic-1.7.10 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies geogram-1.6.11 ${DEPS_INSTALL_PATH}
