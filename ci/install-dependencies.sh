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

downloadFromAliceVisionDependencies boost-1.61.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies eigen-3.3.4 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies ceres-1.13.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies opencv-3.4.2 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies opengv-2018.02.26 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies openexr-2.2.1 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies openimageio-master ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies alembic-1.7.5 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies geogram-1.6.0 ${DEPS_INSTALL_PATH}
