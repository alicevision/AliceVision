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
downloadFromAliceVisionDependencies eigen-3.2.8 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies ceres-1.11.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies opencv-3.0.0 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies opengv-dev ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies openexr-2.2.1 ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies openimageio-master ${DEPS_INSTALL_PATH}
downloadFromAliceVisionDependencies alembic-1.7.5 ${DEPS_INSTALL_PATH}
