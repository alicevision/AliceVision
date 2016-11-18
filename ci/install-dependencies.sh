#!/bin/bash
set -e
CURRDIR="$( cd "$( dirname "$( readlink -f "${BASH_SOURCE[0]}" )" )" && pwd )"
. "${CURRDIR}/env.sh"


# downloadFromPopart TARGET_FULL_NAME INSTALL_PATH
downloadFromPopart()
{
    download_files_from_tar "https://github.com/poparteu/popart-dependencies/releases/download/$1/$1.tgz" $2
    return 0
}

set -x

downloadFromPopart eigen-3.2.8 ${DEPS_INSTALL_PATH}
downloadFromPopart ceres-1.11.0 ${DEPS_INSTALL_PATH}
downloadFromPopart opencv-3.0.0 ${DEPS_INSTALL_PATH}
downloadFromPopart opengv-dev ${DEPS_INSTALL_PATH}

