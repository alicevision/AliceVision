#!/bin/bash
set -e
CURRDIR="$( cd "$( dirname "$( readlink -f "${BASH_SOURCE[0]}" )" )" && pwd )"

. ${CURRDIR}/env.sh

# CMAKE most recent version
if folder_not_empty ${CMAKE_INSTALL}; then
  echo "CMake found in cache.";
else
  echo "Download CMake.";
  download_files_from_tar ${CMAKE_URL} ${CMAKE_INSTALL}
fi
cmake --version

