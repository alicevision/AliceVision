#!/bin/bash
set -e
CURRDIR="$( cd "$( dirname "$( readlink -f "${BASH_SOURCE[0]}" )" )" && pwd )"

. "${CURRDIR}/env.sh"


# EIGEN
if folder_not_empty "$EIGEN_INSTALL"; then
  echo "Eigen found in cache."
else
  echo "Download Eigen."
  hg clone -r "${EIGEN_VERSION}" https://bitbucket.org/eigen/eigen/ "$EIGEN_INSTALL"
fi

# OPENCV
if folder_not_empty "$OPENCV_INSTALL"; then
  echo "OpenCV found in cache."
else
  echo "Download OpenCV."
  mkdir --parent "$OPENCV_CONTRIB"
  mkdir --parent "$OPENCV_BUILD"
  mkdir --parent "$OPENCV_INSTALL"
  if folder_empty "$OPENCV_SOURCE"; then
    git clone --recursive --branch "${OPENCV_VERSION}" --depth 1 https://github.com/Itseez/opencv.git "$OPENCV_SOURCE"
  fi
  if folder_empty "$OPENCV_CONTRIB"; then
    git clone --branch "${OPENCV_VERSION}" --depth 1  https://github.com/Itseez/opencv_contrib.git "$OPENCV_CONTRIB"
  fi

  echo "Build OpenCV."
  cd $OPENCV_BUILD
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$OPENCV_INSTALL" \
    -DOPENCV_EXTRA_MODULES_PATH="$OPENCV_CONTRIB/modules" \
    "$OPENCV_SOURCE"
  make -j 2 > /dev/null
  make install
fi

# OPENGV
if folder_not_empty "$OPENGV_INSTALL"; then
  echo "OPENGV found in cache."
else
  echo "Download OpenGV."
  mkdir --parent "$OPENGV_BUILD"
  mkdir --parent "$OPENGV_INSTALL"
  if folder_empty "$OPENGV_SOURCE"; then
    git clone --depth 1 https://github.com/laurentkneip/opengv.git "$OPENGV_SOURCE"
  fi

  echo "Build OpenGV."
  cd $OPENGV_BUILD
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$OPENGV_INSTALL" \
    -DINSTALL_OPENGV=ON \
    -DEIGEN_INCLUDE_DIR="$EIGEN_INSTALL" \
    "$OPENGV_SOURCE"
  make -j 2 > /dev/null
  make install
fi

# # BOOST
# if folder_not_empty "$BOOST_INSTALL"; then
#   echo "Boost found in cache."
# else
#   echo "Download Boost."
#   mkdir --parent "$BOOST_INSTALL"
#   cd "$BOOST_ROOT"
#   download_files_from_tar "https://sourceforge.net/projects/boost/files/boost/${BOOST_VERSION}/boost_${BOOST_VERSION_FILENAME}.tar.gz" "$BOOST_SOURCE"

#   echo "Build Boost."
#   cd "$BOOST_SOURCE"
#   ./bootstrap.sh --with-libraries=filesystem,program_options,graph,serialization,thread,log --prefix="$BOOST_INSTALL"
#   ./b2 link=shared install > /dev/null
# fi

# # SUITESPARSE		
# if folder_not_empty "$SS_INSTALL"; then		
#   echo "SuiteSparse found in cache."		
# else		
#   echo "Download SuiteSparse."		
#   mkdir --parent "$SS_INSTALL"		
#   cd "$SS_ROOT"		
#   download_files_from_tar "http://faculty.cse.tamu.edu/davis/SuiteSparse/SuiteSparse-${SS_VERSION}.tar.gz" "$SS_SOURCE"		
# 		
#   echo "Build SuiteSparse."		
#   cd "$SS_SOURCE"		
#   make -j 2		
#   make install INSTALL="$SS_INSTALL"		
# fi		

# CERES
if folder_not_empty "$CERES_INSTALL"; then
  echo "Ceres found in cache."
else
  echo "Download Ceres."
  mkdir --parent "$CERES_BUILD"
  mkdir --parent "$CERES_INSTALL"
  cd "$CERES_ROOT"
  download_files_from_tar "http://ceres-solver.org/ceres-solver-${CERES_VERSION}.tar.gz" "$CERES_SOURCE"

  echo "Build Ceres."
  cd "$CERES_BUILD"
  cmake \
    -DCMAKE_INSTALL_PREFIX=$CERES_INSTALL \
    -DEIGEN_INCLUDE_DIR=$EIGEN_INSTALL \
    -DMINIGLOG=ON \
    $CERES_SOURCE
  make -j 2 > /dev/null
  make install
fi

