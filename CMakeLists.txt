cmake_minimum_required(VERSION 3.3)
project(aliceVisionAutoBuild)

option(ALICEVISION_BUILD_DEPENDENCIES "Build all AliceVision dependencies" OFF)

option(AV_BUILD_JPEG "Enable building an embedded Jpeg" ON)

if(ALICEVISION_BUILD_DEPENDENCIES)
# Use CMake ExternalProject to build all dependencies
include(ExternalProject)

set(BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/external)

# Add Geogram
if(WIN32)
   set(VORPALINE_PLATFORM Win-vs-dynamic-generic)
elseif(APPLE)
   set(VORPALINE_PLATFORM Darwin-clang-dynamic)
elseif(UNIX)
   set(VORPALINE_PLATFORM Linux64-gcc-dynamic)
endif()
ExternalProject_Add(geogram
       URL https://github.com/alicevision/geogram/archive/v1.6.0.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/geogram
       BINARY_DIR ${BUILD_DIR}/geogram_build
       INSTALL_DIR ${BUILD_DIR}/geogram_install
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release -DVORPALINE_PLATFORM=${VORPALINE_PLATFORM} -DGEOGRAM_WITH_HLBFGS=OFF -DGEOGRAM_WITH_TETGEN=OFF -DGEOGRAM_WITH_GRAPHICS=OFF -DGEOGRAM_WITH_EXPLORAGRAM=OFF -DGEOGRAM_WITH_LUA=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       BUILD_COMMAND make VERBOSE=1 -j1
       )

# Add Cuda
if(BUILD_CUDA)
ExternalProject_Add(cuda
       # URL https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cuda
       BINARY_DIR ${BUILD_DIR}/cuda_build
       INSTALL_DIR ${BUILD_DIR}/cuda_build
       CONFIGURE_COMMAND echo
       BUILD_COMMAND echo
       INSTALL_COMMAND sh <SOURCE_DIR>/cuda_8.0.61_375.26_linux.run --silent --no-opengl-libs --toolkit --toolkitpath=<INSTALL_DIR>
       )
set(CUDA_CUDART_LIBRARY "")
set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${BUILD_DIR}/cuda_build)
else()
  option(CUDA_TOOLKIT_ROOT_DIR "")
  if(CUDA_TOOLKIT_ROOT_DIR)
    set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${CUDA_TOOLKIT_ROOT_DIR})
  endif()
endif()

# Add Eigen
ExternalProject_Add(eigen
       URL https://github.com/alicevision/eigen-git-mirror/archive/3.3.4.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/eigen
       BINARY_DIR ${BUILD_DIR}/eigen_build
       INSTALL_DIR ${BUILD_DIR}/eigen_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       )

# Add ceres-solver: A Nonlinear Least Squares Minimizer
ExternalProject_Add(ceres
       URL https://github.com/ceres-solver/ceres-solver/archive/1.13.0.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver
       BINARY_DIR ${BUILD_DIR}/ceres_build
       INSTALL_DIR ${BUILD_DIR}/ceres_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release -DEigen3_DIR:PATH=${BUILD_DIR}/eigen_build/share/eigen3/cmake -DEIGEN_INCLUDE_DIR=${BUILD_DIR}/eigen_build/include/eigen3 -DMINIGLOG=ON -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS eigen
       )

# Add IlmBase
ExternalProject_Add(ilmbase
       URL https://github.com/openexr/openexr/archive/v2.2.1.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openexr
       BINARY_DIR ${BUILD_DIR}/ilmbase_build
       INSTALL_DIR ${BUILD_DIR}/openexr_build # Note: Need to install in the same folder than OpenEXR to get valid include paths
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>/IlmBase
       )

# Add OpenEXR
ExternalProject_Add(openexr
       URL https://github.com/openexr/openexr/archive/v2.2.1.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openexr
       BINARY_DIR ${BUILD_DIR}/openexr_build
       INSTALL_DIR ${BUILD_DIR}/openexr_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release -DILMBASE_PACKAGE_PREFIX=${BUILD_DIR}/openexr_build -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> <SOURCE_DIR>/OpenEXR
       DEPENDS ilmbase
       )

# Add LibTiff
ExternalProject_Add(tiff
       URL http://download.osgeo.org/libtiff/tiff-4.0.9.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tiff
       BINARY_DIR ${BUILD_DIR}/tiff_build
       INSTALL_DIR ${BUILD_DIR}/tiff_build
       CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> 
       )

# Add LibPng
ExternalProject_Add(png
       URL http://prdownloads.sourceforge.net/libpng/libpng-1.6.34.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/png
       BINARY_DIR ${BUILD_DIR}/png_build
       INSTALL_DIR ${BUILD_DIR}/png_build
       CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> 
       )

if(AV_BUILD_JPEG)
set(JPEG_TARGET turbojpeg)
# Add turbojpeg
ExternalProject_Add(${JPEG_TARGET}
       URL https://github.com/libjpeg-turbo/libjpeg-turbo/archive/1.5.3.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg
       INSTALL_DIR ${BUILD_DIR}/turbojpeg_build
       CONFIGURE_COMMAND cd <BINARY_DIR> && autoreconf -fiv && ./configure --prefix=<INSTALL_DIR> 
       )
SET(JPEG_CMAKE_FLAGS -DJPEG_LIBRARY=${BUILD_DIR}/turbojpeg_build/lib/libjpeg.so -DJPEG_INCLUDE_DIR=${BUILD_DIR}/turbojpeg_build/include)
endif()

# Add Boost
ExternalProject_Add(boost
       URL http://sourceforge.net/projects/boost/files/boost/1.61.0/boost_1_61_0.tar.bz2
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/boost
       BINARY_DIR ${BUILD_DIR}/boost_build
       INSTALL_DIR ${BUILD_DIR}/boost_build
       CONFIGURE_COMMAND cd <SOURCE_DIR> && ./bootstrap.sh --prefix=<INSTALL_DIR> --with-libraries=filesystem,system,log,regex,thread,serialization,program_options,date_time,exception,atomic,graph,math,test,container
       BUILD_COMMAND cd <SOURCE_DIR> && ./b2 --prefix=<INSTALL_DIR> variant=release link=shared threading=multi -j8
       INSTALL_COMMAND cd <SOURCE_DIR> && ./b2 ${BOOST_MODULES} install
       )

# Add OpenImageIO
ExternalProject_Add(openimageio
       URL https://github.com/OpenImageIO/oiio/archive/Release-1.8.8.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openimageio
       BINARY_DIR ${BUILD_DIR}/openimageio_build
       INSTALL_DIR ${BUILD_DIR}/openimageio_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_PREFIX_PATH="${BUILD_DIR}/png_build;${BUILD_DIR}/turbojpeg_build" -DCMAKE_BUILD_TYPE=Release -DBOOST_ROOT=${BUILD_DIR}/boost_build -DILMBASE_HOME=${BUILD_DIR}/openexr_build -DOPENEXR_HOME=${BUILD_DIR}/openexr_build -DTIFF_INCLUDE_DIR=${BUILD_DIR}/tiff_build/include -DTIFF_LIBRARIES=${BUILD_DIR}/tiff_build/lib/libtiff.so -DPNG_LIBRARY=${BUILD_DIR}/png_build/lib/libpng.so -DPNG_PNG_INCLUDE_DIR=${BUILD_DIR}/png_build/include ${JPEG_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR> -DUSE_PYTHON=OFF -DUSE_OPENCV=OFF -DUSE_OPENGL=OFF
       DEPENDS boost openexr tiff png ${JPEG_TARGET}
       )

# TODO: Raw, openjpeg
#  -DLibRaw_INCLUDE_DIR=$LIBRAW_INCLUDE_DIR
#  -DOPENJPEG_INCLUDE_DIR=$OPENJPEG_INCLUDE_DIR/openjpeg-2.0 -DOPENJPEG_OPENJP2_LIBRARIES=$OPENJPEG_OPENJP2_LIBRARIES

# Add Alembic: I/O for Point Cloud and Cameras
ExternalProject_Add(alembic
       URL https://github.com/alembic/alembic/archive/1.7.5.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/alembic
       BINARY_DIR ${BUILD_DIR}/alembic_build
       INSTALL_DIR ${BUILD_DIR}/alembic_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release -DILMBASE_ROOT=${BUILD_DIR}/openexr_build  -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS boost ilmbase
       )

ExternalProject_Add(aliceVision
       PREFIX ${CMAKE_CURRENT_SOURCE_DIR}
       BUILD_IN_SOURCE 0
       SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src
       BINARY_DIR ${BUILD_DIR}/aliceVision_build
       INSTALL_DIR ${BUILD_DIR}/aliceVision_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_PREFIX_PATH="${BUILD_DIR}/boost_build;${BUILD_DIR}/openimageio_build;${BUILD_DIR}/openexr_build;${BUILD_DIR}/openexr_build;${BUILD_DIR}/alembic_build;${BUILD_DIR}/geogram_install;${BUILD_DIR}/openimageio_build;${BUILD_DIR}/cuda_build;${BUILD_DIR}/ceres_build/lib64/cmake/Ceres;${BUILD_DIR}/alembic_build/lib/cmake/Alembic" -DCMAKE_BUILD_TYPE=Release -DEigen3_DIR:PATH=${BUILD_DIR}/eigen_build/share/eigen3/cmake -DEigen3_DIR:PATH=${BUILD_DIR}/eigen_build/share/eigen3/cmake -DEIGEN3_INCLUDE_DIR=${BUILD_DIR}/eigen_build/include/eigen3 -DEIGEN_INCLUDE_DIR=${BUILD_DIR}/eigen_build/include/eigen3 -DILMBASE_ROOT=${BUILD_DIR}/openexr_build  -DOPENEXR_ROOT=${BUILD_DIR}/openexr_build -DOPENEXR_INCLUDE_PATH=${BUILD_DIR}/openexr_build/include -DILMBASE_INCLUDE_PATH=${BUILD_DIR}/openexr_build/include -DBOOST_ROOT=${BUILD_DIR}/boost_build -DALICEVISION_USE_ALEMBIC=ON -DAlembic_DIR:PATH=${BUILD_DIR}/alembic_build/lib/cmake/Alembic -DMINIGLOG=ON -DALICEVISION_USE_OPENCV=OFF -DALICEVISION_USE_OPENGV=OFF -DALICEVISION_BUILD_DOC=OFF -DALICEVISION_BUILD_EXAMPLES=OFF -DGEOGRAM_INSTALL_PREFIX=${BUILD_DIR}/geogram_build -DGEOGRAM_INCLUDE_DIR=${BUILD_DIR}/geogram_install/include/geogram1 -DOPENIMAGEIO_LIBRARY_DIR_HINTS=${BUILD_DIR}/openimageio_build/lib -DOPENIMAGEIO_INCLUDE_DIR_HINTS=${BUILD_DIR}/openimageio_build/include -DCeres_DIR=${BUILD_DIR}/ceres_build/lib64/cmake/Ceres ${CUDA_CMAKE_FLAGS} -DALICEVISION_BUILD_SHARED=ON -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS boost ceres eigen alembic openimageio geogram
       )

else()
# Standard cmake project without internal support for dependencies

# ==============================================================================
# Add sources
# ==============================================================================
add_subdirectory(src)

endif()
