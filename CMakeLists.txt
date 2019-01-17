cmake_minimum_required(VERSION 3.3)
project(aliceVision LANGUAGES C CXX)


option(ALICEVISION_BUILD_DEPENDENCIES "Build all AliceVision dependencies" OFF)

option(AV_BUILD_CUDA "Enable building an embedded Cuda" OFF)
option(AV_BUILD_ZLIB "Enable building an embedded ZLIB" ON)
option(AV_BUILD_TIFF "Enable building an embedded Tiff" ON)
option(AV_BUILD_JPEG "Enable building an embedded Jpeg" ON)
option(AV_BUILD_PNG "Enable building an embedded Png" ON)
option(AV_BUILD_LIBRAW "Enable building an embedded libraw" ON)
option(AV_BUILD_POPSIFT "Enable building an embedded PopSift" ON)
option(AV_BUILD_OPENGV "Enable building an embedded OpenGV" ON)
option(AV_BUILD_LAPACK "Enable building an embedded Lapack" ON)
option(AV_BUILD_SUITESPARSE "Enable building an embedded SuiteSparse" ON)

option(AV_USE_CUDA "Enable CUDA" ON)

option(INSTALL_DEPS_BUILD "Install all files from dependencies (only used if ALICEVISION_BUILD_DEPENDENCIES=ON)" OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type for AliceVision" FORCE)
endif()

# set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type for AliceVision")
set(DEPS_CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type for all external libraries (only used if ALICEVISION_BUILD_DEPENDENCIES=ON)")
string(TOLOWER ${DEPS_CMAKE_BUILD_TYPE} DEPS_CMAKE_BUILD_TYPE_LOWERCASE)

option(ALICEVISION_BUILD_TESTS "Build AliceVision tests" OFF)

set(ALICEVISION_BUNDLE_PREFIX "${CMAKE_INSTALL_PREFIX}/bundle" CACHE STRING "Path for bundle installation")

set(ALICEVISION_ROOT ${PROJECT_BINARY_DIR})

# ==============================================================================
# GNUInstallDirs CMake module
# - Define GNU standard installation directories
# - Provides install directory variables as defined by the GNU Coding Standards.
# ==============================================================================
include(GNUInstallDirs)

if(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "Force CMAKE_INSTALL_DO_STRIP in Release")
    set(CMAKE_INSTALL_DO_STRIP TRUE)
endif()

message(STATUS "----------------------------------------")
message(STATUS "ALICEVISION_BUILD_DEPENDENCIES: ${ALICEVISION_BUILD_DEPENDENCIES}")
message(STATUS "ALICEVISION_BUILD_TESTS: ${ALICEVISION_BUILD_TESTS}")
message(STATUS "AV_BUILD_CUDA: ${AV_BUILD_CUDA}")
message(STATUS "AV_BUILD_ZLIB: ${AV_BUILD_ZLIB}")
message(STATUS "AV_BUILD_TIFF: ${AV_BUILD_TIFF}")
message(STATUS "AV_BUILD_JPEG: ${AV_BUILD_JPEG}")
message(STATUS "AV_BUILD_PNG: ${AV_BUILD_PNG}")
message(STATUS "AV_BUILD_LIBRAW: ${AV_BUILD_LIBRAW}")
message(STATUS "AV_BUILD_POPSIFT: ${AV_BUILD_POPSIFT}")
message(STATUS "AV_BUILD_OPENGV: ${AV_BUILD_OPENGV}")
message(STATUS "AV_BUILD_LAPACK: ${AV_BUILD_LAPACK}")
message(STATUS "AV_BUILD_SUITESPARSE: ${AV_BUILD_SUITESPARSE}")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "DEPS_CMAKE_BUILD_TYPE: ${DEPS_CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_INSTALL_RPATH: ${CMAKE_INSTALL_RPATH}")
message(STATUS "CMAKE_INSTALL_DO_STRIP: ${CMAKE_INSTALL_DO_STRIP}")
message(STATUS "----------------------------------------")

# ==============================================================================
# Enable cmake UNIT TEST framework
# ==============================================================================
if(ALICEVISION_BUILD_TESTS)
  enable_testing()
endif()

if(WIN32)
  set(SCRIPT_EXTENSION bat)
else()
  set(SCRIPT_EXTENSION sh)
endif()

if(ALICEVISION_BUILD_DEPENDENCIES)

# ==============================================================================
# Use CMake ExternalProject to build all dependencies
# ==============================================================================
include(ExternalProject)

set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/external")

set(CMAKE_CORE_BUILD_FLAGS -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DBUILD_SHARED_LIBS:BOOL=ON -DCMAKE_INSTALL_DO_STRIP=${CMAKE_INSTALL_DO_STRIP})
set(CMAKE_CORE_BUILD_FLAGS_NOSHARED -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_DO_STRIP=${CMAKE_INSTALL_DO_STRIP})

if(AV_BUILD_ZLIB)
set(ZLIB_TARGET zlib)
ExternalProject_Add(${ZLIB_TARGET}
       URL http://www.zlib.net/zlib-1.2.11.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/zlib
       BINARY_DIR ${BUILD_DIR}/zlib_build
       INSTALL_DIR ${BUILD_DIR}/zlib_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       )
set(ZLIB_CMAKE_FLAGS -DZLIB_ROOT=${BUILD_DIR}/zlib_build)
endif()


# Add Geogram
if(WIN32)
   set(VORPALINE_PLATFORM Win-vs-dynamic-generic)
elseif(APPLE)
   set(VORPALINE_PLATFORM Darwin-clang-dynamic)
elseif(UNIX)
   set(VORPALINE_PLATFORM Linux64-gcc-dynamic)
endif()
set(GEOGRAM_TARGET geogram)
ExternalProject_Add(${GEOGRAM_TARGET}
       URL https://github.com/alicevision/geogram/archive/v1.6.6.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/geogram
       BINARY_DIR ${BUILD_DIR}/geogram_internal_build
       INSTALL_DIR ${BUILD_DIR}/geogram_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DVORPALINE_PLATFORM=${VORPALINE_PLATFORM} -DGEOGRAM_WITH_HLBFGS=OFF -DGEOGRAM_WITH_TETGEN=OFF -DGEOGRAM_WITH_GRAPHICS=OFF -DGEOGRAM_WITH_EXPLORAGRAM=OFF -DGEOGRAM_WITH_LUA=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${ZLIB_TARGET}
       )
set(GEOGRAM_CMAKE_FLAGS -DGEOGRAM_INSTALL_PREFIX=${BUILD_DIR}/geogram_build -DGEOGRAM_INCLUDE_DIR=${BUILD_DIR}/geogram_build/include/geogram1)

# Add Cuda
if(AV_USE_CUDA AND AV_BUILD_CUDA)
set(CUDA_TARGET cuda)
ExternalProject_Add(${CUDA_TARGET}
       URL https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run
       # URL https://developer.nvidia.com/compute/cuda/9.2/Prod/local_installers/cuda_9.2.88_396.26_linux
       DOWNLOAD_NO_EXTRACT 1
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cuda
       BINARY_DIR ${BUILD_DIR}/cuda_build
       INSTALL_DIR ${BUILD_DIR}/cuda_build
       CONFIGURE_COMMAND ""
       BUILD_COMMAND ""
       INSTALL_COMMAND sh ${BUILD_DIR}/src/cuda_8.0.61_375.26_linux-run --silent --no-opengl-libs --toolkit --toolkitpath=<INSTALL_DIR>
       )
set(CUDA_CUDART_LIBRARY "")
set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${BUILD_DIR}/cuda_build)
else()
  option(CUDA_TOOLKIT_ROOT_DIR "")
  if(CUDA_TOOLKIT_ROOT_DIR)
    set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${CUDA_TOOLKIT_ROOT_DIR})
  endif()
endif()

# Add Tbb
set(TBB_TARGET tbb)
ExternalProject_Add(${TBB_TARGET}
       URL https://github.com/01org/tbb/archive/2018_U5.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbb
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbb
       INSTALL_DIR ${BUILD_DIR}/tbb_build
       CONFIGURE_COMMAND ""
       BUILD_COMMAND PREFIX=<INSTALL_DIR> make PREFIX=<INSTALL_DIR>
       INSTALL_COMMAND mkdir -p <INSTALL_DIR>/${CMAKE_INSTALL_LIBDIR} && echo "cp <BINARY_DIR>/build/linux_*_release/*.so <INSTALL_DIR>/${CMAKE_INSTALL_LIBDIR}" > tbb_so_files.sh && sh tbb_so_files.sh && cp -r "<BINARY_DIR>/include" "<INSTALL_DIR>"
       )

# Add Eigen
set(EIGEN_TARGET eigen)
ExternalProject_Add(${EIGEN_TARGET}
       URL https://github.com/eigenteam/eigen-git-mirror/archive/3.3.7.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/eigen
       BINARY_DIR ${BUILD_DIR}/eigen_build
       INSTALL_DIR ${BUILD_DIR}/eigen_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       )
set(EIGEN_CMAKE_FLAGS -DEigen3_DIR:PATH=${BUILD_DIR}/eigen_build/share/eigen3/cmake -DEIGEN3_INCLUDE_DIR=${BUILD_DIR}/eigen_build/include/eigen3 -DEIGEN_INCLUDE_DIR=${BUILD_DIR}/eigen_build/include/eigen3)

if(AV_BUILD_OPENGV)
set(OPENGV_TARGET opengv)
ExternalProject_Add(${OPENGV_TARGET}
       GIT_REPOSITORY https://github.com/laurentkneip/opengv.git
       GIT_TAG 306a54e6c6b94e2048f820cdf77ef5281d4b48ad
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opengv
       BINARY_DIR ${BUILD_DIR}/opengv_build
       INSTALL_DIR ${BUILD_DIR}/opengv_install
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${EIGEN_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${EIGEN_TARGET}
       )
set(OPENGV_CMAKE_FLAGS -DOPENGV_DIR=${BUILD_DIR}/opengv_install)
endif()


if(AV_BUILD_LAPACK)
set(LAPACK_TARGET lapack)
ExternalProject_Add(${LAPACK_TARGET}
       URL http://www.netlib.org/lapack/lapack-3.8.0.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/lapack
       BINARY_DIR ${BUILD_DIR}/lapack_build
       INSTALL_DIR ${BUILD_DIR}/lapack_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${TBB_TARGET}
       )
set(BLAS_LIBRARIES ${BUILD_DIR}/lapack_build/${CMAKE_INSTALL_LIBDIR}/libblas.so)
set(LAPACK_LIBRARIES ${BUILD_DIR}/lapack_build/${CMAKE_INSTALL_LIBDIR}/liblapack.so)
set(LAPACK_CMAKE_FLAGS -DBLAS_LIBRARIES=${BLAS_LIBRARIES} -DLAPACK_LIBRARIES=${LAPACK_LIBRARIES})
endif()

if(AV_BUILD_SUITESPARSE)
set(SUITESPARSE_TARGET suitesparse)
set(SUITESPARSE_INTERNAL_MAKE_CMD LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${BUILD_DIR}/lapack_build/${CMAKE_INSTALL_LIBDIR} make BLAS="${BLAS_LIBRARIES}" LAPACK="${LAPACK_LIBRARIES}")
ExternalProject_Add(${SUITESPARSE_TARGET}
       URL https://github.com/jluttine/suitesparse/archive/v4.5.6.tar.gz
       # URL http://faculty.cse.tamu.edu/davis/SuiteSparse/SuiteSparse-5.4.0.tar.gz  # requires gxx >= 4.9, centos 7 use gxx-4.8.5
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
       INSTALL_DIR ${BUILD_DIR}/suitesparse_build
       CONFIGURE_COMMAND ""
       BUILD_COMMAND   cd <BINARY_DIR> && ${SUITESPARSE_INTERNAL_MAKE_CMD}
       INSTALL_COMMAND cd <BINARY_DIR> && ${SUITESPARSE_INTERNAL_MAKE_CMD} install INSTALL=<INSTALL_DIR>
       DEPENDS ${LAPACK_TARGET}
       )
set(SUITESPARSE_CMAKE_FLAGS ${LAPACK_CMAKE_FLAGS} -DSUITESPARSE_INCLUDE_DIR_HINTS=${BUILD_DIR}/suitesparse_build/include -DSUITESPARSE_LIBRARY_DIR_HINTS=${BUILD_DIR}/suitesparse_build/lib)
endif()

# Add ceres-solver: A Nonlinear Least Squares Minimizer
set(CERES_TARGET ceres)
ExternalProject_Add(${CERES_TARGET}
       URL https://github.com/ceres-solver/ceres-solver/archive/1.14.0.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver
       BINARY_DIR ${BUILD_DIR}/ceres_build
       INSTALL_DIR ${BUILD_DIR}/ceres_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${SUITESPARSE_CMAKE_FLAGS} -DSUITESPARSE:BOOL=ON -DLAPACK:BOOL=ON ${EIGEN_CMAKE_FLAGS} -DMINIGLOG=ON -DBUILD_EXAMPLES:BOOL=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${EIGEN_TARGET} ${SUITESPARSE_TARGET}
       )
set(CERES_CMAKE_FLAGS ${SUITESPARSE_CMAKE_FLAGS} -DCeres_DIR=${BUILD_DIR}/ceres_build/${CMAKE_INSTALL_LIBDIR}/cmake/Ceres)

# Add OpenEXR
set(OPENEXR_TARGET openexr)
ExternalProject_Add(${OPENEXR_TARGET}
      URL https://github.com/openexr/openexr/archive/v2.3.0.tar.gz
      # URL https://github.com/openexr/openexr/archive/v2.2.1.tar.gz
      # The release 2.2.1 has troubles with C++17, which breaks compilation with recent compilers.
      # The problem has been fixed https://github.com/openexr/openexr/issues/235
      # but there is no release yet, so we use the development version.
      # GIT_REPOSITORY https://github.com/openexr/openexr
      # Use the latest commit with g++4.X compatibility
      # GIT_TAG 74b5c1dc2dfbdce74987a57f5e011dc711f9ca65
      # Finally use a custom version for a cmake fix
      # GIT_REPOSITORY https://github.com/alicevision/openexr
      # GIT_TAG develop_compatibility_gxx4
      # GIT_REPOSITORY https://github.com/openexr/openexr
      # GIT_TAG a12937f6d7650d4fb81b469900ee2fd4c082c208
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openexr
      BINARY_DIR ${BUILD_DIR}/openexr_build
      INSTALL_DIR ${BUILD_DIR}/openexr_build
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DOPENEXR_BUILD_PYTHON_LIBS=OFF -DOPENEXR_ENABLE_TESTS=OFF ${ZLIB_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> <SOURCE_DIR>
      DEPENDS ${ZLIB_TARGET}
      )
set(ILMBASE_CMAKE_FLAGS -DILMBASE_ROOT=${BUILD_DIR}/openexr_build -DILMBASE_INCLUDE_PATH=${BUILD_DIR}/openexr_build/include)
set(OPENEXR_CMAKE_FLAGS ${ILMBASE_CMAKE_FLAGS} -DOPENEXR_ROOT=${BUILD_DIR}/openexr_build -DOPENEXR_INCLUDE_PATH=${BUILD_DIR}/openexr_build/include)

# Add LibTiff
if(AV_BUILD_TIFF)
set(TIFF_TARGET tiff)
ExternalProject_Add(${TIFF_TARGET}
       URL http://download.osgeo.org/libtiff/tiff-4.0.10.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tiff
       BINARY_DIR ${BUILD_DIR}/tiff_build
       INSTALL_DIR ${BUILD_DIR}/tiff_build
       CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> 
       DEPENDS ${ZLIB_TARGET}
       )
SET(TIFF_CMAKE_FLAGS -DTIFF_LIBRARY=${BUILD_DIR}/tiff_build/lib/libtiff.so -DTIFF_INCLUDE_DIR=${BUILD_DIR}/tiff_build/include)
endif()

# Add LibPng
if(AV_BUILD_PNG)
set(PNG_TARGET png)
ExternalProject_Add(${PNG_TARGET}
       URL http://github.com/glennrp/libpng-releases/raw/master/libpng-1.6.34.tar.gz
       #    http://prdownloads.sourceforge.net/libpng/libpng-1.6.34.tar.gz
       #    https://sourceforge.net/projects/libpng/files/libpng16/1.6.34/libpng-1.6.34.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/png
       BINARY_DIR ${BUILD_DIR}/png_build
       INSTALL_DIR ${BUILD_DIR}/png_build
       # CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> 
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${ZLIB_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${ZLIB_TARGET}
       )
SET(PNG_CMAKE_FLAGS -DPNG_LIBRARY=${BUILD_DIR}/png_build/${CMAKE_INSTALL_LIBDIR}/libpng.so -DPNG_PNG_INCLUDE_DIR=${BUILD_DIR}/png_build/include)
endif()

if(AV_BUILD_JPEG)
set(JPEG_TARGET turbojpeg)
# Add turbojpeg
ExternalProject_Add(${JPEG_TARGET}
       URL https://github.com/libjpeg-turbo/libjpeg-turbo/archive/1.5.3.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg
       INSTALL_DIR ${BUILD_DIR}/turbojpeg_build
       CONFIGURE_COMMAND cd <BINARY_DIR> && autoreconf -fiv && ./configure --prefix=<INSTALL_DIR> 
       DEPENDS ${ZLIB_TARGET}
       )
SET(JPEG_CMAKE_FLAGS -DJPEG_LIBRARY=${BUILD_DIR}/turbojpeg_build/lib/libjpeg.so -DJPEG_INCLUDE_DIR=${BUILD_DIR}/turbojpeg_build/include)
endif()


if(AV_BUILD_LIBRAW)
set(LIBRAW_TARGET libraw)
# Add libraw
ExternalProject_Add(libraw_cmake
      GIT_REPOSITORY https://github.com/LibRaw/LibRaw-cmake
      GIT_TAG master
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/libraw_cmake
      BINARY_DIR ${BUILD_DIR}/libraw_build
      INSTALL_DIR ${BUILD_DIR}/libraw_build
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      )
ExternalProject_Add(${LIBRAW_TARGET}
      URL https://github.com/LibRaw/LibRaw/archive/0.19.2.tar.gz
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/libraw
      BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/libraw
      INSTALL_DIR ${BUILD_DIR}/libraw_build
      # Native libraw configure script doesn't work on centos 7 (autoconf 2.69)
      # CONFIGURE_COMMAND autoconf && ./configure --enable-jpeg --enable-openmp --disable-examples --prefix=<INSTALL_DIR>
      # Use cmake build system (not maintained by libraw devs)
    CONFIGURE_COMMAND cp <SOURCE_DIR>_cmake/CMakeLists.txt . && cp -rf <SOURCE_DIR>_cmake/cmake . && ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DENABLE_OPENMP=ON -DENABLE_LCMS=ON -DENABLE_EXAMPLES=OFF ${ZLIB_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DINSTALL_CMAKE_MODULE_PATH:PATH=<INSTALL_DIR>/cmake <SOURCE_DIR>
      DEPENDS libraw_cmake ${ZLIB_TARGET}
      )
SET(LIBRAW_CMAKE_FLAGS -DLIBRAW_PATH=${BUILD_DIR}/libraw_build -DPC_LIBRAW_INCLUDEDIR=${BUILD_DIR}/libraw_build/include -DPC_LIBRAW_LIBDIR=${BUILD_DIR}/libraw_build/lib -DPC_LIBRAW_R_LIBDIR=${BUILD_DIR}/libraw_build/lib)
endif()

# Add Boost
set(BOOST_TARGET boost)
ExternalProject_Add(${BOOST_TARGET}
       URL https://github.com/alicevision/AliceVisionDependencies/releases/download/boost-src-1.66.0/boost_1_66_0.tar.bz2
       #    http://sourceforge.net/projects/boost/files/boost/1.66.0/boost_1_66_0.tar.bz2
       #    https://dl.bintray.com/boostorg/release/1.66.0/source/boost_1_66_0.tar.gz
       # GIT_REPOSITORY https://github.com/boostorg/boost.git
       # GIT_TAG boost-1.61.0
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/boost
       BINARY_DIR ${BUILD_DIR}/boost_build
       INSTALL_DIR ${BUILD_DIR}/boost_build
       CONFIGURE_COMMAND cd <SOURCE_DIR> && ./bootstrap.${SCRIPT_EXTENSION} --prefix=<INSTALL_DIR> --with-libraries=atomic,container,date_time,exception,filesystem,graph,log,math,program_options,regex,serialization,system,test,thread
       BUILD_COMMAND cd <SOURCE_DIR> && ./b2 --prefix=<INSTALL_DIR> variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE} link=shared threading=multi -j8
       INSTALL_COMMAND cd <SOURCE_DIR> && ./b2 variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE} link=shared threading=multi install
       DEPENDS ${ZLIB_TARGET}
       )
set(BOOST_CMAKE_FLAGS -DBOOST_ROOT=${BUILD_DIR}/boost_build)

# Add OpenImageIO
set(OPENIMAGEIO_TARGET openimageio)
ExternalProject_Add(${OPENIMAGEIO_TARGET}
      URL https://github.com/OpenImageIO/oiio/archive/Release-2.0.3.tar.gz
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openimageio
      BINARY_DIR ${BUILD_DIR}/openimageio_build
      INSTALL_DIR ${BUILD_DIR}/openimageio_build
      CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_CXX_FLAGS="-Wno-deprecated-declarations" ${CMAKE_CORE_BUILD_FLAGS} -DCMAKE_PREFIX_PATH="${BUILD_DIR}/png_build;${BUILD_DIR}/turbojpeg_build;${BUILD_DIR}/libraw_build" -DBOOST_ROOT=${BUILD_DIR}/boost_build -DOIIO_BUILD_TESTS:BOOL=OFF -DILMBASE_HOME=${BUILD_DIR}/openexr_build -DOPENEXR_HOME=${BUILD_DIR}/openexr_build ${TIFF_CMAKE_FLAGS} ${ZLIB_CMAKE_FLAGS} ${PNG_CMAKE_FLAGS} ${JPEG_CMAKE_FLAGS} ${LIBRAW_CMAKE_FLAGS} ${OPENEXR_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR> -DUSE_PYTHON=OFF -DUSE_OPENCV=OFF -DUSE_OPENGL=OFF
      DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${TIFF_TARGET} ${PNG_TARGET} ${JPEG_TARGET} ${LIBRAW_TARGET} ${ZLIB_TARGET}
      )
# TODO: openjpeg
# -DOPENJPEG_INCLUDE_DIR=$OPENJPEG_INCLUDE_DIR/openjpeg-2.0 -DOPENJPEG_OPENJP2_LIBRARIES=$OPENJPEG_OPENJP2_LIBRARIES
set(OPENIMAGEIO_CMAKE_FLAGS -DOPENIMAGEIO_LIBRARY_DIR_HINTS=${BUILD_DIR}/openimageio_build -DOPENIMAGEIO_INCLUDE_DIR_HINTS=${BUILD_DIR}/openimageio_build)

# Add Alembic: I/O for Point Cloud and Cameras
set(ALEMBIC_TARGET alembic)
ExternalProject_Add(${ALEMBIC_TARGET}
      URL https://github.com/alembic/alembic/archive/1.7.10.tar.gz
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/alembic
      BINARY_DIR ${BUILD_DIR}/alembic_build
      INSTALL_DIR ${BUILD_DIR}/alembic_build
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${ZLIB_CMAKE_FLAGS} ${ILMBASE_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
      DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${ZLIB_TARGET}
      )
set(ALEMBIC_CMAKE_FLAGS -DAlembic_DIR:PATH=${BUILD_DIR}/alembic_build/lib/cmake/Alembic)

# Add PopSift
if(AV_BUILD_POPSIFT)
set(POPSIFT_TARGET popsift)
ExternalProject_Add(${POPSIFT_TARGET}
      GIT_REPOSITORY https://github.com/alicevision/popsift
      GIT_TAG aa77bd6d308ad9fced2d5ba63e02f2f10873479b
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/popsift
      BINARY_DIR ${BUILD_DIR}/popsift_build
      INSTALL_DIR ${BUILD_DIR}/popsift_build
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${BOOST_CMAKE_FLAGS} ${CUDA_CMAKE_FLAGS} -DPopSift_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
      DEPENDS ${BOOST_TARGET} ${CUDA_TARGET}
      )
set(POPSIFT_CMAKE_FLAGS -DPopSift_DIR:PATH=${BUILD_DIR}/popsift_build/lib/cmake/PopSift)
endif()

set(AV_DEPS
  ${ZLIB_TARGET}
  ${GEOGRAM_TARGET}
  ${CUDA_TARGET}
  ${TBB_TARGET}
  ${EIGEN_TARGET}
  ${OPENGV_TARGET}
  ${LAPACK_TARGET}
  ${SUITESPARSE_TARGET}
  ${CERES_TARGET}
  ${OPENEXR_TARGET}
  ${TIFF_TARGET}
  ${PNG_TARGET}
  ${JPEG_TARGET}
  ${LIBRAW_TARGET}
  ${BOOST_TARGET}
  ${OPENIMAGEIO_TARGET}
  ${ALEMBIC_TARGET}
  ${POPSIFT_TARGET}
)

ExternalProject_Add(aliceVision
       PREFIX ${CMAKE_CURRENT_SOURCE_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 1
       SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src
       BINARY_DIR ${BUILD_DIR}/aliceVision_build
       INSTALL_DIR ${BUILD_DIR}/aliceVision_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DBUILD_SHARED_LIBS:BOOL=ON -DTARGET_ARCHITECTURE=core -DALICEVISION_ROOT=${ALICEVISION_ROOT} -DALICEVISION_USE_ALEMBIC=ON -DMINIGLOG=ON -DALICEVISION_USE_OPENCV=OFF -DALICEVISION_USE_OPENGV=ON -DALICEVISION_USE_POPSIFT=${AV_BUILD_POPSIFT} -DALICEVISION_USE_CUDA=${AV_USE_CUDA} -DALICEVISION_BUILD_DOC=OFF -DALICEVISION_BUILD_EXAMPLES=OFF ${ZLIB_CMAKE_FLAGS} ${EIGEN_CMAKE_FLAGS} ${OPENIMAGEIO_CMAKE_FLAGS} ${OPENEXR_CMAKE_FLAGS} ${BOOST_CMAKE_FLAGS} ${ALEMBIC_CMAKE_FLAGS} ${GEOGRAM_CMAKE_FLAGS} ${LAPACK_CMAKE_FLAGS} ${CERES_CMAKE_FLAGS} ${CUDA_CMAKE_FLAGS} ${POPSIFT_CMAKE_FLAGS} ${OPENGV_CMAKE_FLAGS} -DALICEVISION_BUILD_SHARED=ON -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${AV_DEPS}
       )

if(CMAKE_INSTALL_PREFIX)
  foreach(LIBNAME ${AV_DEPS})
    set(DEP_FOLDER ${BUILD_DIR}/${LIBNAME}_build)
    message(STATUS "Install ${DEP_FOLDER}/lib/ ${DEP_FOLDER}/lib64/ ===> ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
    # Install dependencies libraries
    if(INSTALL_DEPS_BUILD)
      # Install dependencies build files
      install(
        DIRECTORY ${DEP_FOLDER}/include ${DEP_FOLDER}/share
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        USE_SOURCE_PERMISSIONS
        OPTIONAL
        )
      install(
        DIRECTORY ${DEP_FOLDER}/lib/ ${DEP_FOLDER}/lib64/
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        USE_SOURCE_PERMISSIONS
        OPTIONAL
        )
    else()
      install(
        DIRECTORY ${DEP_FOLDER}/lib/ ${DEP_FOLDER}/lib64/
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        USE_SOURCE_PERMISSIONS
        OPTIONAL
        FILES_MATCHING PATTERN "lib*.so*"
        )
    endif()
  endforeach()
  # Install AliceVision's build files
  install(
    DIRECTORY ${BUILD_DIR}/aliceVision_build/bin ${BUILD_DIR}/aliceVision_build/lib ${BUILD_DIR}/aliceVision_build/lib64 ${BUILD_DIR}/aliceVision_build/include ${BUILD_DIR}/aliceVision_build/share
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
    OPTIONAL
    )
endif()

else()

# ==============================================================================
# Standard cmake project without internal support for dependencies
# ==============================================================================

# Add sources
add_subdirectory(src)

endif()

install(
    FILES LICENSE-MPL2.md LICENSE-MIT-libmv.md COPYING.md CONTRIBUTORS.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision
    )

# Bundle target (see src/cmake/MakeBundle.cmake)
# Note: require that the install rule has been executed

# Include VCPKG installed dir for runtime dependencies lookup
set(BUNDLE_LIBS_PATHS "")
if(_VCPKG_INSTALLED_DIR)
  set(BUNDLE_LIBS_PATHS ${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}$<$<CONFIG:Debug>:/debug>/bin)
endif()

add_custom_target(bundle
                  ${CMAKE_COMMAND}
                    # Script arguments:
                    -DBUNDLE_INSTALL_PREFIX=${ALICEVISION_BUNDLE_PREFIX}
                    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                    -DBUNDLE_LIBS_PATHS=${BUNDLE_LIBS_PATHS}
                    # Script:
                    -P ${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/MakeBundle.cmake
                  )
set_target_properties(bundle PROPERTIES FOLDER Packaging)


