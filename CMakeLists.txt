cmake_minimum_required(VERSION 3.11)
project(aliceVision LANGUAGES C CXX)

option(ALICEVISION_BUILD_DEPENDENCIES "Build all AliceVision dependencies" OFF)

option(AV_BUILD_CUDA "Enable building an embedded Cuda" OFF)
option(AV_BUILD_ZLIB "Enable building an embedded ZLIB" ON)
option(AV_BUILD_TIFF "Enable building an embedded Tiff" ON)
option(AV_BUILD_JPEG "Enable building an embedded Jpeg" ON)
option(AV_BUILD_PNG "Enable building an embedded Png" ON)
option(AV_BUILD_LIBRAW "Enable building an embedded libraw" ON)
option(AV_BUILD_POPSIFT "Enable building an embedded PopSift" ON)
option(AV_BUILD_CCTAG "Enable building an embedded CCTag" ON)
option(AV_BUILD_OPENGV "Enable building an embedded OpenGV" ON)
option(AV_BUILD_OPENCV "Enable building an embedded OpenCV" ON)
option(AV_BUILD_LAPACK "Enable building an embedded Lapack" ON)
option(AV_BUILD_SUITESPARSE "Enable building an embedded SuiteSparse" ON)
option(AV_BUILD_ALICEVISION "Enable building of AliceVision" ON)

option(AV_USE_CUDA "Enable CUDA" ON)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

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
message(STATUS "AV_BUILD_CCTAG: ${AV_BUILD_CCTAG}")
message(STATUS "AV_BUILD_POPSIFT: ${AV_BUILD_POPSIFT}")
message(STATUS "AV_BUILD_OPENGV: ${AV_BUILD_OPENGV}")
message(STATUS "AV_BUILD_OPENCV: ${AV_BUILD_OPENCV}")
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

set(CMAKE_CORE_BUILD_FLAGS -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS} -DCMAKE_INSTALL_DO_STRIP:BOOL=${CMAKE_INSTALL_DO_STRIP} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})

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
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       )
set(ZLIB_CMAKE_FLAGS -DZLIB_ROOT=${CMAKE_INSTALL_PREFIX})
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
       URL https://github.com/alicevision/geogram/archive/v1.7.4.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/geogram
       BINARY_DIR ${BUILD_DIR}/geogram_internal_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DVORPALINE_PLATFORM=${VORPALINE_PLATFORM} -DGEOGRAM_WITH_HLBFGS=OFF -DGEOGRAM_WITH_TETGEN=OFF -DGEOGRAM_WITH_GRAPHICS=OFF -DGEOGRAM_WITH_EXPLORAGRAM=OFF -DGEOGRAM_WITH_LUA=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${ZLIB_TARGET}
       )
set(GEOGRAM_CMAKE_FLAGS -DGEOGRAM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DGEOGRAM_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/geogram1)

# Add Cuda
if(AV_USE_CUDA AND AV_BUILD_CUDA)
set(CUDA_TARGET cuda)
ExternalProject_Add(${CUDA_TARGET}
       # URL https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run
       URL https://developer.nvidia.com/compute/cuda/9.2/Prod/local_installers/cuda_9.2.88_396.26_linux
       DOWNLOAD_NO_EXTRACT 1
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cuda
       BINARY_DIR ${BUILD_DIR}/cuda_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ""
       BUILD_COMMAND ""
       INSTALL_COMMAND sh ${BUILD_DIR}/src/cuda_9.2.88_396.26_linux --silent --no-opengl-libs --toolkit --toolkitpath=<INSTALL_DIR>
       )
set(CUDA_CUDART_LIBRARY "")
set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${CMAKE_INSTALL_PREFIX})
else()
  option(CUDA_TOOLKIT_ROOT_DIR "")
  if(CUDA_TOOLKIT_ROOT_DIR)
    set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${CUDA_TOOLKIT_ROOT_DIR})
  endif()
endif()

# Add Tbb
set(TBB_TARGET tbb)
ExternalProject_Add(${TBB_TARGET}
       URL https://github.com/01org/tbb/archive/2019_U6.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbb
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbb
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ""
       BUILD_COMMAND PREFIX=<INSTALL_DIR> make PREFIX=<INSTALL_DIR>
       INSTALL_COMMAND mkdir -p <INSTALL_DIR>/${CMAKE_INSTALL_LIBDIR} && echo "cp <BINARY_DIR>/build/linux_*_release/*.so* <INSTALL_DIR>/${CMAKE_INSTALL_LIBDIR}" > tbb_so_files.sh && sh tbb_so_files.sh && cp -r "<BINARY_DIR>/include" "<INSTALL_DIR>"
       )
set(TBB_CMAKE_FLAGS -DTBB_INCLUDE_DIRS:PATH=${CMAKE_INSTALL_PREFIX}/include -DTBB_LIBRARIES=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libtbb.so)

# Add Eigen
set(EIGEN_TARGET eigen)
set(EIGEN_DISABLE_ALIGN_FLAGS
    "-DCMAKE_CXX_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0"
    "-DCMAKE_C_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0")
ExternalProject_Add(${EIGEN_TARGET}
       URL https://github.com/eigenteam/eigen-git-mirror/archive/3.3.7.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/eigen
       BINARY_DIR ${BUILD_DIR}/eigen_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${EIGEN_DISABLE_ALIGN_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       )
set(EIGEN_CMAKE_FLAGS ${EIGEN_DISABLE_ALIGN_FLAGS} -DEigen3_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake -DEIGEN3_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/eigen3 -DEIGEN_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/eigen3 -DEigen_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/eigen3)

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
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${EIGEN_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${EIGEN_TARGET}
       )
set(OPENGV_CMAKE_FLAGS -DOPENGV_DIR=${CMAKE_INSTALL_PREFIX})
endif()

if(AV_BUILD_LAPACK)
set(LAPACK_TARGET lapack)
ExternalProject_Add(${LAPACK_TARGET}
       URL https://github.com/Reference-LAPACK/lapack/archive/v3.9.0.tar.gz
       #   http://www.netlib.org/lapack/lapack-3.9.0.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/lapack
       BINARY_DIR ${BUILD_DIR}/lapack_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${TBB_TARGET}
       )
set(BLAS_LIBRARIES ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libblas.so)
set(LAPACK_LIBRARIES ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/liblapack.so)
set(LAPACK_CMAKE_FLAGS -DBLAS_LIBRARIES=${BLAS_LIBRARIES} -DLAPACK_LIBRARIES=${LAPACK_LIBRARIES})
endif()

if(AV_BUILD_SUITESPARSE)
set(SUITESPARSE_TARGET suitesparse)
set(SUITESPARSE_INTERNAL_MAKE_CMD LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR} make BLAS="${BLAS_LIBRARIES}" LAPACK="${LAPACK_LIBRARIES}")
ExternalProject_Add(${SUITESPARSE_TARGET}
       # URL https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/v4.5.6.tar.gz
       URL https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/v5.7.2.tar.gz  # requires gxx >= 4.9, centos 7 use gxx-4.8.5 by default
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ""
       BUILD_COMMAND   cd <BINARY_DIR> && ${SUITESPARSE_INTERNAL_MAKE_CMD} library CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER}
       INSTALL_COMMAND cd <BINARY_DIR> && ${SUITESPARSE_INTERNAL_MAKE_CMD} install library INSTALL=<INSTALL_DIR> CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER}
       DEPENDS ${LAPACK_TARGET}
       )
set(SUITESPARSE_CMAKE_FLAGS ${LAPACK_CMAKE_FLAGS} -DSUITESPARSE_INCLUDE_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/include -DSUITESPARSE_LIBRARY_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Add ceres-solver: A Nonlinear Least Squares Minimizer
set(CERES_TARGET ceres)
ExternalProject_Add(${CERES_TARGET}
       # URL https://github.com/ceres-solver/ceres-solver/archive/1.14.0.tar.gz
       GIT_REPOSITORY https://github.com/alicevision/ceres-solver
       GIT_TAG compatibility_gcc_4  # specific commit from the WIP 2.0 version with a fix for gcc-4
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver
       BINARY_DIR ${BUILD_DIR}/ceres_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${SUITESPARSE_CMAKE_FLAGS} -DSUITESPARSE:BOOL=ON -DLAPACK:BOOL=ON ${EIGEN_CMAKE_FLAGS} -DMINIGLOG=ON -DBUILD_EXAMPLES:BOOL=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${EIGEN_TARGET} ${SUITESPARSE_TARGET}
       )
set(CERES_CMAKE_FLAGS ${SUITESPARSE_CMAKE_FLAGS} -DCeres_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/Ceres)

# Add OpenEXR
set(OPENEXR_TARGET openexr)
ExternalProject_Add(${OPENEXR_TARGET}
      URL https://github.com/openexr/openexr/archive/v2.4.0.tar.gz
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
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DOPENEXR_BUILD_PYTHON_LIBS=OFF -DOPENEXR_ENABLE_TESTS=OFF ${ZLIB_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> <SOURCE_DIR>
      DEPENDS ${ZLIB_TARGET}
      )
set(ILMBASE_CMAKE_FLAGS -DILMBASE_ROOT=${CMAKE_INSTALL_PREFIX} -DILMBASE_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include/OpenEXR)
set(OPENEXR_CMAKE_FLAGS ${ILMBASE_CMAKE_FLAGS} -DOPENEXR_ROOT=${CMAKE_INSTALL_PREFIX} -DOPENEXR_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include)

# Add LibTiff
if(AV_BUILD_TIFF)
set(TIFF_TARGET tiff)
ExternalProject_Add(${TIFF_TARGET}
       URL http://download.osgeo.org/libtiff/tiff-4.1.0.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tiff
       BINARY_DIR ${BUILD_DIR}/tiff_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> 
       DEPENDS ${ZLIB_TARGET}
       )
SET(TIFF_CMAKE_FLAGS -DTIFF_LIBRARY=${CMAKE_INSTALL_PREFIX}/lib/libtiff.so -DTIFF_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include)
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
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       # CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> 
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${ZLIB_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${ZLIB_TARGET}
       )
SET(PNG_CMAKE_FLAGS -DPNG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libpng.so -DPNG_PNG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include)
endif()

if(AV_BUILD_JPEG)
set(JPEG_TARGET turbojpeg)
# Add turbojpeg
ExternalProject_Add(${JPEG_TARGET}
       URL https://github.com/libjpeg-turbo/libjpeg-turbo/archive/2.0.4.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       # CONFIGURE_COMMAND cd <BINARY_DIR> && autoreconf -fiv && ./configure --prefix=<INSTALL_DIR> 
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${ZLIB_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${ZLIB_TARGET}
       )
   SET(JPEG_CMAKE_FLAGS -DJPEG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libjpeg.so -DJPEG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include)
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
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      )
ExternalProject_Add(${LIBRAW_TARGET}
      # URL https://github.com/LibRaw/LibRaw/archive/0.19.5.tar.gz
      GIT_REPOSITORY https://github.com/LibRaw/LibRaw  # Need to use git to be able to apply the patch
      GIT_TAG 0.19.5
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      PATCH_COMMAND git config user.name docker && git config user.email "alicevision-team@googlegroups.com" && git cherry-pick 87073c236672741f89b4ddbd7898548141d4c45f  # This patch is needed to get the correct orientation on Canon RAW files
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/libraw
      BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/libraw
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      # Native libraw configure script doesn't work on centos 7 (autoconf 2.69)
      # CONFIGURE_COMMAND autoconf && ./configure --enable-jpeg --enable-openmp --disable-examples --prefix=<INSTALL_DIR>
      # Use cmake build system (not maintained by libraw devs)
    CONFIGURE_COMMAND cp <SOURCE_DIR>_cmake/CMakeLists.txt . && cp -rf <SOURCE_DIR>_cmake/cmake . && ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DENABLE_OPENMP=ON -DENABLE_LCMS=ON -DENABLE_EXAMPLES=OFF ${ZLIB_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DINSTALL_CMAKE_MODULE_PATH:PATH=<INSTALL_DIR>/cmake <SOURCE_DIR>
      DEPENDS libraw_cmake ${ZLIB_TARGET}
      )
SET(LIBRAW_CMAKE_FLAGS -DLIBRAW_PATH=${CMAKE_INSTALL_PREFIX} -DPC_LIBRAW_INCLUDEDIR=${CMAKE_INSTALL_PREFIX}/include -DPC_LIBRAW_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib -DPC_LIBRAW_R_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Add Boost
set(BOOST_TARGET boost)
ExternalProject_Add(${BOOST_TARGET}
       URL https://github.com/alicevision/AliceVisionDependencies/releases/download/boost-src-1.70.0/boost_1_70_0.tar.bz2
       #    http://sourceforge.net/projects/boost/files/boost/1.70.0/boost_1_70_0.tar.bz2
       #    https://dl.bintray.com/boostorg/release/1.70.0/source/boost_1_70_0.tar.gz
       # GIT_REPOSITORY https://github.com/boostorg/boost.git
       # GIT_TAG boost-1.70.0
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/boost
       BINARY_DIR ${BUILD_DIR}/boost_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND cd <SOURCE_DIR> && ./bootstrap.${SCRIPT_EXTENSION} --prefix=<INSTALL_DIR> --with-libraries=atomic,container,date_time,exception,filesystem,graph,log,math,program_options,regex,serialization,system,test,thread,stacktrace,timer
       BUILD_COMMAND cd <SOURCE_DIR> && ./b2 --prefix=<INSTALL_DIR> variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE} link=shared threading=multi -j8
       INSTALL_COMMAND cd <SOURCE_DIR> && ./b2 variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE} link=shared threading=multi install
       DEPENDS ${ZLIB_TARGET}
       )
set(BOOST_CMAKE_FLAGS -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX})

# Add OpenImageIO
set(OPENIMAGEIO_TARGET openimageio)
ExternalProject_Add(${OPENIMAGEIO_TARGET}
      URL https://github.com/OpenImageIO/oiio/archive/Release-2.1.15.0.tar.gz
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openimageio
      BINARY_DIR ${BUILD_DIR}/openimageio_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX} -DOIIO_BUILD_TESTS:BOOL=OFF -DILMBASE_HOME=${CMAKE_INSTALL_PREFIX} -DOPENEXR_HOME=${CMAKE_INSTALL_PREFIX} ${TIFF_CMAKE_FLAGS} ${ZLIB_CMAKE_FLAGS} ${PNG_CMAKE_FLAGS} ${JPEG_CMAKE_FLAGS} ${LIBRAW_CMAKE_FLAGS} ${OPENEXR_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR> -DUSE_PYTHON=OFF -DUSE_OPENCV=OFF -DUSE_OPENGL=OFF
      DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${TIFF_TARGET} ${PNG_TARGET} ${JPEG_TARGET} ${LIBRAW_TARGET} ${ZLIB_TARGET}
      )
# TODO: openjpeg
# -DOPENJPEG_INCLUDE_DIR=$OPENJPEG_INCLUDE_DIR/openjpeg-2.0 -DOPENJPEG_OPENJP2_LIBRARIES=$OPENJPEG_OPENJP2_LIBRARIES
set(OPENIMAGEIO_CMAKE_FLAGS -DOpenImageIO_DIR=${CMAKE_INSTALL_PREFIX})

# Add Alembic: I/O for Point Cloud and Cameras
set(ALEMBIC_TARGET alembic)
ExternalProject_Add(${ALEMBIC_TARGET}
      URL https://github.com/alembic/alembic/archive/1.7.12.tar.gz
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/alembic
      BINARY_DIR ${BUILD_DIR}/alembic_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${ZLIB_CMAKE_FLAGS} ${ILMBASE_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
      DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${ZLIB_TARGET}
      )
set(ALEMBIC_CMAKE_FLAGS -DAlembic_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/Alembic)

if(AV_BUILD_OPENCV)
set(OPENCV_TARGET opencv)
ExternalProject_Add(opencv_contrib
  # URL https://github.com/opencv/opencv_contrib/archive/4.2.0.zip
  # URL_MD5 4776354662667c85a91bcd19f6a13da7
  URL https://github.com/opencv/opencv_contrib/archive/4.1.0.zip
  URL_MD5 3cd00bbfdebb69ad24756ccfb801ebac
  SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv_contrib
  BUILD_ALWAYS 0
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)
ExternalProject_Add(${OPENCV_TARGET}
  DEPENDS opencv_contrib ${TBB_TARGET} ${ZLIB_TARGET} ${OPENEXR_TARGET} ${TIFF_TARGET} ${PNG_TARGET} ${JPEG_TARGET} ${LIBRAW_TARGET}
  # URL https://github.com/opencv/opencv/archive/4.2.0.zip
  # URL_MD5 b02b54115f1f99cb9e885d1e5988ff70
  URL https://github.com/opencv/opencv/archive/4.1.0.zip
  URL_MD5 5c5a9ce3519415b263d512e7f6a1e2af
  UPDATE_COMMAND ""
  BUILD_IN_SOURCE 0
  BUILD_ALWAYS 0
  SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv
  BINARY_DIR ${BUILD_DIR}/opencv_build
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
    -DOPENCV_EXTRA_MODULES_PATH=${CMAKE_CURRENT_BINARY_DIR}/opencv_contrib/modules
    ${ZLIB_CMAKE_FLAGS} ${TBB_CMAKE_FLAGS}
    ${TIFF_CMAKE_FLAGS} ${PNG_CMAKE_FLAGS} ${JPEG_CMAKE_FLAGS} ${LIBRAW_CMAKE_FLAGS}
    -DWITH_TBB=ON
    -DBUILD_opencv_python2=OFF
    -DBUILD_opencv_python3=OFF
    -DWITH_GTK_2_X=OFF
    -DWITH_V4L=OFF
    -DINSTALL_C_EXAMPLES=OFF
    -DINSTALL_PYTHON_EXAMPLES=OFF
    -DBUILD_EXAMPLES=OFF
    -DWITH_QT=OFF
    -DWITH_OPENGL=OFF
    -DWITH_VTK=OFF
    -DWITH_OPENEXR=OFF  # Build error on IlmBase includes without "OpenEXR/" prefix
    -DENABLE_PRECOMPILED_HEADERS=OFF
    -DBUILD_SHARED_LIBS=ON
    -DWITH_CUDA=OFF
    -DWITH_OPENCL=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_LIST=core,improc,photo,objdetect,video,imgcodecs,videoio,features2d,xfeatures2d,version
    ../${OPENCV_SRC_PATH}
)
# set(OPENCV_CMAKE_FLAGS -DOpenCV_DIR=${BUILD_DIR}/opencv_install -DCMAKE_PREFIX_PATH=${BUILD_DIR}/opencv_install)
set(OPENCV_CMAKE_FLAGS -DOpenCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4 -DOPENCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4)
endif()

# Add CCTag
if(AV_BUILD_CCTAG)
set(CCTAG_TARGET cctag)
ExternalProject_Add(${CCTAG_TARGET}
      GIT_REPOSITORY https://github.com/alicevision/CCTag
      GIT_TAG c8bebbbd15ba450bd7655f87aba34ab0c2d32f57
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cctag
      BINARY_DIR ${BUILD_DIR}/cctag_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${BOOST_CMAKE_FLAGS} ${CUDA_CMAKE_FLAGS} ${OPENCV_CMAKE_FLAGS} ${EIGEN_CMAKE_FLAGS} ${TBB_CMAKE_FLAGS} -DCCTAG_WITH_CUDA:BOOL=ON -DBUILD_TESTS=OFF -DBUILD_APPS=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
      DEPENDS ${BOOST_TARGET} ${CUDA_TARGET} ${OPENCV_TARGET} ${EIGEN_TARGET} ${TBB_TARGET}
      )
set(CCTAG_CMAKE_FLAGS -DCCTag_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/CCTag)
endif()

# Add PopSift
if(AV_BUILD_POPSIFT)
set(POPSIFT_TARGET popsift)
ExternalProject_Add(${POPSIFT_TARGET}
      GIT_REPOSITORY https://github.com/alicevision/popsift
      GIT_TAG 7fc809b73ff0313320013465e2f8b1af4f9d63a8
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/popsift
      BINARY_DIR ${BUILD_DIR}/popsift_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} ${BOOST_CMAKE_FLAGS} ${CUDA_CMAKE_FLAGS} -DPopSift_BUILD_EXAMPLES:BOOL=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
      DEPENDS ${BOOST_TARGET} ${CUDA_TARGET}
      )
set(POPSIFT_CMAKE_FLAGS -DPopSift_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/PopSift)
endif()

set(AV_DEPS
  ${ZLIB_TARGET}
  ${GEOGRAM_TARGET}
  ${CUDA_TARGET}
  ${TBB_TARGET}
  ${EIGEN_TARGET}
  ${OPENGV_TARGET}
  ${OPENCV_TARGET}
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
  ${CCTAG_TARGET}
  ${POPSIFT_TARGET}
)

if(AV_BUILD_ALICEVISION)
ExternalProject_Add(aliceVision
       PREFIX ${CMAKE_CURRENT_SOURCE_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 1
       SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src
       BINARY_DIR ${BUILD_DIR}/aliceVision_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DBUILD_SHARED_LIBS:BOOL=ON -DTARGET_ARCHITECTURE=core -DALICEVISION_ROOT=${ALICEVISION_ROOT} -DALICEVISION_USE_ALEMBIC=ON -DMINIGLOG=ON -DALICEVISION_USE_CCTAG=${AV_BUILD_CCTAG} -DALICEVISION_USE_OPENCV=${AV_BUILD_OPENCV} -DALICEVISION_USE_OPENGV=${AV_BUILD_OPENGV} -DALICEVISION_USE_POPSIFT=${AV_BUILD_POPSIFT} -DALICEVISION_USE_CUDA=${AV_USE_CUDA} -DALICEVISION_BUILD_DOC=OFF -DALICEVISION_BUILD_EXAMPLES=OFF ${ZLIB_CMAKE_FLAGS} ${EIGEN_CMAKE_FLAGS} ${OPENIMAGEIO_CMAKE_FLAGS} ${OPENEXR_CMAKE_FLAGS} ${BOOST_CMAKE_FLAGS} ${ALEMBIC_CMAKE_FLAGS} ${GEOGRAM_CMAKE_FLAGS} ${LAPACK_CMAKE_FLAGS} ${CERES_CMAKE_FLAGS} ${CUDA_CMAKE_FLAGS} ${POPSIFT_CMAKE_FLAGS} ${OPENGV_CMAKE_FLAGS} ${OPENCV_CMAKE_FLAGS} ${CCTAG_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${AV_DEPS}
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


