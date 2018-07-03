cmake_minimum_required(VERSION 3.3)
project(aliceVision LANGUAGES C CXX)


option(ALICEVISION_BUILD_DEPENDENCIES "Build all AliceVision dependencies" OFF)

option(AV_BUILD_JPEG "Enable building an embedded Jpeg" ON)
option(AV_BUILD_PNG "Enable building an embedded Png" ON)
option(AV_BUILD_CUDA "Enable building an embedded Cuda" OFF)
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

message(STATUS "----------------------------------------")
message(STATUS "ALICEVISION_BUILD_DEPENDENCIES: ${ALICEVISION_BUILD_DEPENDENCIES}")
message(STATUS "ALICEVISION_BUILD_TESTS: ${ALICEVISION_BUILD_TESTS}")
message(STATUS "AV_BUILD_JPEG: ${AV_BUILD_JPEG}")
message(STATUS "AV_BUILD_PNG: ${AV_BUILD_PNG}")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "DEPS_CMAKE_BUILD_TYPE: ${DEPS_CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
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
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/geogram
       BINARY_DIR ${BUILD_DIR}/geogram_build
       INSTALL_DIR ${BUILD_DIR}/geogram_install
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DVORPALINE_PLATFORM=${VORPALINE_PLATFORM} -DGEOGRAM_WITH_HLBFGS=OFF -DGEOGRAM_WITH_TETGEN=OFF -DGEOGRAM_WITH_GRAPHICS=OFF -DGEOGRAM_WITH_EXPLORAGRAM=OFF -DGEOGRAM_WITH_LUA=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       )
set(GEOGRAM_CMAKE_FLAGS -DGEOGRAM_INSTALL_PREFIX=${BUILD_DIR}/geogram_build -DGEOGRAM_INCLUDE_DIR=${BUILD_DIR}/geogram_install/include/geogram1)

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

# Add Eigen
ExternalProject_Add(eigen
       URL https://github.com/alicevision/eigen-git-mirror/archive/3.3.4.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/eigen
       BINARY_DIR ${BUILD_DIR}/eigen_build
       INSTALL_DIR ${BUILD_DIR}/eigen_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       )
set(EIGEN_CMAKE_FLAGS -DEigen3_DIR:PATH=${BUILD_DIR}/eigen_build/share/eigen3/cmake -DEigen3_DIR:PATH=${BUILD_DIR}/eigen_build/share/eigen3/cmake -DEIGEN3_INCLUDE_DIR=${BUILD_DIR}/eigen_build/include/eigen3 -DEIGEN_INCLUDE_DIR=${BUILD_DIR}/eigen_build/include/eigen3)

# Add ceres-solver: A Nonlinear Least Squares Minimizer
ExternalProject_Add(ceres
       URL https://github.com/ceres-solver/ceres-solver/archive/1.13.0.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver
       BINARY_DIR ${BUILD_DIR}/ceres_build
       INSTALL_DIR ${BUILD_DIR}/ceres_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DEigen3_DIR:PATH=${BUILD_DIR}/eigen_build/share/eigen3/cmake -DEIGEN_INCLUDE_DIR=${BUILD_DIR}/eigen_build/include/eigen3 -DMINIGLOG=ON -DBUILD_EXAMPLES:BOOL=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS eigen
       )
# Test 32/64 bits
if (CMAKE_SYSTEM_NAME MATCHES "Linux" AND
    NOT DEFINED LIB_SUFFIX AND
    NOT CMAKE_CROSSCOMPILING AND
    CMAKE_SIZEOF_VOID_P EQUAL "8" AND
    NOT EXISTS "/etc/debian_version" AND
    NOT EXISTS "/etc/arch-release")
    # message(STATUS "Target is 64 bits")
    set(CERES_CMAKE_FLAGS -DCeres_DIR=${BUILD_DIR}/ceres_build/lib64/cmake/Ceres )
else()
    # message(STATUS "Target is 32 bits")
    set(CERES_CMAKE_FLAGS -DCeres_DIR=${BUILD_DIR}/ceres_build/lib/cmake/Ceres )
endif()

# Add IlmBase
ExternalProject_Add(ilmbase
       GIT_REPOSITORY https://github.com/alicevision/openexr
       GIT_TAG develop_compatibility_gxx4
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openexr
       BINARY_DIR ${BUILD_DIR}/ilmbase_build
       INSTALL_DIR ${BUILD_DIR}/openexr_build # Note: Need to install in the same folder than OpenEXR to get valid include paths
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>/IlmBase
       )
set(ILMBASE_CMAKE_FLAGS -DILMBASE_ROOT=${BUILD_DIR}/openexr_build -DILMBASE_INCLUDE_PATH=${BUILD_DIR}/openexr_build/include)

# Add OpenEXR
ExternalProject_Add(openexr
       # URL https://github.com/openexr/openexr/archive/v2.2.1.tar.gz
       # The release 2.2.1 has troubles with C++17, which breaks compilation with recent compilers.
       # The problem has been fixed https://github.com/openexr/openexr/issues/235
       # but there is no release yet, so we use the development version.
       # GIT_REPOSITORY https://github.com/openexr/openexr
       # Use the latest commit with g++4.X compatibility
       # GIT_TAG 74b5c1dc2dfbdce74987a57f5e011dc711f9ca65
       # Finally use a custom version for a cmake fix
       GIT_REPOSITORY https://github.com/alicevision/openexr
       GIT_TAG develop_compatibility_gxx4
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openexr
       BINARY_DIR ${BUILD_DIR}/openexr_build
       INSTALL_DIR ${BUILD_DIR}/openexr_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DILMBASE_PACKAGE_PREFIX=${BUILD_DIR}/openexr_build -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> <SOURCE_DIR>/OpenEXR
       DEPENDS ilmbase
       )
set(OPENEXR_CMAKE_FLAGS -DOPENEXR_ROOT=${BUILD_DIR}/openexr_build -DOPENEXR_INCLUDE_PATH=${BUILD_DIR}/openexr_build/include)

# Add LibTiff
ExternalProject_Add(tiff
       URL http://download.osgeo.org/libtiff/tiff-4.0.9.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tiff
       BINARY_DIR ${BUILD_DIR}/tiff_build
       INSTALL_DIR ${BUILD_DIR}/tiff_build
       CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> 
       )

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
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/png
       BINARY_DIR ${BUILD_DIR}/png_build
       INSTALL_DIR ${BUILD_DIR}/png_build
       CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> 
       )
SET(PNG_CMAKE_FLAGS -DPNG_LIBRARY=${BUILD_DIR}/png_build/lib/libpng.so -DPNG_PNG_INCLUDE_DIR=${BUILD_DIR}/png_build/include)
endif()

if(AV_BUILD_JPEG)
set(JPEG_TARGET turbojpeg)
# Add turbojpeg
ExternalProject_Add(${JPEG_TARGET}
       URL https://github.com/libjpeg-turbo/libjpeg-turbo/archive/1.5.3.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg
       INSTALL_DIR ${BUILD_DIR}/turbojpeg_build
       CONFIGURE_COMMAND cd <BINARY_DIR> && autoreconf -fiv && ./configure --prefix=<INSTALL_DIR> 
       )
SET(JPEG_CMAKE_FLAGS -DJPEG_LIBRARY=${BUILD_DIR}/turbojpeg_build/lib/libjpeg.so -DJPEG_INCLUDE_DIR=${BUILD_DIR}/turbojpeg_build/include)
endif()

# Add Boost
ExternalProject_Add(boost
       URL https://github.com/alicevision/AliceVisionDependencies/releases/download/boost-src.1.61.0/boost_1_61_0.tar.gz
       #    http://sourceforge.net/projects/boost/files/boost/1.61.0/boost_1_61_0.tar.bz2
       #    https://dl.bintray.com/boostorg/release/1.61.0/source/boost_1_61_0.tar.gz
       # GIT_REPOSITORY https://github.com/boostorg/boost.git
       # GIT_TAG boost-1.61.0
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/boost
       BINARY_DIR ${BUILD_DIR}/boost_build
       INSTALL_DIR ${BUILD_DIR}/boost_build
       CONFIGURE_COMMAND cd <SOURCE_DIR> && ./bootstrap.${SCRIPT_EXTENSION} --prefix=<INSTALL_DIR> --with-libraries=atomic,container,date_time,exception,filesystem,graph,log,math,program_options,regex,serialization,system,test,thread
       BUILD_COMMAND cd <SOURCE_DIR> && ./b2 --prefix=<INSTALL_DIR> variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE} link=shared threading=multi -j8
       INSTALL_COMMAND cd <SOURCE_DIR> && ./b2 variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE} link=shared threading=multi install
       )
set(BOOST_CMAKE_FLAGS -DBOOST_ROOT=${BUILD_DIR}/boost_build)

# Add OpenImageIO
ExternalProject_Add(openimageio
       URL https://github.com/OpenImageIO/oiio/archive/Release-1.8.11.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openimageio
       BINARY_DIR ${BUILD_DIR}/openimageio_build
       INSTALL_DIR ${BUILD_DIR}/openimageio_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DCMAKE_PREFIX_PATH="${BUILD_DIR}/png_build;${BUILD_DIR}/turbojpeg_build" -DBOOST_ROOT=${BUILD_DIR}/boost_build -DILMBASE_HOME=${BUILD_DIR}/openexr_build -DOPENEXR_HOME=${BUILD_DIR}/openexr_build -DTIFF_INCLUDE_DIR=${BUILD_DIR}/tiff_build/include -DTIFF_LIBRARIES=${BUILD_DIR}/tiff_build/lib/libtiff.so ${PNG_CMAKE_FLAGS} ${JPEG_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR> -DUSE_PYTHON=OFF -DUSE_OPENCV=OFF -DUSE_OPENGL=OFF
       DEPENDS boost openexr tiff ${PNG_TARGET} ${JPEG_TARGET}
       )
# TODO: Raw, openjpeg
#  -DLibRaw_INCLUDE_DIR=$LIBRAW_INCLUDE_DIR
#  -DOPENJPEG_INCLUDE_DIR=$OPENJPEG_INCLUDE_DIR/openjpeg-2.0 -DOPENJPEG_OPENJP2_LIBRARIES=$OPENJPEG_OPENJP2_LIBRARIES
set(OPENIMAGEIO_CMAKE_FLAGS -DOPENIMAGEIO_LIBRARY_DIR_HINTS=${BUILD_DIR}/openimageio_build -DOPENIMAGEIO_INCLUDE_DIR_HINTS=${BUILD_DIR}/openimageio_build)

# Add Alembic: I/O for Point Cloud and Cameras
ExternalProject_Add(alembic
       URL https://github.com/alembic/alembic/archive/1.7.5.tar.gz
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/alembic
       BINARY_DIR ${BUILD_DIR}/alembic_build
       INSTALL_DIR ${BUILD_DIR}/alembic_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} ${ILMBASE_CMAKE_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS boost ilmbase
       )
set(ALEMBIC_CMAKE_FLAGS -DAlembic_DIR:PATH=${BUILD_DIR}/alembic_build/lib/cmake/Alembic)

set(AV_DEPS ${JPEG_TARGET} ${PNG_TARGET} boost openimageio openexr alembic geogram ${CUDA_TARGET} ceres)
set(AV_DEPS_CMAKE_PREFIX_PATH "${BUILD_DIR}/boost_build;${BUILD_DIR}/openimageio_build;${BUILD_DIR}/openexr_build;${BUILD_DIR}/openexr_build;${BUILD_DIR}/alembic_build;${BUILD_DIR}/geogram_install;${BUILD_DIR}/cuda_build;${BUILD_DIR}/ceres_build/lib64/cmake/Ceres;${BUILD_DIR}/alembic_build/lib/cmake/Alembic")

ExternalProject_Add(aliceVision
       PREFIX ${CMAKE_CURRENT_SOURCE_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 1
       SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src
       BINARY_DIR ${BUILD_DIR}/aliceVision_build
       INSTALL_DIR ${BUILD_DIR}/aliceVision_build
       CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DALICEVISION_ROOT=${ALICEVISION_ROOT} -DALICEVISION_USE_ALEMBIC=ON -DMINIGLOG=ON -DALICEVISION_USE_OPENCV=OFF -DALICEVISION_USE_OPENGV=OFF -DALICEVISION_USE_CUDA=${AV_USE_CUDA} -DALICEVISION_BUILD_DOC=OFF -DALICEVISION_BUILD_EXAMPLES=OFF ${EIGEN_CMAKE_FLAGS} ${ILMBASE_CMAKE_FLAGS} ${OPENIMAGEIO_CMAKE_FLAGS} ${OPENEXR_CMAKE_FLAGS} ${BOOST_CMAKE_FLAGS} ${ALEMBIC_CMAKE_FLAGS} ${GEOGRAM_CMAKE_FLAGS} ${CERES_CMAKE_FLAGS} ${CUDA_CMAKE_FLAGS} -DALICEVISION_BUILD_SHARED=ON -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS boost ceres eigen alembic openimageio geogram ${CUDA_TARGET}
       )

if(CMAKE_INSTALL_PREFIX)
  foreach(LIBNAME ${AV_DEPS})
    set(DEP_FOLDER ${BUILD_DIR}/${LIBNAME}_build)
    # Install dependencies libraries
    if(INSTALL_DEPS_BUILD)
      # Install dependencies build files
      install(
        DIRECTORY ${DEP_FOLDER}/bin ${DEP_FOLDER}/include ${DEP_FOLDER}/share
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        )
      install(
        DIRECTORY ${DEP_FOLDER}/lib/ ${DEP_FOLDER}/lib64/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        )
    else()
      install(
        DIRECTORY ${DEP_FOLDER}/lib/ ${DEP_FOLDER}/lib64/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        FILES_MATCHING PATTERN "lib*.so*"
        )
    endif()
  endforeach()
  # Install AliceVision's build files
  install(
    DIRECTORY ${BUILD_DIR}/aliceVision_build/bin ${BUILD_DIR}/aliceVision_build/lib ${BUILD_DIR}/aliceVision_build/include ${BUILD_DIR}/aliceVision_build/share
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    )
endif()

else()
# Standard cmake project without internal support for dependencies

# ==============================================================================
# Add sources
# ==============================================================================
add_subdirectory(src)

endif()

# Bundle target (see src/cmake/MakeBundle.cmake)
# Note: require that the install rule has been executed
add_custom_target(bundle
                  ${CMAKE_COMMAND}
                    # Script arguments:
                    -DBUNDLE_INSTALL_PREFIX=${ALICEVISION_BUNDLE_PREFIX}
                    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                    # Script:
                    -P ${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/MakeBundle.cmake
                  )
set_target_properties(bundle PROPERTIES FOLDER Packaging)


