#Build rules for all dependencies 
include(ExternalProject)

set(AV_BUILD_DEPENDENCIES_PARALLEL 1
    CACHE STRING "Number of cores to use when building dependencies (0 - use the number of cores of the processor)"
)
set(AV_ONNX_APPLE_ARCH "arm64" CACHE STRING "Version to download OFF Apple [arm64, x86_64]")

option(AV_BUILD_CUDA "Enable building an embedded Cuda" OFF)
option(AV_BUILD_ZLIB "Enable building an embedded ZLIB" OFF)
option(AV_BUILD_ASSIMP "Enable building an embedded ASSIMP" ON)
option(AV_BUILD_TIFF "Enable building an embedded Tiff" ON)
option(AV_BUILD_JPEG "Enable building an embedded Jpeg" ON)
option(AV_BUILD_PNG "Enable building an embedded Png" ON)
option(AV_BUILD_LIBRAW "Enable building an embedded libraw" ON)
option(AV_BUILD_POPSIFT "Enable building an embedded PopSift" ON)
option(AV_BUILD_CCTAG "Enable building an embedded CCTag" ON)
option(AV_BUILD_APRILTAG "Enable building an embedded AprilTag" ON)
option(AV_BUILD_OPENGV "Enable building an embedded OpenGV" ON)
option(AV_BUILD_OPENCV "Enable building an embedded OpenCV" ON)
option(AV_BUILD_ONNXRUNTIME "Enable building an embedded ONNX runtime" ON)
option(AV_BUILD_LAPACK "Enable building an embedded Lapack" ON)
option(AV_BUILD_SUITESPARSE "Enable building an embedded SuiteSparse" ON)
option(AV_BUILD_FFMPEG "Enable building an embedded FFMpeg" ON)
option(AV_BUILD_VPX "Enable building an embedded libvpx required for ffmpeg" ON)
option(AV_BUILD_COINUTILS "Enable building an embedded CoinUtils" ON)
option(AV_BUILD_OSI "Enable building an embedded Osi" ON)
option(AV_BUILD_CLP "Enable building an embedded Clp" ON)
option(AV_BUILD_FLANN "Enable building an embedded Flann" ON)
option(AV_BUILD_LEMON "Enable building an embedded LEMON library" ON)
option(AV_BUILD_PCL "Enable building an embedded PointCloud library" OFF)
option(AV_BUILD_USD "Enable building an embedded USD library" OFF)
option(AV_BUILD_GEOGRAM "Enable building an embedded Geogram library" ON)
option(AV_BUILD_TBB "Enable building an embedded TBB library" ON)
option(AV_BUILD_EIGEN "Enable building an embedded Eigen library" ON)
option(AV_BUILD_EXPAT "Enable building an embedded Expat library" ON)
option(AV_BUILD_OPENEXR "Enable building an embedded OpenExr library" ON)
option(AV_BUILD_ALEMBIC "Enable building an embedded Alembic library" ON)
option(AV_BUILD_OPENIMAGEIO "Enable building an embedded OpenImageIO library" ON)
option(AV_BUILD_BOOST "Enable building an embedded Boost library" ON)
option(AV_BUILD_CERES "Enable building an embedded Ceres library" ON)
option(AV_BUILD_SWIG "Enable building an embedded SWIG library" ON)

if(AV_BUILD_DEPENDENCIES_PARALLEL EQUAL 0)
    cmake_host_system_information(RESULT AV_BUILD_DEPENDENCIES_PARALLEL QUERY NUMBER_OF_LOGICAL_CORES)
endif()

##########LOGGING#########""
message(STATUS "")
message(STATUS "AV_BUILD_CUDA: ${AV_BUILD_CUDA}")
message(STATUS "AV_BUILD_ZLIB: ${AV_BUILD_ZLIB}")
message(STATUS "AV_BUILD_ASSIMP: ${AV_BUILD_ASSIMP}")
message(STATUS "AV_BUILD_TIFF: ${AV_BUILD_TIFF}")
message(STATUS "AV_BUILD_JPEG: ${AV_BUILD_JPEG}")
message(STATUS "AV_BUILD_PNG: ${AV_BUILD_PNG}")
message(STATUS "AV_BUILD_LIBRAW: ${AV_BUILD_LIBRAW}")
message(STATUS "AV_BUILD_CCTAG: ${AV_BUILD_CCTAG}")
message(STATUS "AV_BUILD_APRILTAG: ${AV_BUILD_APRILTAG}")
message(STATUS "AV_BUILD_POPSIFT: ${AV_BUILD_POPSIFT}")
message(STATUS "AV_BUILD_OPENGV: ${AV_BUILD_OPENGV}")
message(STATUS "AV_BUILD_OPENCV: ${AV_BUILD_OPENCV}")
message(STATUS "AV_BUILD_ONNXRUNTIME: ${AV_BUILD_ONNXRUNTIME}")
if(APPLE)
    message(STATUS "AV_ONNX_APPLE_ARCH: ${AV_ONNX_APPLE_ARCH}")
endif()
message(STATUS "AV_BUILD_LAPACK: ${AV_BUILD_LAPACK}")
message(STATUS "AV_BUILD_SUITESPARSE: ${AV_BUILD_SUITESPARSE}")
message(STATUS "AV_BUILD_FFMPEG: ${AV_BUILD_FFMPEG}")
message(STATUS "AV_BUILD_VPX: ${AV_BUILD_VPX}")
message(STATUS "AV_USE_CUDA: ${AV_USE_CUDA}")
message(STATUS "AV_USE_OPENMP: ${AV_USE_OPENMP}")
message(STATUS "AV_BUILD_COINUTILS: ${AV_BUILD_COINUTILS}")
message(STATUS "AV_BUILD_OSI: ${AV_BUILD_OSI}")
message(STATUS "AV_BUILD_CLP: ${AV_BUILD_CLP}")
message(STATUS "AV_BUILD_FLANN: ${AV_BUILD_FLANN}")
message(STATUS "AV_BUILD_PCL: ${AV_BUILD_PCL}")
message(STATUS "AV_BUILD_USD: ${AV_BUILD_USD}")
message(STATUS "AV_BUILD_LEMON: ${AV_BUILD_LEMON}")
message(STATUS "AV_BUILD_GEOGRAM: ${AV_BUILD_GEOGRAM}")
message(STATUS "AV_BUILD_TBB ${AV_BUILD_TBB}")
message(STATUS "AV_BUILD_EIGEN ${AV_BUILD_EIGEN}")
message(STATUS "AV_BUILD_EXPAT ${AV_BUILD_EXPAT}")
message(STATUS "AV_BUILD_OPENEXR ${AV_BUILD_OPENEXR}")
message(STATUS "AV_BUILD_BOOST ${AV_BUILD_BOOST}")
message(STATUS "AV_BUILD_ALEMBIC ${AV_BUILD_ALEMBIC}")
message(STATUS "AV_BUILD_OPENIMAGEIO ${AV_BUILD_OPENIMAGEIO}")
message(STATUS "AV_BUILD_CERES ${AV_BUILD_CERES}")
message(STATUS "AV_BUILD_SWIG ${AV_BUILD_SWIG}")
message(STATUS "AV_BUILD_DEPENDENCIES_PARALLEL: ${AV_BUILD_DEPENDENCIES_PARALLEL}")
##########END LOGGING#########"

set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/external")

set(CMAKE_CORE_BUILD_FLAGS 
        -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} 
        -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS} 
        -DCMAKE_INSTALL_DO_STRIP:BOOL=${CMAKE_INSTALL_DO_STRIP} 
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} 
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} 
        -DCMAKE_CXX_STANDARD=17
)


#### START EXTERNAL ####
if(AV_BUILD_ZLIB)
    set(ZLIB_TARGET zlib)

    ExternalProject_Add(${ZLIB_TARGET}
        URL https://www.zlib.net/zlib-1.3.1.tar.gz
        URL_HASH SHA256=9a93b2b7dfdac77ceba5a558a580e74667dd6fede4585b91eefb60f03b72df23
        DOWNLOAD_DIR ${BUILD_DIR}/download/zlib
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/zlib
        BINARY_DIR ${BUILD_DIR}/zlib_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )
    set(ZLIB_CMAKE_FLAGS -DZLIB_ROOT=${CMAKE_INSTALL_PREFIX})
endif()

if(AV_USE_CUDA AND AV_BUILD_CUDA)
    # Add Cuda
    set(CUDA_TARGET cuda)
    set(CUDA_EXE cuda_12.0.0_525.60.13_linux.run)

    ExternalProject_Add(${CUDA_TARGET}
        # URL https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run
        # URL https://developer.nvidia.com/compute/cuda/9.2/Prod/local_installers/cuda_9.2.88_396.26_linux
        URL https://developer.download.nvidia.com/compute/cuda/12.0.0/local_installers/${CUDA_EXE}
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
        INSTALL_COMMAND sh ${BUILD_DIR}/src/${CUDA_EXE} --silent --no-opengl-libs --toolkit --toolkitpath=<INSTALL_DIR>
    )
    
    set(CUDA_CUDART_LIBRARY "")
    set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${CMAKE_INSTALL_PREFIX})
else()
    option(CUDA_TOOLKIT_ROOT_DIR "")
    if(CUDA_TOOLKIT_ROOT_DIR)
        set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${CUDA_TOOLKIT_ROOT_DIR})
    endif()
endif()

if(AV_BUILD_GEOGRAM)
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
        URL https://github.com/BrunoLevy/geogram/releases/download/v1.8.3/geogram_1.8.3.tar.gz
        URL_HASH MD5=06fa5a70c05830d103ff71c55da5bb53
        DOWNLOAD_DIR ${BUILD_DIR}/download/geogram
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/geogram
        BINARY_DIR ${BUILD_DIR}/geogram_internal_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
            ${ZLIB_CMAKE_FLAGS}
            -DVORPALINE_PLATFORM=${VORPALINE_PLATFORM}
            -DGEOGRAM_WITH_HLBFGS=OFF
            -DGEOGRAM_WITH_TETGEN=OFF
            -DGEOGRAM_WITH_GRAPHICS=OFF
            -DGEOGRAM_WITH_EXPLORAGRAM=OFF
            -DGEOGRAM_WITH_LUA=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${ZLIB_TARGET}
    )

    set(GEOGRAM_CMAKE_FLAGS 
        -DGEOGRAM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} 
        -DGEOGRAM_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/geogram1
    )
endif()

if(AV_BUILD_ASSIMP)
    set(ASSIMP_TARGET assimp)

    set(ASSIMP_BUILD_OPTIONS 
        -DASSIMP_BUILD_ASSIMP_TOOLS:BOOL=OFF 
        -DASSIMP_BUILD_TESTS:BOOL=OFF 
        -DASSIMP_BUILD_DRACO:BOOL=ON
    )

    set(ASSIMP_AV_VERSION 5.2.5)

    ExternalProject_Add(${ASSIMP_TARGET}
        URL https://github.com/assimp/assimp/archive/refs/tags/v5.2.5.tar.gz
        URL_HASH MD5=0b5a5a2714f1126b9931cdb95f512c91
        DOWNLOAD_DIR ${BUILD_DIR}/download/assimp
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/assimp
        BINARY_DIR ${BUILD_DIR}/assimp_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
            ${ASSIMP_BUILD_OPTIONS}
            ${ZLIB_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DASSIMP_WARNINGS_AS_ERRORS=OFF
            -DASSIMP_BUILD_TESTS=OFF
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${ZLIB_TARGET}
    )
    
    set(ASSIMP_CMAKE_FLAGS 
        -DAssimp_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/assimp-${ASSIMP_AV_VERSION}
    )
endif()

if(AV_BUILD_TBB)
    # Add Tbb
    set(TBB_TARGET tbb)

    ExternalProject_Add(${TBB_TARGET}
        URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.8.0.tar.gz
        URL_HASH MD5=392421c6f33ebd00edb57eba36054da9
        DOWNLOAD_DIR ${BUILD_DIR}/download/tbb
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbb
        BINARY_DIR ${BUILD_DIR}/tbb_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS} 
            -DTBB_TEST:BOOL=OFF 
            -DTBB_STRICT:BOOL=OFF 
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    set(TBB_CMAKE_FLAGS -DTBB_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/TBB)
endif()

if(AV_BUILD_EIGEN)
    # Add Eigen
    set(EIGEN_TARGET eigen)

    ExternalProject_Add(${EIGEN_TARGET}
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2
        URL_HASH MD5=132dde48fe2b563211675626d29f1707
        DOWNLOAD_DIR ${BUILD_DIR}/download/eigen
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/eigen
        BINARY_DIR ${BUILD_DIR}/eigen_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} 
            -DCMAKE_CXX_STANDARD=17
            ${EIGEN_CMAKE_ALIGNMENT_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    set(EIGEN_CMAKE_FLAGS
        ${EIGEN_CMAKE_ALIGNMENT_FLAGS}
        -DEigen3_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake
        -DEIGEN3_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/eigen3
        -DEIGEN_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/eigen3
        -DEigen_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/eigen3
    )
endif()

if(AV_BUILD_EXPAT)
    # Add Expat: XML parser
    set(EXPAT_TARGET expat)

    ExternalProject_Add(${EXPAT_TARGET}
        GIT_REPOSITORY https://github.com/libexpat/libexpat.git
        GIT_TAG R_2_5_0
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/expat
        BINARY_DIR ${BUILD_DIR}/libexpat_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
            -DEXPAT_BUILD_DOCS:BOOL=OFF
            -DEXPAT_BUILD_EXAMPLES:BOOL=OFF
            -DEXPAT_BUILD_TOOLS:BOOL=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>/expat
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )
endif()

if (AV_BUILD_ONNXRUNTIME)
    # Add ONNXRuntime

    # when changing the version remember to update all the 3 different hashes AV_ONNX_HASH with the following bash script (update the version variable)
    ##!/usr/bin/env bash
    # AV_ONNX_VERSION="1.12.0"
    # BASE_URL="https://github.com/microsoft/onnxruntime/releases/download/v${AV_ONNX_VERSION}"
    # platforms=("onnxruntime-linux-x64"  "onnxruntime-osx-arm64"  "onnxruntime-osx-x86_64" "onnxruntime-linux-aarch64")
    # # Iterate over the main options
    # for platform in "${platforms[@]}"; do
    #     AV_ONNX_FILENAME="${platform}-${AV_ONNX_VERSION}.tgz"
    #     echo "${AV_ONNX_FILENAME}"
    #     url="${BASE_URL}/${AV_ONNX_FILENAME}"; curl -sLO "$url" && checksum=$(sha256sum "${AV_ONNX_FILENAME}" | awk '{ print $1 }') && echo "SHA256 Checksum: $checksum" && rm "${AV_ONNX_FILENAME}"
    # done

    set(AV_ONNX_VERSION "1.12.0")
    if(APPLE)
        set(AV_ONNX_FILENAME_PREFIX "onnxruntime-osx-${AV_ONNX_APPLE_ARCH}")
        if(AV_ONNX_APPLE_ARCH STREQUAL "arm64")
            set(AV_ONNX_HASH "23117b6f5d7324d4a7c51184e5f808dd952aec411a6b99a1b6fd1011de06e300")
        elseif(AV_ONNX_APPLE_ARCH STREQUAL "x86_64")
            set(AV_ONNX_HASH "09b17f712f8c6f19bb63da35d508815b443cbb473e16c6192abfaa297c02f600")
        else()
            message(FATAL_ERROR "Unsupported arch version ${AV_ONNX_APPLE_ARCH} for Apple")
        endif()
    else()
        string(FIND "${CMAKE_HOST_SYSTEM_PROCESSOR}" "aarch64" POSITION)
        if(NOT POSITION EQUAL -1)
            set(AV_ONNX_FILENAME_PREFIX "onnxruntime-linux-aarch64")
            set(AV_ONNX_HASH "5820d9f343df73c63b6b2b174a1ff62575032e171c9564bcf92060f46827d0ac")
        else()        
            set(AV_ONNX_FILENAME_PREFIX "onnxruntime-linux-x64")
            set(AV_ONNX_HASH "5d503ce8540358b59be26c675e42081be14a3e833a5301926f555451046929c5")
        endif()
    endif()

    set(AV_ONNX_FILENAME "${AV_ONNX_FILENAME_PREFIX}-${AV_ONNX_VERSION}.tgz")

    set(ONNXRUNTIME_TARGET onnxruntime)
    
    ExternalProject_Add(${ONNXRUNTIME_TARGET}
        URL https://github.com/microsoft/onnxruntime/releases/download/v${AV_ONNX_VERSION}/${AV_ONNX_FILENAME}
        URL_HASH SHA256=${AV_ONNX_HASH}
        DOWNLOAD_DIR ${BUILD_DIR}/download/onnxruntime
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/onnxruntime
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        PREFIX ${BUILD_DIR}
        INSTALL_COMMAND sh -c "mkdir -p <INSTALL_DIR>/include <INSTALL_DIR>/lib && cp -r <SOURCE_DIR>/lib/* <INSTALL_DIR>/lib && cp -r <SOURCE_DIR>/include/* <INSTALL_DIR>/include"
        CONFIGURE_COMMAND ""
        UPDATE_COMMAND ""
        BUILD_COMMAND ""
        BUILD_ALWAYS 0
        BUILD_IN_SOURCE 0
    )
endif()

if(AV_BUILD_OPENGV)
    set(OPENGV_TARGET opengv)

    ExternalProject_Add(${OPENGV_TARGET}
        # Official repository
        # GIT_REPOSITORY https://github.com/laurentkneip/opengv.git
        # Our fork, with a fix:
        GIT_REPOSITORY https://github.com/alicevision/opengv.git
        # Use a custom commit with a fix to override the cxx standard from cmake command line
        GIT_TAG 65f7edccf5044d445d305580f79c50c2efcbd438
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opengv
        BINARY_DIR ${BUILD_DIR}/opengv_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            ${EIGEN_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${EIGEN_TARGET}
    )
    set(OPENGV_CMAKE_FLAGS -DOPENGV_DIR=${CMAKE_INSTALL_PREFIX})
endif()

if(AV_BUILD_OPENEXR)
    # Add OpenEXR
    set(OPENEXR_TARGET openexr)

    ExternalProject_Add(${OPENEXR_TARGET}
        # vfxplatform CY2022: 3.1.x
        URL https://github.com/AcademySoftwareFoundation/openexr/archive/v3.1.6.tar.gz
        URL_HASH MD5=da5daf4d7954c034921e7201bf815938
        DOWNLOAD_DIR ${BUILD_DIR}/download/openexr
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openexr
        BINARY_DIR ${BUILD_DIR}/openexr_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} 
                -DOPENEXR_BUILD_PYTHON_LIBS:BOOL=OFF 
                -DBUILD_TESTING:BOOL=OFF 
                -DOPENEXR_INSTALL_EXAMPLES:BOOL=OFF
                -DOPENEXR_BUILD_TOOLS:BOOL=OFF
                ${ZLIB_CMAKE_FLAGS} 
                -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> 
                <SOURCE_DIR>
        BUILD_COMMAND VERBOSE=1 $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${ZLIB_TARGET}
    )
    
    set(ILMBASE_CMAKE_FLAGS 
        -DILMBASE_ROOT=${CMAKE_INSTALL_PREFIX} 
        -DILMBASE_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include/OpenEXR
    )
    
    set(OPENEXR_CMAKE_FLAGS 
        ${ILMBASE_CMAKE_FLAGS} 
        -DOPENEXR_ROOT=${CMAKE_INSTALL_PREFIX} 
        -DOPENEXR_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include
    )
endif()

if(AV_BUILD_TIFF)
    # Add LibTiff
    set(TIFF_TARGET tiff)

    ExternalProject_Add(${TIFF_TARGET}
        URL http://download.osgeo.org/libtiff/tiff-4.5.0.tar.gz
        URL_HASH MD5=db9e220a1971acc64487f1d51a20dcaa
        DOWNLOAD_DIR ${BUILD_DIR}/download/tiff
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tiff
        BINARY_DIR ${BUILD_DIR}/tiff_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND <SOURCE_DIR>/configure 
            --prefix=<INSTALL_DIR>
            --disable-tests
            --disable-docs
            --disable-tools
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND $(MAKE) install
        DEPENDS ${ZLIB_TARGET}
    )

    set(TIFF_CMAKE_FLAGS 
        -DTIFF_LIBRARY=${CMAKE_INSTALL_PREFIX}/lib/libtiff${CMAKE_SHARED_LIBRARY_SUFFIX} 
        -DTIFF_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
    )
endif()

if(AV_BUILD_PNG)
    # Add LibPng
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm") 
        set(AV_PNG_ARM_NEON OFF)
    else()
        set(AV_PNG_ARM_NEON off)
    endif()

    set(PNG_TARGET png)

    ExternalProject_Add(
        ${PNG_TARGET}
        URL https://download.sourceforge.net/libpng/libpng-1.6.39.tar.gz
        URL_HASH MD5=93b8e79a008747e70f7704f600349559
        DOWNLOAD_DIR ${BUILD_DIR}/download/libpng
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/png
        BINARY_DIR ${BUILD_DIR}/png_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}

        CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
            ${ZLIB_CMAKE_FLAGS}
            -DPNG_ARM_NEON=${AV_PNG_ARM_NEON}
            -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${ZLIB_TARGET}
    )

    set(PNG_CMAKE_FLAGS 
        -DPNG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libpng${CMAKE_SHARED_LIBRARY_SUFFIX} 
        -DPNG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
    )
endif()

if(AV_BUILD_JPEG)
    # Add LibPng
    set(JPEG_TARGET turbojpeg)

    ExternalProject_Add(
        ${JPEG_TARGET}
        URL https://github.com/libjpeg-turbo/libjpeg-turbo/archive/2.1.5.1.tar.gz
        URL_HASH MD5=33f72421d83ba487ff7b5c81e8765185
        DOWNLOAD_DIR ${BUILD_DIR}/download/libjpeg-turbo
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            ${ZLIB_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND $(MAKE) install
        DEPENDS ${ZLIB_TARGET}
    )

    set(JPEG_CMAKE_FLAGS 
        -DJPEG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libjpeg${CMAKE_SHARED_LIBRARY_SUFFIX} 
        -DJPEG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
    )
endif()

if(AV_BUILD_LIBRAW)
    # Add libraw
    set(LIBRAW_TARGET libraw)

    ExternalProject_Add(libraw_cmake
        GIT_REPOSITORY https://github.com/LibRaw/LibRaw-cmake
        GIT_TAG 6e26c9e73677dc04f9eb236a97c6a4dc225ba7e8
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
        GIT_REPOSITORY https://github.com/LibRaw/LibRaw
        GIT_TAG 0.21.1
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/libraw
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/libraw
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}

        # Native libraw configure script doesn't work OFF centos 7 (autoconf 2.69)
        # CONFIGURE_COMMAND autoconf && ./configure --enable-jpeg --enable-openmp --disable-examples --prefix=<INSTALL_DIR>
        # Use cmake build system (not maintained by libraw devs)
        CONFIGURE_COMMAND 
            cp <SOURCE_DIR>_cmake/CMakeLists.txt . &&
            cp -rf <SOURCE_DIR>_cmake/cmake . &&
            ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
            -DENABLE_OPENMP=${AV_USE_OPENMP}
            -DENABLE_LCMS=ON
            -DENABLE_EXAMPLES=OFF
            ${ZLIB_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DINSTALL_CMAKE_MODULE_PATH:PATH=<INSTALL_DIR>/cmake
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS libraw_cmake ${ZLIB_TARGET}
    )

    set(LIBRAW_CMAKE_FLAGS 
        -DLIBRAW_PATH=${CMAKE_INSTALL_PREFIX} 
        -DPC_LIBRAW_INCLUDEDIR=${CMAKE_INSTALL_PREFIX}/include 
        -DPC_LIBRAW_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib 
        -DPC_LIBRAW_R_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib
    )
endif()

if(AV_BUILD_BOOST)
    # Add Boost
    set(BOOST_TARGET boost)

    if(WIN32)
        set(SCRIPT_EXTENSION bat)
    else()
        set(SCRIPT_EXTENSION sh)
    endif()
    
    ExternalProject_Add(${BOOST_TARGET}
        URL https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.bz2
        URL_HASH MD5=df7dc2fc6de751753198a5bf70210da7
        DOWNLOAD_DIR ${BUILD_DIR}/download/boost
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/boost
        BINARY_DIR ${BUILD_DIR}/boost_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            cd <SOURCE_DIR> && 
            ./bootstrap.${SCRIPT_EXTENSION} --prefix=<INSTALL_DIR> --with-libraries=atomic,container,date_time,exception,graph,iostreams,json,log,math,program_options,regex,serialization,system,test,thread,stacktrace,timer
        BUILD_COMMAND 
            cd <SOURCE_DIR> && 
            ./b2 --prefix=<INSTALL_DIR> variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE} cxxstd=11 link=shared threading=multi -j8
        INSTALL_COMMAND 
            cd <SOURCE_DIR> && 
            ./b2 variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE}  cxxstd=11 link=shared threading=multi install
        DEPENDS ${ZLIB_TARGET}
    )

    set(BOOST_CMAKE_FLAGS -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX})
endif()

if(AV_BUILD_FFMPEG)
    if(AV_BUILD_VPX)
        set(VPX_TARGET libvpx)

        ExternalProject_add(${VPX_TARGET}
            GIT_REPOSITORY https://chromium.googlesource.com/webm/libvpx.git
            GIT_TAG v1.13.0
            GIT_PROGRESS OFF
            PREFIX ${BUILD_DIR}
            BUILD_IN_SOURCE 0
            BUILD_ALWAYS 0
            UPDATE_COMMAND ""
            INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
            CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR>
                --enable-shared --disable-static --disable-examples
            BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        )
    endif()

    set(FFMPEG_TARGET ffmpeg)

    ExternalProject_add(${FFMPEG_TARGET}
        URL http://ffmpeg.org/releases/ffmpeg-5.1.2.tar.bz2
        URL_HASH MD5=53ce2a391fe1db4b5ce5c43b9ea9a814
        DOWNLOAD_DIR ${BUILD_DIR}/download/ffmpeg
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg
        UPDATE_COMMAND ""
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND <SOURCE_DIR>/configure 
            --prefix=<INSTALL_DIR>
            --extra-cflags="-I<INSTALL_DIR>/include"
            --extra-ldflags="-L<INSTALL_DIR>/lib"
            --enable-shared
            --disable-static
            --disable-gpl
            --enable-nonfree
            --enable-libvpx
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${VPX_TARGET}
    )
endif()

if(AV_BUILD_FLANN)
    # Add lz4 for flann
    set(LZ4_TARGET lz4)

    ExternalProject_Add(${LZ4_TARGET}
        GIT_REPOSITORY https://github.com/lz4/lz4
        GIT_TAG v1.9.4
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/${LZ4_TARGET}
        BINARY_DIR ${BUILD_DIR}/${LZ4_TARGET}_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>/build/cmake/
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL} install 
    )
    set(LZ4_CMAKE_FLAGS -Dlz4_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/lz4/)

    set(FLANN_TARGET flann)
    ExternalProject_Add(${FLANN_TARGET}
        GIT_REPOSITORY https://github.com/alicevision/flann
        GIT_TAG 46e72429ef60ce9c413fa926ac7729f8dee96395
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/${FLANN_TARGET}
        BINARY_DIR ${BUILD_DIR}/${FLANN_TARGET}_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} -E env PKG_CONFIG_PATH=${CMAKE_INSTALL_PREFIX}/lib64/pkgconfig/
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            -DBUILD_C_BINDINGS:BOOL=OFF
            -DBUILD_EXAMPLES=OFF
            -DBUILD_TESTS:BOOL=OFF
            -DBUILD_DOC:BOOL=OFF
            -DBUILD_PYTHON_BINDINGS:BOOL=OFF
            -DBUILD_MATLAB_BINDINGS:BOOL=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL} install
        DEPENDS ${LZ4_TARGET}
    )

    set(FLANN_CMAKE_FLAGS -Dflann_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/flann/)
endif()

if(AV_BUILD_PCL)
    # Add Point Cloud Library
    set(PCL_TARGET pcl)

    ExternalProject_Add(${PCL_TARGET}
        URL https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.13.0.tar.gz
        URL_HASH MD5=987a5f6e440407a2bcae10c1022568b0
        DOWNLOAD_DIR ${BUILD_DIR}/download/${PCL_TARGET}
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PCL_TARGET}
        BINARY_DIR ${BUILD_DIR}/${PCL_TARGET}_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            ${EIGEN_CMAKE_FLAGS}
            ${BOOST_CMAKE_FLAGS}
            ${PNG_CMAKE_FLAGS}
            ${CUDA_CMAKE_FLAGS}
            -DWITH_CUDA:BOOL=${AV_USE_CUDA}
            -DWITH_OPENGL:BOOL=OFF
            -DWITH_OPENMP:BOOL=ON
            -DWITH_LIBUSB:BOOL=OFF
            -DWITH_VTK:BOOL=OFF
            -DWITH_PCAP:BOOL=OFF
            ${FLANN_CMAKE_FLAGS}
            ${LZ4_CMAKE_FLAGS}
            ${ZLIB_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${FLANN_TARGET} ${LZ4_TARGET} ${EIGEN_TARGET} ${BOOST_TARGET} ${PNG_TARGET} ${CUDA_TARGET} ${ZLIB_TARGET}
    )

    set(PCL_CMAKE_FLAGS -DPCL_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/pcl-1.12/)
endif()

if(AV_BUILD_USD)
    set(USD_TARGET pxr)

    ExternalProject_Add(${USD_TARGET}
        GIT_REPOSITORY https://github.com/PixarAnimationStudios/USD.git
        GIT_TAG v23.05
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND ""
        INSTALL_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/usd
        BINARY_DIR ${BUILD_DIR}/usd_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        BUILD_COMMAND python ${CMAKE_CURRENT_BINARY_DIR}/usd/build_scripts/build_usd.py
            --build-shared
            --no-examples
            --no-tools
            --no-ptex
            --no-prman
            --no-openimageio
            --no-opencolorio
            --no-alembic
            --no-draco
            --no-materialx
            --no-tutorials
            --no-tests
            --no-docs
            --no-python
            <INSTALL_DIR>
    )

    set(USD_CMAKE_FLAGS -Dpxr_DIR:PATH=${CMAKE_INSTALL_PREFIX})
endif()

if(AV_BUILD_COINUTILS)
    set(COINUTILS_TARGET coinutils)

    ExternalProject_Add(${COINUTILS_TARGET}
        GIT_REPOSITORY https://github.com/alicevision/CoinUtils
        GIT_TAG b29532e31471d26dddee99095da3340e80e8c60c
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/coinutils
        BINARY_DIR ${BUILD_DIR}/coinutils_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    set(COINUTILS_CMAKE_FLAGS -DCoinUtils_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/coinutils)
endif()

if(AV_BUILD_OSI)
    set(OSI_TARGET osi)

    ExternalProject_Add(${OSI_TARGET}
        GIT_REPOSITORY https://github.com/alicevision/Osi
        GIT_TAG 52bafbabf8d29bcfd57818f0dd50ee226e01db7f
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/osi
        BINARY_DIR ${BUILD_DIR}/osi_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${COINUTILS_TARGET}
    )

    set(OSI_CMAKE_FLAGS -DOsi_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/osi)
endif()

if(AV_BUILD_CLP)
    set(CLP_TARGET clp)

    ExternalProject_Add(${CLP_TARGET}
        GIT_REPOSITORY https://github.com/alicevision/Clp
        GIT_TAG 4da587acebc65343faafea8a134c9f251efab5b9
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/clp
        BINARY_DIR ${BUILD_DIR}/clp_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${COINUTILS_TARGET} ${OSI_TARGET}
    )

    set(CLP_CMAKE_FLAGS -DClp_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/clp)
endif()

if(AV_BUILD_POPSIFT)
    # Add PopSift
    set(POPSIFT_TARGET popsift)

    ExternalProject_Add(${POPSIFT_TARGET}
        GIT_REPOSITORY https://github.com/alicevision/popsift
        GIT_TAG 4b4b2478d5f0cdb6c4215a031572e951c0c2502e
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/popsift
        BINARY_DIR ${BUILD_DIR}/popsift_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            ${BOOST_CMAKE_FLAGS}
            ${CUDA_CMAKE_FLAGS}
            -DPopSift_BUILD_EXAMPLES:BOOL=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${BOOST_TARGET} ${CUDA_TARGET}
    )

    set(POPSIFT_CMAKE_FLAGS -DPopSift_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/PopSift)
endif()

if(AV_BUILD_APRILTAG)
    # Add AprilTag
    set(APRILTAG_TARGET apriltag)

    ExternalProject_Add(${APRILTAG_TARGET}
        GIT_REPOSITORY https://github.com/AprilRobotics/apriltag
        GIT_TAG v3.2.0
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/apriltag
        BINARY_DIR ${BUILD_DIR}/apriltag_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            -DBUILD_PYTHON_WRAPPER=OFF
            -DOpenCV_FOUND=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    set(APRILTAG_CMAKE_FLAGS -Dapriltag_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/apriltag/cmake)
endif()

if(AV_BUILD_OPENCV)
    set(OPENCV_TARGET opencv)

    ExternalProject_Add(opencv_contrib
        URL https://github.com/opencv/opencv_contrib/archive/4.7.0.zip
        URL_HASH MD5=a3969f1db6732340e492c0323178f6f1
        DOWNLOAD_DIR ${BUILD_DIR}/download/opencv_contrib
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv_contrib
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
    )

    ExternalProject_Add(${OPENCV_TARGET}
        URL https://github.com/opencv/opencv/archive/4.7.0.zip
        URL_HASH MD5=481a9ee5b0761978832d02d8861b8156
        DOWNLOAD_DIR ${BUILD_DIR}/download/opencv
        UPDATE_COMMAND ""
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv
        BINARY_DIR ${BUILD_DIR}/opencv_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            -DOPENCV_EXTRA_MODULES_PATH=${CMAKE_CURRENT_BINARY_DIR}/opencv_contrib/modules
            ${ZLIB_CMAKE_FLAGS} ${TBB_CMAKE_FLAGS}
            ${TIFF_CMAKE_FLAGS} ${PNG_CMAKE_FLAGS} ${JPEG_CMAKE_FLAGS} ${LIBRAW_CMAKE_FLAGS}
            -DWITH_TBB=ON
            -DWITH_FFMPEG=${AV_BUILD_FFMPEG}
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
            -DWITH_OPENEXR=OFF  # Build error OFF IlmBase includes without "OpenEXR/" prefix
            -DENABLE_PRECOMPILED_HEADERS=OFF
            -DBUILD_SHARED_LIBS=ON
            -DWITH_CUDA=OFF
            -DWITH_OPENCL=OFF
            -DBUILD_TESTS=OFF
            -DBUILD_LIST=core,improc,photo,objdetect,video,imgcodecs,videoio,features2d,xfeatures2d,version,mcc,optflow
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}

        DEPENDS opencv_contrib 
            ${TBB_TARGET} ${ZLIB_TARGET} ${OPENEXR_TARGET} 
            ${TIFF_TARGET} ${PNG_TARGET} ${JPEG_TARGET} 
            ${LIBRAW_TARGET} ${FFMPEG_TARGET}
    )
    
    set(OPENCV_CMAKE_FLAGS 
        -DOpenCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4 
        -DOPENCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4
    )
endif()

if(AV_BUILD_CCTAG)
    # Add CCTag
    set(CCTAG_TARGET cctag)

    ExternalProject_Add(${CCTAG_TARGET}
        GIT_REPOSITORY https://github.com/alicevision/CCTag
        GIT_TAG v1.0.3
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cctag
        BINARY_DIR ${BUILD_DIR}/cctag_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            ${BOOST_CMAKE_FLAGS}
            ${CUDA_CMAKE_FLAGS}
            ${OPENCV_CMAKE_FLAGS}
            ${EIGEN_CMAKE_FLAGS}
            ${TBB_CMAKE_FLAGS}
            -DCCTAG_WITH_CUDA:BOOL=${AV_USE_CUDA}
            -DCCTAG_BUILD_TESTS=OFF
            -DCCTAG_BUILD_APPS=OFF
            -DCCTAG_EIGEN_MEMORY_ALIGNMENT=ON
            -DCCTAG_CXX_STANDARD=17
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${BOOST_TARGET} ${CUDA_TARGET} ${OPENCV_TARGET} ${EIGEN_TARGET} ${TBB_TARGET}
    )

    set(CCTAG_CMAKE_FLAGS -DCCTag_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/CCTag)
endif()

if(AV_BUILD_ALEMBIC)
    # Add Alembic: I/O for Point Cloud and Cameras
    set(ALEMBIC_TARGET alembic)

    ExternalProject_Add(${ALEMBIC_TARGET}
        # vfxplatform CY2022 1.8.x
        URL https://github.com/alembic/alembic/archive/1.8.5.tar.gz
        URL_HASH MD5=fcd5b5492a005057e11b601b60ac9a49
        DOWNLOAD_DIR ${BUILD_DIR}/download/alembic
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/alembic
        BINARY_DIR ${BUILD_DIR}/alembic_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            ${ZLIB_CMAKE_FLAGS}
            ${ILMBASE_CMAKE_FLAGS}
            -DUSE_TESTS=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${ZLIB_TARGET}
    )

    set(ALEMBIC_CMAKE_FLAGS -DAlembic_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/Alembic)
endif()

if(AV_BUILD_OPENIMAGEIO)
    # Add OpenImageIO
    set(OPENIMAGEIO_TARGET openimageio)

    ExternalProject_Add(${OPENIMAGEIO_TARGET}
        URL https://github.com/AcademySoftwareFoundation/OpenImageIO/archive/refs/tags/v2.4.13.0.tar.gz
        URL_HASH MD5=30e8b433bb71a262a51f56a41fc50ac7
        DOWNLOAD_DIR ${BUILD_DIR}/download/oiio
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openimageio
        BINARY_DIR ${BUILD_DIR}/openimageio_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
            -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX}
            -DOIIO_BUILD_TESTS:BOOL=OFF
            -DOIIO_BUILD_TOOLS:BOOL=OFF
            -DILMBASE_HOME=${CMAKE_INSTALL_PREFIX}
            -DOPENEXR_HOME=${CMAKE_INSTALL_PREFIX}
            ${TIFF_CMAKE_FLAGS} ${ZLIB_CMAKE_FLAGS} ${PNG_CMAKE_FLAGS} ${JPEG_CMAKE_FLAGS} ${LIBRAW_CMAKE_FLAGS} ${OPENEXR_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
            -DSTOP_ON_WARNING=OFF
            -DUSE_FFMPEG=${AV_BUILD_FFMPEG}
            -DUSE_TURBOJPEG=${AV_BUILD_JPEG}
            -DUSE_LIBRAW=${AV_BUILD_LIBRAW}
            -DUSE_OPENEXR=${AV_BUILD_OPENEXR}
            -DUSE_TIFF=${AV_BUILD_TIFF}
            -DUSE_PNG=${AV_BUILD_PNG}
            -DUSE_PYTHON=OFF -DUSE_OPENCV=OFF -DUSE_OPENGL=OFF
            # TODO: build with libheif
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${TIFF_TARGET} ${PNG_TARGET} ${JPEG_TARGET} ${LIBRAW_TARGET} ${ZLIB_TARGET} ${FFMPEG_TARGET}
    )

    set(OPENIMAGEIO_CMAKE_FLAGS -DOpenImageIO_DIR=${CMAKE_INSTALL_PREFIX})
endif()

if(AV_BUILD_LAPACK)
    set(LAPACK_TARGET lapack)

    ExternalProject_Add(${LAPACK_TARGET}
        URL https://github.com/Reference-LAPACK/lapack/archive/v3.11.0.tar.gz
        URL_HASH MD5=595b064fd448b161cd711fe346f498a7
        DOWNLOAD_DIR ${BUILD_DIR}/download/lapack
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/lapack
        BINARY_DIR ${BUILD_DIR}/lapack_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${TBB_TARGET}
    )

    set(BLAS_LIBRARIES ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libblas${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(LAPACK_LIBRARIES ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/liblapack${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(LAPACK_CMAKE_FLAGS -DBLAS_LIBRARIES=${BLAS_LIBRARIES} -DLAPACK_LIBRARIES=${LAPACK_LIBRARIES})
endif()

if(AV_BUILD_SUITESPARSE)
    ExternalProject_add(gmp
        URL https://gmplib.org/download/gmp/gmp-6.2.1.tar.xz
        URL_HASH MD5=0b82665c4a92fd2ade7440c13fcaa42b
        DOWNLOAD_DIR ${BUILD_DIR}/download/gmp
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --enable-cxx
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    ExternalProject_add(mpfr
        URL https://ftp.gnu.org/gnu/mpfr/mpfr-4.2.0.tar.gz
        URL_HASH MD5=279b527503118a22bd0022e0d64807cb
        DOWNLOAD_DIR ${BUILD_DIR}/download/mpfr
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --with-gmp=<INSTALL_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS gmp
    )

    set(SUITESPARSE_TARGET suitesparse)
    if(APPLE)
        set(SUITESPARSE_INTERNAL_MAKE_CMD VERBOSE=1 MPFR_ROOT=${CMAKE_INSTALL_PREFIX} GMP_ROOT=${CMAKE_INSTALL_PREFIX} DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR} $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL} BLAS="${BLAS_LIBRARIES}" LAPACK="${LAPACK_LIBRARIES}" LAPACK_LIBRARIES="${LAPACK_LIBRARIES}")
    else()
        set(SUITESPARSE_INTERNAL_MAKE_CMD VERBOSE=1 MPFR_ROOT=${CMAKE_INSTALL_PREFIX} GMP_ROOT=${CMAKE_INSTALL_PREFIX} LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR} $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL} BLAS_LIBRARIES="${BLAS_LIBRARIES}" BLAS="${BLAS_LIBRARIES}" LAPACK="${LAPACK_LIBRARIES}" LAPACK_LIBRARIES="${LAPACK_LIBRARIES}")
    endif()

    ExternalProject_Add(${SUITESPARSE_TARGET}
        URL https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/v7.3.0.tar.gz
        URL_HASH MD5=6ff86003a85d73eb383d82db04af7373
        DOWNLOAD_DIR ${BUILD_DIR}/download/suitesparse
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND   cd <BINARY_DIR> && ${SUITESPARSE_INTERNAL_MAKE_CMD} library CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER} CMAKE_OPTIONS=-DBLAS_LIBRARIES=${BLAS_LIBRARIES}\ -DLAPACK_LIBRARIES=${LAPACK_LIBRARIES}\ -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        INSTALL_COMMAND cd <BINARY_DIR> && ${SUITESPARSE_INTERNAL_MAKE_CMD} install library INSTALL=<INSTALL_DIR> CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER} CMAKE_OPTIONS=-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        DEPENDS ${LAPACK_TARGET} mpfr
    )
    
    set(SUITESPARSE_CMAKE_FLAGS ${LAPACK_CMAKE_FLAGS} -DSUITESPARSE_INCLUDE_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/include -DSUITESPARSE_LIBRARY_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/lib)
endif()

if(AV_BUILD_CERES)
    # Add ceres-solver: A Nonlinear Least Squares Minimizer
    set(CERES_TARGET ceres)

    ExternalProject_Add(${CERES_TARGET}
        GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver
        GIT_TAG 2.2.0
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver
        BINARY_DIR ${BUILD_DIR}/ceres_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND 
            ${CMAKE_COMMAND} 
            ${CMAKE_CORE_BUILD_FLAGS}
            ${SUITESPARSE_CMAKE_FLAGS}
            -DSUITESPARSE:BOOL=ON
            -DLAPACK:BOOL=ON
            ${EIGEN_CMAKE_FLAGS}
            -DMINIGLOG=ON
            -DBUILD_EXAMPLES:BOOL=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${EIGEN_TARGET} ${SUITESPARSE_TARGET}
    )

    set(CERES_CMAKE_FLAGS ${SUITESPARSE_CMAKE_FLAGS} -DCeres_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/Ceres)
endif()

if(AV_BUILD_LEMON)
    # Add Lemon
    set(LEMON_TARGET LEMON)

    ExternalProject_Add(${LEMON_TARGET}
        GIT_REPOSITORY https://github.com/The-OpenROAD-Project/lemon-graph.git
        GIT_TAG 62ac75337e5a8d7221823f03e9cc782270cfef4b
        DOWNLOAD_DIR ${BUILD_DIR}/download/${LEMON_TARGET}
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/${LEMON_TARGET}
        BINARY_DIR ${BUILD_DIR}/${LEMON_TARGET}_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    set(LEMON_CMAKE_FLAGS -DLEMON_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/lemon/cmake)
endif()

if(AV_BUILD_SWIG)
    set(SWIG_TARGET SWIG)

    ExternalProject_Add(${SWIG_TARGET}
        GIT_REPOSITORY https://github.com/swig/swig
        GIT_TAG v4.2.0
        DOWNLOAD_DIR ${BUILD_DIR}/download/${SWIG_TARGET}
        PREFIX ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 0
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/${SWIG_TARGET}
        BINARY_DIR ${BUILD_DIR}/${SWIG_TARGET}_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
        BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    set(SWIG_CMAKE_FLAGS
        -DSWIG_DIR=${CMAKE_INSTALL_PREFIX}/share/swig/4.2.0
        -DSWIG_EXECUTABLE=${CMAKE_INSTALL_PREFIX}/bin-deps
    )
endif()

set(AV_DEPS
    ${ZLIB_TARGET}
    ${ASSIMP_TARGET}
    ${GEOGRAM_TARGET}
    ${CUDA_TARGET}
    ${TBB_TARGET}
    ${EIGEN_TARGET}
    ${ONNXRUNTIME_TARGET}
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
    ${APRILTAG_TARGET}
    ${POPSIFT_TARGET}
    ${EXPAT_TARGET}
    ${COINUTILS_TARGET}
    ${OSI_TARGET}
    ${CLP_TARGET}
    ${USD_TARGET}
    ${FLANN_TARGET}
    ${LZ4_TARGET}
    ${LEMON_TARGET}
    ${SWIG_TARGET}
)

if(AV_BUILD_ALICEVISION)
    # Build Alicevision super build mode
    ExternalProject_Add(aliceVision
        PREFIX ${CMAKE_CURRENT_SOURCE_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS 1
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src
        BINARY_DIR ${BUILD_DIR}/aliceVision_build
        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ${CMAKE_COMMAND}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DBUILD_SHARED_LIBS:BOOL=ON -DTARGET_ARCHITECTURE=core
        -DALICEVISION_ROOT=${ALICEVISION_ROOT}
        -DALICEVISION_USE_ALEMBIC=ON
        -DMINIGLOG=ON
        -DALICEVISION_USE_CCTAG=${AV_BUILD_CCTAG}
        -DALICEVISION_USE_APRILTAG=${AV_BUILD_APRILTAG}
        -DALICEVISION_USE_OPENCV=${AV_BUILD_OPENCV}
        -DALICEVISION_USE_OPENGV=${AV_BUILD_OPENGV}
        -DALICEVISION_USE_POPSIFT=${AV_BUILD_POPSIFT}
        -DALICEVISION_USE_CUDA=${AV_USE_CUDA}
        -DALICEVISION_BUILD_SWIG_BINDING=${AV_USE_SWIG}
        -DALICEVISION_BUILD_DOC=OFF
        -DALICEVISION_BUILD_EXAMPLES=OFF

        ${ZLIB_CMAKE_FLAGS}
        ${ASSIMP_CMAKE_FLAGS}
        ${EIGEN_CMAKE_FLAGS}
        ${OPENIMAGEIO_CMAKE_FLAGS}
        ${OPENEXR_CMAKE_FLAGS}
        ${BOOST_CMAKE_FLAGS}
        ${ALEMBIC_CMAKE_FLAGS}
        ${GEOGRAM_CMAKE_FLAGS}
        ${LAPACK_CMAKE_FLAGS}
        ${CERES_CMAKE_FLAGS}
        ${CUDA_CMAKE_FLAGS}
        ${POPSIFT_CMAKE_FLAGS}
        ${OPENGV_CMAKE_FLAGS}
        ${OPENCV_CMAKE_FLAGS}
        ${CCTAG_CMAKE_FLAGS}
        ${APRILTAG_CMAKE_FLAGS}
        ${EXPAT_CMAKE_FLAGS}
        ${COINUTILS_CMAKE_FLAGS} ${OSI_CMAKE_FLAGS} ${CLP_CMAKE_FLAGS}
        ${LZ4_CMAKE_FLAGS}
        ${FLANN_CMAKE_FLAGS}
        ${PCL_CMAKE_FLAGS}
        ${USD_CMAKE_FLAGS}
        ${SWIG_CMAKE_FLAGS}

        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
        DEPENDS ${AV_DEPS}
    )
endif()
