cmake_minimum_required(VERSION 3.11)
project(aliceVision LANGUAGES C CXX)

option(ALICEVISION_BUILD_DEPENDENCIES "Build all AliceVision dependencies" OFF)
set(AV_BUILD_DEPENDENCIES_PARALLEL 1
    CACHE STRING "Number of cores to use when building dependencies (0 - use the number of cores of the processor)")
option(AV_BUILD_CUDA "Enable building an embedded Cuda" OFF)
option(AV_BUILD_ZLIB "Enable building an embedded ZLIB" ON)
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
option(AV_BUILD_LAPACK "Enable building an embedded Lapack" ON)
option(AV_BUILD_SUITESPARSE "Enable building an embedded SuiteSparse" ON)
option(AV_BUILD_FFMPEG "Enable building an embedded FFMpeg" ON)
option(AV_BUILD_COINUTILS "Enable building an embedded CoinUtils" ON)
option(AV_BUILD_OSI "Enable building an embedded Osi" ON)
option(AV_BUILD_CLP "Enable building an embedded Clp" ON)
option(AV_BUILD_PCL "Enable building an embedded PointCloud library" ON)
option(AV_BUILD_ALICEVISION "Enable building of AliceVision" ON)
option(AV_EIGEN_MEMORY_ALIGNMENT "Enable Eigen memory alignment" OFF)

option(AV_USE_CUDA "Enable CUDA" ON)
# disable by default for AppleClang
option(AV_USE_OPENMP "Enable OpenMP" $<$<CXX_COMPILER_ID:"AppleClang">,OFF,ON>)

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
message(STATUS "AV_BUILD_LAPACK: ${AV_BUILD_LAPACK}")
message(STATUS "AV_BUILD_SUITESPARSE: ${AV_BUILD_SUITESPARSE}")
message(STATUS "AV_BUILD_FFMPEG: ${AV_BUILD_FFMPEG}")
message(STATUS "AV_USE_CUDA: ${AV_USE_CUDA}")
message(STATUS "AV_USE_OPENMP: ${AV_USE_OPENMP}")
message(STATUS "AV_BUILD_COINUTILS: ${AV_BUILD_COINUTILS}")
message(STATUS "AV_BUILD_OSI: ${AV_BUILD_OSI}")
message(STATUS "AV_BUILD_CLP: ${AV_BUILD_CLP}")
message(STATUS "AV_BUILD_PCL: ${AV_BUILD_PCL}")
message(STATUS "AV_BUILD_DEPENDENCIES_PARALLEL: ${AV_BUILD_DEPENDENCIES_PARALLEL}")
message(STATUS "")
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

if(AV_BUILD_DEPENDENCIES_PARALLEL EQUAL 0)
  cmake_host_system_information(RESULT AV_BUILD_DEPENDENCIES_PARALLEL QUERY NUMBER_OF_LOGICAL_CORES)
endif()

# ==============================================================================
# Use CMake ExternalProject to build all dependencies
# ==============================================================================
include(ExternalProject)

set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/external")

set(CMAKE_CORE_BUILD_FLAGS -DCMAKE_BUILD_TYPE=${DEPS_CMAKE_BUILD_TYPE} -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS} -DCMAKE_INSTALL_DO_STRIP:BOOL=${CMAKE_INSTALL_DO_STRIP} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER})

if(AV_BUILD_ZLIB)
set(ZLIB_TARGET zlib)
ExternalProject_Add(${ZLIB_TARGET}
       URL http://www.zlib.net/zlib-1.2.13.tar.gz
       URL_HASH SHA256=b3a24de97a8fdbc835b9833169501030b8977031bcb54b3b3ac13740f846ab30
       DOWNLOAD_DIR ${BUILD_DIR}/download/zlib
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/zlib
       BINARY_DIR ${BUILD_DIR}/zlib_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
           -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
           <SOURCE_DIR>
       BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
       )
set(ZLIB_CMAKE_FLAGS -DZLIB_ROOT=${CMAKE_INSTALL_PREFIX})
endif()


if(AV_BUILD_ASSIMP)
set(ASSIMP_TARGET assimp)
set(ASSIMP_BUILD_OPTIONS -DASSIMP_BUILD_ASSIMP_TOOLS:BOOL=OFF -DASSIMP_BUILD_TESTS:BOOL=OFF)
set(ASSIMP_AV_VERSION 5.0.1)
set(ASSIMP_FILENAME v${ASSIMP_AV_VERSION}.tar.gz)
ExternalProject_Add(${ASSIMP_TARGET}
      URL https://github.com/assimp/assimp/archive/refs/tags/${ASSIMP_FILENAME}
      URL_HASH MD5=eb7b7385a5c3194ab46d7f869d7ac6cf
      DOWNLOAD_DIR ${BUILD_DIR}/download/assimp
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/assimp
      BINARY_DIR ${BUILD_DIR}/assimp_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
          ${ASSIMP_BUILD_OPTIONS}
          -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
          <SOURCE_DIR>
      )
set(ASSIMP_CMAKE_FLAGS -DAssimp_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/assimp-${ASSIMP_AV_VERSION})
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
       URL https://github.com/alicevision/geogram/archive/v1.7.9.tar.gz
       URL_HASH MD5=dbdb9c1fd48ce277b570b0a23af3b518
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
       URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.5.0.tar.gz
       URL_HASH MD5=5e5f2ee22a0d19c0abbe7478f1c7ccf6
       DOWNLOAD_DIR ${BUILD_DIR}/download/tbb
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbb
       BINARY_DIR ${BUILD_DIR}/tbb_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DTBB_TEST:BOOL=OFF -DTBB_STRICT:BOOL=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  <SOURCE_DIR>
       BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
       )

set(TBB_CMAKE_FLAGS -DTBB_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/TBB)

# Eigen requires overaligned buffers for maximum efficiency (e.g. on AVX512 buffers may need to
# be aligned to 64 bytes). AliceVision currently does not support this. Fortunately this is fixed
# in C++17. While we can't upgrade to C++17 just yet, some compilers support overaligned
# allocation feature with a separate flag, so use it if alignment is enabled in Eigen.
# See https://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html
if(AV_EIGEN_MEMORY_ALIGNMENT)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 7.1)
    set(EIGEN_CMAKE_ALIGNMENT_FLAGS "-DCMAKE_CXX_FLAGS:STRING=-faligned-new")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 6.0)
    set(EIGEN_CMAKE_ALIGNMENT_FLAGS "-DCMAKE_CXX_FLAGS:STRING=-faligned-new")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.0)
    set(EIGEN_CMAKE_ALIGNMENT_FLAGS "-DCMAKE_CXX_FLAGS:STRING=-faligned-new")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 19.12)
    set(EIGEN_CMAKE_ALIGNMENT_FLAGS "-DCMAKE_CXX_FLAGS:STRING=/Zc:alignedNew")
  else()
    message(FATAL_ERROR "AV_EIGEN_MEMORY_ALIGNMENT is only supported starting GCC 7.1, Clang 6.0 and MSVC 2017 15.5")
  endif()
else()
  set(EIGEN_CMAKE_ALIGNMENT_FLAGS
      "-DCMAKE_CXX_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0"
      "-DCMAKE_C_FLAGS:STRING=-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0")
endif()

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
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
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

set(EXPAT_TARGET expat)
ExternalProject_Add(${EXPAT_TARGET}
       GIT_REPOSITORY https://github.com/libexpat/libexpat.git
       GIT_TAG R_2_4_8
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

if(AV_BUILD_OPENGV)
set(OPENGV_TARGET opengv)
ExternalProject_Add(${OPENGV_TARGET}
       GIT_REPOSITORY https://github.com/laurentkneip/opengv.git
       GIT_TAG 91f4b19c73450833a40e463ad3648aae80b3a7f3
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opengv
       BINARY_DIR ${BUILD_DIR}/opengv_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
           ${EIGEN_CMAKE_FLAGS}
           -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
           <SOURCE_DIR>
       BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
       DEPENDS ${EIGEN_TARGET}
       )
set(OPENGV_CMAKE_FLAGS -DOPENGV_DIR=${CMAKE_INSTALL_PREFIX})
endif()

if(AV_BUILD_LAPACK)
set(LAPACK_TARGET lapack)
ExternalProject_Add(${LAPACK_TARGET}
       URL https://github.com/Reference-LAPACK/lapack/archive/v3.10.0.tar.gz
       URL_HASH MD5=d70fc27a8bdebe00481c97c728184f09
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
      URL https://ftp.gnu.org/gnu/mpfr/mpfr-4.1.0.tar.gz
      URL_HASH MD5=81a97a9ba03590f83a30d26d4400ce39
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
    set(SUITESPARSE_INTERNAL_MAKE_CMD MPFR_ROOT=${CMAKE_INSTALL_PREFIX} GMP_ROOT=${CMAKE_INSTALL_PREFIX} DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR} $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL} BLAS="${BLAS_LIBRARIES}" LAPACK="${LAPACK_LIBRARIES}")
else()
    set(SUITESPARSE_INTERNAL_MAKE_CMD MPFR_ROOT=${CMAKE_INSTALL_PREFIX} GMP_ROOT=${CMAKE_INSTALL_PREFIX} LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR} $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL} BLAS="${BLAS_LIBRARIES}" LAPACK="${LAPACK_LIBRARIES}")
endif()
ExternalProject_Add(${SUITESPARSE_TARGET}
       # URL https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/v5.10.1.tar.gz  # requires gxx >= 4.9, centos 7 use gxx-4.8.5 by default
       # URL_HASH MD5=1
       GIT_REPOSITORY https://github.com/alicevision/SuiteSparse
       GIT_TAG fix/gmp_mpfr  # based on v5.8.1
       # DOWNLOAD_DIR ${BUILD_DIR}/download/suitesparse
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ""
       PATCH_COMMAND sh -c "test `nvcc --version | grep -o 'release [0-9]*' | grep -o '[0-9]*'` -gt 10" && sed -i /compute_30/d ${CMAKE_CURRENT_BINARY_DIR}/suitesparse/SuiteSparse_config/SuiteSparse_config.mk || true
       BUILD_COMMAND   cd <BINARY_DIR> && ${SUITESPARSE_INTERNAL_MAKE_CMD} library CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER}
       INSTALL_COMMAND cd <BINARY_DIR> && ${SUITESPARSE_INTERNAL_MAKE_CMD} install library INSTALL=<INSTALL_DIR> CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER}
       DEPENDS ${LAPACK_TARGET} mpfr
       )
set(SUITESPARSE_CMAKE_FLAGS ${LAPACK_CMAKE_FLAGS} -DSUITESPARSE_INCLUDE_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/include -DSUITESPARSE_LIBRARY_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Add ceres-solver: A Nonlinear Least Squares Minimizer
set(CERES_TARGET ceres)
ExternalProject_Add(${CERES_TARGET}
       GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver
       GIT_TAG 31008453fe979f947e594df15a7e254d6631881b  # 2021/10/06
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver
       BINARY_DIR ${BUILD_DIR}/ceres_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
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

# Add OpenEXR
set(OPENEXR_TARGET openexr)
ExternalProject_Add(${OPENEXR_TARGET}
      # vfxplatform CY2022: 3.1.x
      URL https://github.com/AcademySoftwareFoundation/openexr/archive/v3.1.2.tar.gz
      URL_HASH MD5=2e976f5fb610abb6f75d8f20014cf72a
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
set(ILMBASE_CMAKE_FLAGS -DILMBASE_ROOT=${CMAKE_INSTALL_PREFIX} -DILMBASE_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include/OpenEXR)
set(OPENEXR_CMAKE_FLAGS ${ILMBASE_CMAKE_FLAGS} -DOPENEXR_ROOT=${CMAKE_INSTALL_PREFIX} -DOPENEXR_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include)

# Add LibTiff
if(AV_BUILD_TIFF)
set(TIFF_TARGET tiff)
ExternalProject_Add(${TIFF_TARGET}
       URL http://download.osgeo.org/libtiff/tiff-4.3.0.tar.gz
       URL_HASH MD5=0a2e4744d1426a8fc8211c0cdbc3a1b3
       DOWNLOAD_DIR ${BUILD_DIR}/download/tiff
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/tiff
       BINARY_DIR ${BUILD_DIR}/tiff_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR>
       BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
       DEPENDS ${ZLIB_TARGET}
       )
SET(TIFF_CMAKE_FLAGS -DTIFF_LIBRARY=${CMAKE_INSTALL_PREFIX}/lib/libtiff${CMAKE_SHARED_LIBRARY_SUFFIX} -DTIFF_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include)
endif()

# Add LibPng
if(AV_BUILD_PNG)
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm") 
        set(AV_PNG_ARM_NEON on)
    else()
        set(AV_PNG_ARM_NEON off)
    endif()
    set(PNG_TARGET png)
    ExternalProject_Add(${PNG_TARGET}
           URL https://download.sourceforge.net/libpng/libpng-1.6.37.tar.gz
           URL_HASH MD5=6c7519f6c75939efa0ed3053197abd54
           DOWNLOAD_DIR ${BUILD_DIR}/download/libpng
           PREFIX ${BUILD_DIR}
           BUILD_IN_SOURCE 0
           BUILD_ALWAYS 0
           UPDATE_COMMAND ""
           SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/png
           BINARY_DIR ${BUILD_DIR}/png_build
           INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
           # CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR>
           CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
               ${ZLIB_CMAKE_FLAGS}
               -DPNG_ARM_NEON=${AV_PNG_ARM_NEON}
               -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
               <SOURCE_DIR>
           BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
           DEPENDS ${ZLIB_TARGET}
           )
    SET(PNG_CMAKE_FLAGS -DPNG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libpng${CMAKE_SHARED_LIBRARY_SUFFIX} -DPNG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include)
endif()

if(AV_BUILD_JPEG)
set(JPEG_TARGET turbojpeg)
# Add turbojpeg
ExternalProject_Add(${JPEG_TARGET}
       URL https://github.com/libjpeg-turbo/libjpeg-turbo/archive/2.1.1.tar.gz
       URL_HASH MD5=167d52e2348b6f1af33c70f63197edd8
       DOWNLOAD_DIR ${BUILD_DIR}/download/libjpeg-turbo
       PREFIX ${BUILD_DIR}
       BUILD_IN_SOURCE 0
       BUILD_ALWAYS 0
       UPDATE_COMMAND ""
       SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg
       BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/turbojpeg_build
       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
       # CONFIGURE_COMMAND cd <BINARY_DIR> && autoreconf -fiv && ./configure --prefix=<INSTALL_DIR>
       CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
           ${ZLIB_CMAKE_FLAGS}
           -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
           <SOURCE_DIR>
       BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
       DEPENDS ${ZLIB_TARGET}
       )
   SET(JPEG_CMAKE_FLAGS -DJPEG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libjpeg${CMAKE_SHARED_LIBRARY_SUFFIX} -DJPEG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include)
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
      #URL https://github.com/LibRaw/LibRaw/archive/0.20.0.tar.gz
      GIT_REPOSITORY https://github.com/LibRaw/LibRaw
      GIT_TAG 9c861fd72f3961167ef55b037d7ce16056dd32d8 # specific commit on the 2021.09.16
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/libraw
      BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/libraw
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      # Native libraw configure script doesn't work on centos 7 (autoconf 2.69)
      # CONFIGURE_COMMAND autoconf && ./configure --enable-jpeg --enable-openmp --disable-examples --prefix=<INSTALL_DIR>
      # Use cmake build system (not maintained by libraw devs)
      CONFIGURE_COMMAND cp <SOURCE_DIR>_cmake/CMakeLists.txt . &&
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
SET(LIBRAW_CMAKE_FLAGS -DLIBRAW_PATH=${CMAKE_INSTALL_PREFIX} -DPC_LIBRAW_INCLUDEDIR=${CMAKE_INSTALL_PREFIX}/include -DPC_LIBRAW_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib -DPC_LIBRAW_R_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Add Boost
set(BOOST_TARGET boost)
# vfxplatform CY2021: 1.73
# vfxplatform CY2022: 1.76
ExternalProject_Add(${BOOST_TARGET}
      #URL https://github.com/alicevision/AliceVisionDependencies/releases/download/boost-src-1.80.0/boost_1_80_0.tar.bz2
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
      CONFIGURE_COMMAND cd <SOURCE_DIR> && ./bootstrap.${SCRIPT_EXTENSION} --prefix=<INSTALL_DIR> --with-libraries=atomic,container,date_time,exception,filesystem,graph,iostreams,json,log,math,program_options,regex,serialization,system,test,thread,stacktrace,timer
      BUILD_COMMAND cd <SOURCE_DIR> && ./b2 --prefix=<INSTALL_DIR> variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE} link=shared threading=multi -j8
      INSTALL_COMMAND cd <SOURCE_DIR> && ./b2 variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE} link=shared threading=multi install
      DEPENDS ${ZLIB_TARGET}
      )
set(BOOST_CMAKE_FLAGS -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX})

if(AV_BUILD_FFMPEG)

# GPL
# ExternalProject_add(x264
#       GIT_REPOSITORY https://code.videolan.org/videolan/x264.git
#       GIT_TAG 5493be84
#       GIT_PROGRESS ON
#       PREFIX ${BUILD_DIR}
#       BUILD_IN_SOURCE 0
#       BUILD_ALWAYS 0
#       UPDATE_COMMAND ""
#       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
#       CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-static --enable-shared
#       BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
# )
# GPL
# ExternalProject_add(x265
#       GIT_REPOSITORY https://code.videolan.org/videolan/x265.git
#       GIT_TAG 3.1.1
#       GIT_PROGRESS ON
#       PREFIX ${BUILD_DIR}
#       BUILD_IN_SOURCE 0
#       BUILD_ALWAYS 0
#       UPDATE_COMMAND ""
#       UPDATE_COMMAND echo add_subdirectory(source) > CMakeLists.txt
#       INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
#       CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
#                  -DCMAKE_PROJECT_NAME=x265 -DENABLE_SHARED:bool=ON -DENABLE_PIC:bool=ON
#       BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
# )
ExternalProject_add(libvpx
      GIT_REPOSITORY https://chromium.googlesource.com/webm/libvpx.git
      GIT_TAG v1.10.0
      GIT_PROGRESS ON
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR>
            --enable-shared --disable-static --disable-examples
      BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
)

set(FFMPEG_TARGET ffmpeg)
ExternalProject_add(${FFMPEG_TARGET}
      URL http://ffmpeg.org/releases/ffmpeg-4.4.tar.bz2
      URL_HASH MD5=aa46d2168f30becd4fd3b22d2e8a91ec
      DOWNLOAD_DIR ${BUILD_DIR}/download/ffmpeg
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR>
            --extra-cflags="-I<INSTALL_DIR>/include"
            --extra-ldflags="-L<INSTALL_DIR>/lib"
            --enable-shared
            --disable-static
            --disable-gpl
            --enable-nonfree
            # --enable-libfreetype
            # --enable-libfdk-aac # audio
            # --enable-libmp3lame # audio
            # --enable-libopus # audio
            # --enable-libvorbis # audio
            --enable-libvpx
            # --enable-libx264 # gpl
            # --enable-libx265 # gpl
      BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
      DEPENDS libvpx
)
endif()

# Add OpenImageIO
set(OPENIMAGEIO_TARGET openimageio)
ExternalProject_Add(${OPENIMAGEIO_TARGET}
      URL https://github.com/OpenImageIO/oiio/archive/refs/tags/v2.3.7.2.tar.gz
      URL_HASH MD5=3a6fe4e0b45e611dad378f99b44a696b
      DOWNLOAD_DIR ${BUILD_DIR}/download/oiio
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openimageio
      BINARY_DIR ${BUILD_DIR}/openimageio_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
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
# TODO: openjpeg
# -DOPENJPEG_INCLUDE_DIR=$OPENJPEG_INCLUDE_DIR/openjpeg-2.0 -DOPENJPEG_OPENJP2_LIBRARIES=$OPENJPEG_OPENJP2_LIBRARIES
set(OPENIMAGEIO_CMAKE_FLAGS -DOpenImageIO_DIR=${CMAKE_INSTALL_PREFIX})

# Add Alembic: I/O for Point Cloud and Cameras
set(ALEMBIC_TARGET alembic)
ExternalProject_Add(${ALEMBIC_TARGET}
      # vfxplatform CY2022 1.8.x
      URL https://github.com/alembic/alembic/archive/1.8.3.tar.gz
      URL_HASH MD5=2cd8d6e5a3ac4a014e24a4b04f4fadf9
      DOWNLOAD_DIR ${BUILD_DIR}/download/alembic
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/alembic
      BINARY_DIR ${BUILD_DIR}/alembic_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
          ${ZLIB_CMAKE_FLAGS}
          ${ILMBASE_CMAKE_FLAGS}
          -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
          <SOURCE_DIR>
      BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
      DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${ZLIB_TARGET}
      )
set(ALEMBIC_CMAKE_FLAGS -DAlembic_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/Alembic)

if(AV_BUILD_OPENCV)
set(OPENCV_TARGET opencv)
ExternalProject_Add(opencv_contrib
  URL https://github.com/opencv/opencv_contrib/archive/4.5.3.zip
  URL_MD5 bdf41b61fbb998e118ffd2e81437d44f
  DOWNLOAD_DIR ${BUILD_DIR}/download/opencv_contrib
  SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv_contrib
  BUILD_ALWAYS 0
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)
ExternalProject_Add(${OPENCV_TARGET}
  DEPENDS opencv_contrib ${TBB_TARGET} ${ZLIB_TARGET} ${OPENEXR_TARGET} ${TIFF_TARGET} ${PNG_TARGET} ${JPEG_TARGET} ${LIBRAW_TARGET} ${FFMPEG_TARGET}
  URL https://github.com/opencv/opencv/archive/4.5.3.zip
  URL_MD5 e4e25de494ba3dfb676c9f51d7c853fb
  DOWNLOAD_DIR ${BUILD_DIR}/download/opencv
  UPDATE_COMMAND ""
  BUILD_IN_SOURCE 0
  BUILD_ALWAYS 0
  SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv
  BINARY_DIR ${BUILD_DIR}/opencv_build
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
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
    -DWITH_OPENEXR=OFF  # Build error on IlmBase includes without "OpenEXR/" prefix
    -DENABLE_PRECOMPILED_HEADERS=OFF
    -DBUILD_SHARED_LIBS=ON
    -DWITH_CUDA=OFF
    -DWITH_OPENCL=OFF
    -DBUILD_TESTS=OFF
    -DBUILD_LIST=core,improc,photo,objdetect,video,imgcodecs,videoio,features2d,xfeatures2d,version,mcc,optflow
    -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    <SOURCE_DIR>
  BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
)
# set(OPENCV_CMAKE_FLAGS -DOpenCV_DIR=${BUILD_DIR}/opencv_install -DCMAKE_PREFIX_PATH=${BUILD_DIR}/opencv_install)
set(OPENCV_CMAKE_FLAGS -DOpenCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4 -DOPENCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4)
endif()

# Add CCTag
if(AV_BUILD_CCTAG)
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
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
          ${BOOST_CMAKE_FLAGS}
          ${CUDA_CMAKE_FLAGS}
          ${OPENCV_CMAKE_FLAGS}
          ${EIGEN_CMAKE_FLAGS}
          ${TBB_CMAKE_FLAGS}
          -DCCTAG_WITH_CUDA:BOOL=${AV_USE_CUDA}
          -DCCTAG_BUILD_TESTS=OFF
          -DCCTAG_BUILD_APPS=OFF
          -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
          <SOURCE_DIR>
      BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
      DEPENDS ${BOOST_TARGET} ${CUDA_TARGET} ${OPENCV_TARGET} ${EIGEN_TARGET} ${TBB_TARGET}
      )
set(CCTAG_CMAKE_FLAGS -DCCTag_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/CCTag)
endif()

# Add AprilTag
if(AV_BUILD_APRILTAG)
set(APRILTAG_TARGET apriltag)
ExternalProject_Add(${APRILTAG_TARGET}
      # GIT_REPOSITORY https://github.com/AprilRobotics/apriltag
      # GIT_TAG v3.1.7
      GIT_REPOSITORY https://github.com/alicevision/apriltag.git
      GIT_TAG 25ccfab4230a35fb000814c9c1749d9902173012  # commit 2021/10/26
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/apriltag
      BINARY_DIR ${BUILD_DIR}/apriltag_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
          -DBUILD_PYTHON_WRAPPER=OFF
          -DOpenCV_FOUND=OFF
          -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
          <SOURCE_DIR>
      BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
      )
set(APRILTAG_CMAKE_FLAGS -Dapriltag_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/apriltag/cmake)
endif()

# Add PopSift
if(AV_BUILD_POPSIFT)
set(POPSIFT_TARGET popsift)
ExternalProject_Add(${POPSIFT_TARGET}
      GIT_REPOSITORY https://github.com/alicevision/popsift
      GIT_TAG 4c22d41579c17d7326938929c00c54cfa01a4592
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/popsift
      BINARY_DIR ${BUILD_DIR}/popsift_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
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

# Add Expat: XML parser
set(EXPAT_TARGET expat)
ExternalProject_Add(${EXPAT_TARGET}
      # vfxplatform CY2022 1.8.x
      URL https://github.com/libexpat/libexpat/releases/tag/R_2_4_8/expat-2.4.8.tar.gz
      URL_HASH MD5=CE5FA3FA4D866D83AB0CFB00BB95B77A
      DOWNLOAD_DIR ${BUILD_DIR}/download/expat
      PREFIX ${BUILD_DIR}
      BUILD_IN_SOURCE 0
      BUILD_ALWAYS 0
      UPDATE_COMMAND ""
      SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/expat
      BINARY_DIR ${BUILD_DIR}/expat_build
      INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS} -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
      BUILD_COMMAND $(MAKE)
      #DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${ZLIB_TARGET}
      )
set(EXPAT_CMAKE_FLAGS -DExpat_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/Expat)

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
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
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
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
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
      CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
          -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
          <SOURCE_DIR>
      BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
      DEPENDS ${COINUTILS_TARGET} ${OSI_TARGET}
      )
set(CLP_CMAKE_FLAGS -DClp_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/clp)
endif()

# Add PCL
if(AV_BUILD_PCL)
    # build other dependencies for pcl
    # add lz4 for flann
    set(LZ4_TARGET lz4)
    ExternalProject_Add(${LZ4_TARGET}
            GIT_REPOSITORY https://github.com/alicevision/lz4
            GIT_TAG 47a530e8a8c945ad3bb809e2ce4591a56c5081e2
            PREFIX ${BUILD_DIR}
            BUILD_IN_SOURCE 0
            BUILD_ALWAYS 0
            UPDATE_COMMAND ""
            SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/${LZ4_TARGET}
            BINARY_DIR ${BUILD_DIR}/${LZ4_TARGET}_build
            INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
            CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
                -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
                <SOURCE_DIR>
            BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
            )
    set(LZ4_CMAKE_FLAGS -Dlz4_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/lz4/)

    # @TODO whenever flann will be removed as embedded dependency move this and lz4 out from if(AV_BUILD_PCL) as they will be needed by the library
    # add flann
    set(FLANN_TARGET flann)
    ExternalProject_Add(${FLANN_TARGET}
            GIT_REPOSITORY https://github.com/alicevision/flann
            GIT_TAG 9e79ca6881b779fb842e83c17aef1f4c816ce058
            PREFIX ${BUILD_DIR}
            BUILD_IN_SOURCE 0
            BUILD_ALWAYS 0
            UPDATE_COMMAND ""
            SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/${FLANN_TARGET}
            BINARY_DIR ${BUILD_DIR}/${FLANN_TARGET}_build
            INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
            CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
                -DBUILD_C_BINDINGS:BOOL=OFF
                -DBUILD_EXAMPLES=OFF
                -DBUILD_TESTS:BOOL=OFF
                -DBUILD_DOC:BOOL=OFF
                -DBUILD_PYTHON_BINDINGS:BOOL=OFF
                -DBUILD_MATLAB_BINDINGS:BOOL=OFF
                ${LZ4_CMAKE_FLAGS}
                -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
            BUILD_COMMAND $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
            DEPENDS ${LZ4_TARGET}
            )
    set(FLANN_CMAKE_FLAGS -Dflann_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/flann/)

    # add point cloud library
    set(PCL_TARGET pcl)
    ExternalProject_Add(${PCL_TARGET}
            URL https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.12.1.tar.gz
            URL_MD5 eb1be79330872f9674c9850bb35023bf
            DOWNLOAD_DIR ${BUILD_DIR}/download/${PCL_TARGET}
            PREFIX ${BUILD_DIR}
            BUILD_IN_SOURCE 0
            BUILD_ALWAYS 0
            UPDATE_COMMAND ""
            SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PCL_TARGET}
            BINARY_DIR ${BUILD_DIR}/${PCL_TARGET}_build
            INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
            CONFIGURE_COMMAND ${CMAKE_COMMAND} ${CMAKE_CORE_BUILD_FLAGS}
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

set(AV_DEPS
  ${ZLIB_TARGET}
  ${ASSIMP_TARGET}
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
  ${APRILTAG_TARGET}
  ${POPSIFT_TARGET}
  ${EXPAT_TARGET}
  ${COINUTILS_TARGET}
  ${OSI_TARGET}
  ${CLP_TARGET}
)

if(AV_BUILD_ALICEVISION)
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
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> <SOURCE_DIR>
       DEPENDS ${AV_DEPS}
       )
endif()

else()

# ==============================================================================
# Standard cmake project without internal support for dependencies
# ==============================================================================

# Add sources
add_subdirectory(src)

install(
        FILES LICENSE-MPL2.md LICENSE-MIT-libmv.md COPYING.md CONTRIBUTORS.md
        DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision
)

endif()

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
