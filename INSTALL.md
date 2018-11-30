AliceVision
===========

Build instructions
------------------

Required tools:
* CMake >= 3.4 
* Git
* C/C++ compiler (gcc or visual studio or clang) with C++11 support.

### Compile the project

Getting the sources:

```bash
git clone https://github.com/alicevision/AliceVision.git --recursive
```

As AliceVision use some C++11 features you must have a c++11 ready compiler:
- Visual studio >= 2015
- GCC >= 4.7
- Clang >= 3.3

Dependencies
------------

AliceVision depends on:

* Boost >= 1.60.0
* Eigen >= 3.3.4
* Ceres >= 1.10.0
* Flann >= 1.8.4 (internal)
* CoinUtils >= 2.9.3 (internal)
* Coin-or linear programming (Clp) (internal)
* Open Solver Interface (Osi) >= 0.106.10 (internal)
* Lemon >= 1.3 (internal)
* OpenEXR >= 2.2.0
* OpenImageIO >= 1.8.7
* Geogram >= 1.5.4 (https://gforge.inria.fr/frs/?group_id=5833)
* OpenEXR >= 2.2
* MeshSDFilter (internal)
* OpenMesh (internal)
* zlib

Other optional libraries can enable specific features (check "CMake Options" for enabling them):

* OpenMP (enable multi-threading)
* Mosek 5 (linear programming)
* OpenCV >= 3.2 (feature extraction, calibration module, video IO)
* Alembic (data I/O)
* CCTag (feature extraction/matching and localization on CPU or GPU)
* PopSift (feature extraction on GPU)
* UncertaintyTE (Uncertainty computation)
* Magma (required for UncertaintyTE)
* Cuda >= 7.0 (feature extraction and depth map computation)
* OpenGV (rig calibration and localization)

Building the project using vcpkg (recommended on Windows)
--------------------------------
[Vcpkg](https://github.com/Microsoft/vcpkg) is a tool that helps in acquiring, building, and managing C/C++ libraries.
AliceVision's required dependencies can be built with it. Follow the [installation guide](https://github.com/Microsoft/vcpkg/blob/master/README.md#quick-start) to setup vcpkg.

**Note**: while started as a Windows only project, vcpkg recently became cross-platform. In the scope of AliceVision, it has only been tested on Windows.

To build AliceVision using vcpkg:
1. Setup the environment

#### Windows: CUDA and Visual Studio
* CUDA >= 10.0  
CUDA 10.0 is fully compatible with **Visual Studio 2017 (Update 8)** which allows to use 
the most recent compiler toolset (v141) to build AliceVision and its dependencies. 
VS2015 can also be used in this configuration.  
*This requires a recent version of CMake (tested with 3.12).*

* CUDA < 10.0  
Due to incompatibilities between CUDA and Visual Studio 2017, **Visual 2015 toolset** must be used to build AliceVision.
Visual Studio 2017 can be used as an IDE, but v140 toolset must be installed and used for this to work. 
To setup a VS2017 command prompt using the v140 toolset:
```bash
# Windows: CUDA < 10.0 + VS2017 v140 toolset 
"C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 -vcvars_ver=14.0
nmake /version
# should print Version 14.00.xxx
```
2. Build the required dependencies
```bash
cd <VCPKG_INSTALL_DIR>
set VCPKG_ROOT=%cd%

vcpkg install ^
          boost-algorithm boost-accumulators boost-atomic boost-container boost-date-time boost-exception boost-filesystem boost-graph boost-log ^
          boost-program-options boost-property-tree boost-ptr-container boost-regex boost-serialization boost-system boost-test boost-thread ^
          openexr ^
          openimageio[libraw] ^
          alembic ^
          geogram ^
          eigen3 ^
          ceres[suitesparse] ^
          cuda ^
          --triplet x64-windows
```
3. Build AliceVision
```bash
# With VCPKG_ROOT being the path to the root of vcpkg installation
cd /path/to/aliceVision/
mkdir build && cd build

# Windows: CUDA >= 10.0 + Visual 2017
cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -G "Visual Studio 15 2017" -A x64 -T host=x64

# Windows: CUDA < 10.0 + Visual 2017 v140 (Visual 2015 toolset)
cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -G "Visual Studio 15 2017" -A x64 -T v140,host=x64

# Windows: CUDA < 10.0 + Visual 2015
cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -G "Visual Studio 14 2015" -A x64 -T v140,host=x64

# Windows: this generates a "aliceVision.sln" solution inside the build folder
```

Building the project with embedded dependencies
-----------------------------------------------

```bash
git clone https://github.com/alicevision/AliceVision.git --recursive
mkdir build && cd build
cmake -DALICEVISION_BUILD_DEPENDENCIES=ON -DCMAKE_INSTALL_PREFIX=$PWD/../install ../AliceVision
make -j10
```

* JPEG
You need `autoreconf`, `libtool` and `nasm` to compile `libturbo-jpeg`.
Else if you have jpeg already install on your OS, you can disable the JPEG build with `-DAV_BUILD_JPEG=OFF`.

* PNG
You need `automake` to compile `libpng`.
Else if you have png already install on your OS, you can disable the PNG build with `-DAV_BUILD_PNG=OFF`.


Building the project using external dependencies
------------------------------------------------

In order to build the library with existing versions of the dependencies (e.g. system installed libraries or user built libraries), and thus reduce the compilation time and favour the modularization, the paths where to find such libraries can be given at cmake command line. In particular:

* For Ceres solver library, `Ceres_DIR` can be passed pointing to where CeresConfig.cmake can be found.
  e.g. `-DCeres_DIR:PATH=/path/to/ceres/install/share/Ceres/`

* For FLANN library, `FLANN_INCLUDE_DIR_HINTS` can be passed pointing to the include directory, e.g.
  `-DFLANN_INCLUDE_DIR_HINTS:PATH=/path/to/flann/1.8.4/include/`

* For Eigen library, `CMAKE_MODULE_PATH` should be passed pointing at the `<EigenInstallDir>/share/cmake/Modules/` directory of the Eigen installation, in which `Eigen-config.cmake` or `FindEigen3.cmake` can be found. 
  In case only `FindEigen3.cmake` is available (e.g. Homebrew installations), an environment variable `EIGEN_ROOT_DIR` must be set pointing at Eigen install directory.
  For example,
  
  `-DCMAKE_MODULE_PATH:PATH=/usr/local/Cellar/eigen/3.3.4/share/cmake/Modules/`

  may require to set the environment variable if only `FindEigen3.cmake`, i.e.
  
  `export EIGEN_ROOT_DIR=/usr/local/Cellar/eigen/3.3.4/`

* For OpenEXR library, `OPENEXR_HOME` can be passed pointing to the install directory, e.g.
  `-DOPENEXR_HOME:PATH=/path/to/openexr/install`

* For OpenImageIO library, library and include dir paths can be passed, e.g.
  `-DOPENIMAGEIO_LIBRARY_DIR_HINTS:PATH=/path/to/oiio/install/lib/`
and `-DOPENIMAGEIO_INCLUDE_DIR:PATH=/path/to/oiio/install/include/`



At the end of the cmake process, a report shows for each library which version (internal/external) will be used in the building process, e.g.:

```
-- EIGEN: 3.3.4
-- CERES: 1.10.0
-- FLANN: 1.8.4 (external)
-- CLP: 1.15.11 (internal)
-- COINUTILS: 2.9.3 (internal)
-- OSI: 0.106.10 (internal)
-- LEMON: 1.3 (internal)
```


CMake Options
-------------

* GEOGRAM
  `-DGEOGRAM_INSTALL_PREFIX:PATH=path/to/geogram/install`

* OPENIMAGEIO
  `-DOPENIMAGEIO_LIBRARY_DIR_HINTS:PATH=/path/to/oiio/install/lib/`
  `-DOPENIMAGEIO_INCLUDE_DIR:PATH=/path/to/oiio/install/include/`

* `BOOST_NO_CXX11` (default `OFF`)
  If your Boost binaries are compiled without C++11 support, you need to set this option to avoid compilation errors.
  This is most likely to be the case if you use the system packages to install boost.

* `ALICEVISION_USE_OPENMP` (default `ON`)
  Use OpenMP parallelization (huge impact on performances)
  **OSX**: if you are compiling with clang shipped with XCode, please note that OpenMP is not supported and you need to
  disable OpenMP passing `-DALICEVISION_USE_OPENMP:BOOL=OFF`.

* `ALICEVISION_USE_CCTAG` (default: `AUTO`)
  Build with CCTag markers support.
  `-DCCTag_DIR:PATH=/path/to/cctag/install/lib/cmake/CCTag` (where CCTagConfig.cmake can be found)

* `ALICEVISION_USE_OPENGV` (default `AUTO`)
  Enable use of OpenGV algorithms. Build with openGV for multi-cameras localization.
  `-DOPENGV_DIR:PATH=/path/to/opengv/install/` (where "include" and "lib" folders can be found)
  We recommend: `git clone https://github.com/alicevision/opengv.git --branch=cmake_fix_install`

* `ALICEVISION_USE_ALEMBIC` (default `AUTO`)
  Build with Alembic file format support (required version >= 1.7).
  `-DAlembic_DIR:PATH=/path/to/alembic/install/lib/cmake/Alembic/` (where AlembicConfig.cmake can be found)
  With old Alembic versions (<1.6), you need to set many variables: `ALEMBIC_ROOT`, `ALEMBIC_HDF5_ROOT`, `ALEMBIC_ILMBASE_ROOT`, `ALEMBIC_OPENEXR_ROOT`.
   
* `ALICEVISION_USE_OPENMP` (default: `AUTO`)
  Enable OpenMP parallelization

* `ALICEVISION_USE_CUDA` (default: `ON`)
  Enable build with CUDA (for feature extraction and depth map computation)
  `-DCUDA_TOOLKIT_ROOT_DIR:PATH=/usr/local/cuda-9.1` (adjust the path to your cuda installation)

* `ALICEVISION_USE_POPSIFT` (default: `AUTO`)
  Enable GPU SIFT implementation.
  `-DPopSift_DIR:PATH=/path/to/popsift/install/lib/cmake/PopSift` (where PopSiftConfig.cmake can be found)
  
* `ALICEVISION_USE_UNCERTAINTYTE` (default: `AUTO`)
  Enable Uncertainty computation.
  `-DUNCERTAINTYTE_DIR:PATH=/path/to/uncertaintyTE/install/` (where `inlude` and `lib` can be found)
  `-DMAGMA_ROOT:PATH=/path/to/magma/install/` (where `inlude` and `lib` can be found)
  
* `ALICEVISION_USE_OPENCV` (default: `OFF`)
  Build with openCV
  `-DOpenCV_DIR:PATH=/path/to/opencv/install/share/OpenCV/` (where OpenCVConfig.cmake can be found)
  
* `ALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE` (default: `ON`)
  By default, aliceVision requires Ceres built with SuiteSparse to ensure best performances but you can make SuiteSparse optional with this flag.

* `ALICEVISION_BUILD_SHARED` (default `OFF`)
  Build AliceVision as shared libs (instead of static libs)

* `ALICEVISION_BUILD_TESTS` (default `OFF`)
  Build AliceVision tests

* `ALICEVISION_BUILD_DOC` (default `AUTO`)
  Build AliceVision documentation

* `ALICEVISION_BUILD_EXAMPLES` (default `ON`)
  Build AliceVision samples applications (aliceVision software are still built)

* `ALICEVISION_BUILD_COVERAGE` (default `OFF`)
  Enable code coverage generation (gcc only)


Linux compilation
-----------------

### Setup the required external library.

* `sudo apt-get install libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev`

* If you want see the view graph svg logs
  `sudo apt-get install graphviz`

### Clone and configure the project:

```bash
 git clone --recursive https://github.com/alicevision/AliceVision.git
 mkdir build && cd build
 cmake -DCMAKE_BUILD_TYPE=Release . ../AliceVision
```

If you want enable unit tests and examples to the build:
```bash
cmake -DCMAKE_BUILD_TYPE=Release -DALICEVISION_BUILD_TESTS=ON -DALICEVISION_BUILD_EXAMPLES=ON ../AliceVision
```

In order to use the MOSEK 6 back-end for the linear programming aliceVision module:

- Check that you have an up-to-date MOSEK licence, else aliceVision MOSEK unit test will fail.

- Then:

  ```bash
  cmake -DCMAKE_BUILD_TYPE=Release \
        -DMOSEK_SEARCH_HEADER="~/Documents/Lib/mosek/6/tools/platform/linux64x86/h" \
        -DMOSEK_SEARCH_LIB="~/Documents/Lib/mosek/6/tools/platform/linux64x86/bin" \
        ../AliceVision
  ```

If you want to have an IDE openable project with codeblocks:

```bash
cmake -G "CodeBlocks - Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ../AliceVision
```

### Compile the project

```bash
make
```

For a multi-core compilation (Replace NBcore with the number of threads)
```bash
make -j NBcore
```

Launch unity tests (if asked at cmake step)
```bash
make test
```


Windows compilation
-------------------

* Checkout the project
  `git clone --recursive https://github.com/alicevision/aliceVision.git`
* Open cmake-gui
  * Fill the source path with the AliceVision path.
  * Fill the build path with a new directory
  * Select your Visual Studio IDE and click configure and then generate
* Open the .sln solution created in your build directory.
  * Change the target to Release.
  * Compile the libraries and binaries samples.

-------------------


Mac OSX compilation
-------------------
```bash
git clone --recursive https://github.com/alicevision/AliceVision.git
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "Xcode" ../AliceVision
```
If you want enable unit tests and examples to the build:
```bash
cmake -DCMAKE_BUILD_TYPE=Release \
      -DALICEVISION_BUILD_TESTS=ON \
      -DALICEVISION_BUILD_EXAMPLES=ON \
      -G "Xcode" \
      ../AliceVision
xcodebuild -configuration Release
```
--------------------


Using AliceVision as a third party library dependency in cmake
--------------------------------------------------------------

AliceVision can be used as a third party once it have been installed.
Because it can use its own Ceres version, it is better to install it locally and not in system files.
So please consider using the `CMAKE_INSTALL_PREFIX` cmake variable to specify a local installation directory.

Here the syntax to add the variable to the cmake command line (use absolute path), e.g.: 
```bash
-DCMAKE_INSTALL_PREFIX="/home/user/dev/AliceVision_install"
```

Perform `make` and `make install`

Then you will be able to use AliceVision as an external library in your `CMakeLists.txt`:
```cmake
find_package(AliceVision REQUIRED)
include_directories(${ALICEVISION_INCLUDE_DIRS})
add_executable(main main.cpp)
target_link_libraries(main ${ALICEVISION_LIBRARIES})
```

Specify to CMake where AliceVision is installed by using the `AliceVision_DIR` cmake variable: `-DAliceVision_DIR:STRING="YourInstallPath"/share/aliceVision/cmake`


### Docker image

A docker image can be built using the CentOS 7 or Ubuntu 18 Dockerfiles.
The Dockerfiles are based on `nvidia/cuda` images (https://hub.docker.com/r/nvidia/cuda/)

```
docker build --tag alicevision:centos7-cuda7.0 .
docker build --tag alicevision:ubuntu18.04-cuda9.2 -f Dockerfile_ubuntu .
```

Parameters `OS_TAG` and `CUDA_TAG` can be passed to build the image with a specific OS and CUDA version.
Use NPROC=8 to select the number of cores to use, 1 by default.
For example, in order to create a CentOS 7 with Cuda 9.2, use:

```
docker build --build-arg OS_TAG=7 CUDA_TAG=9.2 --tag alicevision:centos7-cuda9.2 .
docker build --build-arg OS_TAG=16.04 CUDA_TAG=8.0 NPROC=8 --tag alicevision:ubuntu16.04-cuda8.0 -f Dockerfile_ubuntu .
```

In order to run the image [nvidia docker](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)) is needed.

```
docker run -it --runtime=nvidia alicevision:centos7-cuda9.2
```

To retrieve the generated files:

```
# Create an instance of the image, copy the files and remove the temporary docker instance.
CID=$(docker create alicevision:centos7-cuda9.2) && docker cp ${CID}:/opt/AliceVision_install . && docker cp ${CID}:/opt/AliceVision_bundle . && docker rm ${CID}
```

