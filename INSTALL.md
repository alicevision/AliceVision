AliceVision
===========

Build instructions
------------------

Required tools:
* CMake >= 3.11
* Git
* C/C++ compiler (gcc or visual studio or clang) with C++17 support (i.e. gcc >= 7, clang >= 5, msvc >= 19.15, cuda >= 11.0).

### Compile the project

Getting the sources:

```bash
git clone https://github.com/alicevision/AliceVision.git --recursive
```

Dependencies
------------

AliceVision depends on external libraries:

* [Assimp >= 5.0.0](https://github.com/assimp/assimp)
* [Boost >= 1.74.0](https://www.boost.org)
* [Ceres >= 1.10.0](https://github.com/ceres-solver/ceres-solver)
* CoinUtils >= 2.9.3 use [our fork](https://github.com/alicevision/CoinUtils) with a CMake build system so that it can be easily found
* Coin-or linear programming (Clp) use [our fork](https://github.com/alicevision/Clp) with a CMake build system
* [Eigen >= 3.3.4](https://gitlab.com/libeigen/eigen)
* [Expat >= 2.4.8](https://libexpat.github.io/)
* Flann >= 1.8.4, use [our fork](https://github.com/alicevision/flann) with a CMake build system
* [Geogram >= 1.7.5](https://github.com/BrunoLevy/geogram)
* [OpenEXR >= 2.5](https://github.com/AcademySoftwareFoundation/openexr)
* [OpenImageIO >= 2.1.0 (recommended >= 2.4.13)](https://github.com/OpenImageIO/oiio)
* Open Solver Interface (Osi) >= 0.106.10 use [our fork](https://github.com/alicevision/Osi)) with a CMake build system
* [zlib](https://www.zlib.net)

Other optional libraries can enable specific features (check "CMake Options" for enabling them):

* Alembic (data I/O)
* CCTag (feature extraction/matching and localization on CPU or GPU)
* Cuda >= 11.0 (feature extraction and depth map computation)
* Magma (required for UncertaintyTE)
* Mosek >= 6 (linear programming)
* OpenCV >= 3.4.11 (feature extraction, calibration module, video IO), >= 4.5 for colorchecker (mcc)
* OpenGV (rig calibration and localization)
* OpenMP (enable multi-threading)
* PCL (Point Cloud Library) >= 1.12.1 for the registration module
* PopSift (feature extraction on GPU)
* UncertaintyTE (Uncertainty computation)
* Lemon >= 1.3

AliceVision also depends on some embedded libraries:

* MeshSDFilter (internal)
* OpenMesh (internal)



Building the project using vcpkg (recommended on Windows)
--------------------------------
[Vcpkg](https://github.com/alicevision/vcpkg) is a package manager that helps in acquiring, building, and managing C/C++ libraries.
AliceVision's required dependencies can be built with it.
Vcpkg evolved from being a Windows-only project to becoming cross-platform.
In the scope of AliceVision, vcpkg has only been tested on Windows.

1. Install vcpkg itself

See the reference [installation guide](https://github.com/alicevision/vcpkg/blob/alicevision_master/README.md#quick-start-windows) to setup vcpkg.
We recommend to use our vcpkg fork, where dependencies have been validated by the AliceVision development team but it should work with the latest version.
```bash
git clone https://github.com/alicevision/vcpkg --branch alicevision_master
cd vcpkg
.\bootstrap-vcpkg.bat
```

2. Build the required dependencies
```bash
cd <VCPKG_INSTALL_DIR>
set VCPKG_ROOT=%cd%

vcpkg install ^
          boost-algorithm boost-accumulators boost-atomic boost-container boost-date-time boost-exception boost-geometry boost-graph boost-json boost-log ^
          boost-program-options boost-property-tree boost-ptr-container boost-regex boost-serialization boost-system boost-test boost-thread boost-timer ^
          lz4 ^
          liblemon ^
          openexr ^
          alembic ^
          geogram ^
          eigen3 ^
          expat ^
          flann ^
          onnxruntime-gpu ^
          opencv[eigen,ffmpeg,webp,contrib,nonfree,cuda] ^
          openimageio[libraw,ffmpeg,freetype,opencv,gif,openjpeg,webp] ^
          ceres[suitesparse,cxsparse] ^
          cuda ^
          tbb ^
          assimp ^
          pcl ^
          clp ^
          --triplet x64-windows
```

3. Build AliceVision
```bash
# With VCPKG_ROOT being the path to the root of vcpkg installation
cd /path/to/aliceVision/
mkdir build && cd build

# Windows: Visual 2022 + Powershell
cmake .. -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT"\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -G "Visual Studio 17 2022" -A x64 -T host=x64

# Windows: Visual 2022
cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -G "Visual Studio 17 2022" -A x64 -T host=x64

# Windows: Visual 2017
cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -G "Visual Studio 15 2017" -A x64 -T host=x64
```

This generates a "aliceVision.sln" solution inside the build folder that you can open in Visual Studio to launch the build. Do not forget to switch the build type to "Release".


Building the project with embedded dependencies (recommended on linux)
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
-- FLANN: 1.8.4
-- LEMON: 1.3
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

* `ALICEVISION_USE_APRILTAG` (default: `AUTO`)
  Build with AprilTag markers support.
  `-Dapriltag_DIR:PATH=/path/to/apriltag/install/share/apriltag/cmake` (where apriltagConfig.cmake can be found)

* `ALICEVISION_USE_OPENGV` (default `AUTO`)
  Enable use of OpenGV algorithms. Build with openGV for multi-cameras localization.
  `-DOPENGV_DIR:PATH=/path/to/opengv/install/` (where "include" and "lib" folders can be found)
  We recommend: `git clone https://github.com/alicevision/opengv.git --branch=cmake_fix_install`

* `ALICEVISION_USE_ALEMBIC` (default `AUTO`)
  Build with Alembic file format support (required version >= 1.7).
  `-DAlembic_DIR:PATH=/path/to/alembic/install/lib/cmake/Alembic/` (where AlembicConfig.cmake can be found)
  With old Alembic versions (<1.6), you need to set many variables: `ALEMBIC_ROOT`, `ALEMBIC_HDF5_ROOT`, `ALEMBIC_ILMBASE_ROOT`, `ALEMBIC_OPENEXR_ROOT`.

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

* `BUILD_SHARED_LIBS` (default `ON`)
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
Consider using the `CMAKE_INSTALL_PREFIX` cmake variable to specify a local installation directory.
Here the syntax to add the variable to the cmake command line (use absolute path), e.g.:
```bash
-DCMAKE_INSTALL_PREFIX="/home/user/dev/AliceVision_install"
```

Perform `make` and `make install`

Then you will be able to use AliceVision as an external library in your `CMakeLists.txt` using
the modern CMake approach as imported target. For example, if your target `main` depends on the
AliceVision module `aliceVision_sfmDataIO`:

```cmake
find_package(AliceVision CONFIG REQUIRED)
message(STATUS "Found AliceVision : ${AliceVision_FOUND}")
message(STATUS "Found AliceVision version: ${AliceVision_VERSION}")
add_executable(main main.cpp)
target_link_libraries(main PUBLIC aliceVision_sfmDataIO)
```

In general, you need to specify the list of the AliceVision modules that your library or executable
depends on.

Specify to CMake where AliceVision is installed by using the `AliceVision_DIR` cmake variable: `-DAliceVision_DIR:PATH="YourInstallPath"/share/aliceVision/cmake`
or by simply adding the installation path to your `CMAKE_PREFIX_PATH`, i.e. `-DCMAKE_PREFIX_PATH:PATH="YourInstallPath"`.
Check the sample in [samples](src/samples/aliceVisionAs3rdParty) for an example of use.

### Docker image

A docker image can be built using the CentOS or Ubuntu Dockerfiles.
The Dockerfiles are based on `nvidia/cuda` images (https://hub.docker.com/r/nvidia/cuda/)

To generate the docker image, just run:
```
./docker/build-centos.sh
```

To do it manually, parameters `OS_TAG` and `CUDA_TAG` should be passed to choose the OS and CUDA version.
For example, the first line of below's commands shows the example to create docker for a CentOS 7 with Cuda 11.3.1 and second line for Ubuntu 16.04 with Cuda 11.0:

```
docker build --build-arg OS_TAG=7 --build-arg CUDA_TAG=11.3.1 --tag alicevision:centos7-cuda11.3.1 .
docker build --build-arg OS_TAG=16.04 --build-arg CUDA_TAG=11.0 --build-arg NPROC=8 --tag alicevision:ubuntu16.04-cuda11.0 -f Dockerfile_ubuntu .
```

In order to run the image [nvidia docker](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)) is needed.

```
docker run -it --runtime=nvidia alicevision:centos7-cuda9.2
```

To retrieve the generated files:

```
# Create an instance of the image, copy the files and remove the temporary docker instance.
CID=$(docker create alicevision:centos7-cuda11.3.1) && docker cp ${CID}:/opt/AliceVision_install . && docker cp ${CID}:/opt/AliceVision_bundle . && docker rm ${CID}
```

Environment variable
--------------------

Whatever the way AliceVision has been installed, before using it, an environment variable named ALICEVISION_ROOT must be created and set with the local installation directory. 

