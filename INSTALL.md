# AliceVision

## Build instructions

Required tools:
* CMake >= 3.11
* Git
* C/C++ compiler (gcc or Visual Studio or clang) with C++17 support (i.e. gcc >= 7, clang >= 5, msvc >= 19.15, cuda >= 11.0).

### Compile the project

Getting the sources:

```bash
git clone https://github.com/alicevision/AliceVision.git --recursive
```

## Dependencies

AliceVision depends on external libraries:

* [Assimp >= 5.0.0](https://github.com/assimp/assimp)
* [Boost >= 1.74.0](https://www.boost.org)
* [Ceres >= 1.10.0](https://github.com/ceres-solver/ceres-solver)
* CoinUtils >= 2.9.3; use [our fork](https://github.com/alicevision/CoinUtils) with a CMake build system
* Coin-or linear programming (Clp); use [our fork](https://github.com/alicevision/Clp) with a CMake build system
* [Eigen >= 3.3.4](https://gitlab.com/libeigen/eigen)
* [Expat >= 2.4.8](https://libexpat.github.io/)
* Flann >= 1.8.4; use [our fork](https://github.com/alicevision/flann) with a CMake build system
* [Geogram >= 1.7.5 (recommended >= 1.8.8)](https://github.com/BrunoLevy/geogram)
* [nanoflann >= 1.5.4](https://github.com/jlblancoc/nanoflann)
* [OpenEXR >= 2.5](https://github.com/AcademySoftwareFoundation/openexr)
* [OpenImageIO >= 2.1.0 (recommended >= 2.4.13)](https://github.com/OpenImageIO/oiio)
* [OpenMesh >= 9.0](https://www.graphics.rwth-aachen.de/software/openmesh/)
* Open Solver Interface (Osi) >= 0.106.10; use [our fork](https://github.com/alicevision/Osi) with a CMake build system
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
* libe57format (support reading .e57 files)
* SWIG, Python 3 and NumPy 1.26 (Python binding for AliceVision modules)


AliceVision also depends on some embedded libraries:

* MeshSDFilter (internal)



### Building the project using vcpkg (recommended on Windows)

[vcpkg](https://github.com/alicevision/vcpkg) is a package manager that helps in acquiring, building, and managing C/C++ libraries.
AliceVision's required dependencies can be built with it.
vcpkg evolved from being a Windows-only project to becoming cross-platform.
In the scope of AliceVision, vcpkg has only been tested on Windows.

1. **Install vcpkg**

    See the reference [installation guide](https://github.com/alicevision/vcpkg/blob/alicevision_master/README.md#quick-start-windows) to setup vcpkg.
    We recommend to use our vcpkg fork, where dependencies have been validated by the AliceVision development team and where some ports may have custom changes.
    ```bash
    git clone https://github.com/alicevision/vcpkg --branch alicevision_master
    cd vcpkg
    .\bootstrap-vcpkg.bat
    ```

2. **Build/install the required dependencies**

    There are two options for the dependencies:

    - Build them from scratch on your system.
        ```bash
        cd <VCPKG_INSTALL_DIR>
        set VCPKG_ROOT=%cd%

        vcpkg install ^
                  boost-algorithm boost-accumulators boost-atomic boost-container boost-date-time boost-exception boost-format ^
                  boost-geometry boost-graph boost-json boost-log boost-program-options boost-property-tree ^
                  boost-ptr-container boost-regex boost-serialization boost-system boost-test boost-thread boost-timer ^
                  lz4 ^
                  liblemon ^
                  openexr ^
                  alembic ^
                  geogram ^
                  eigen3 ^
                  expat ^
                  flann nanoflann ^
                  onnxruntime-gpu ^
                  opencv[eigen,ffmpeg,webp,contrib,nonfree,cuda,python] ^
                  openimageio[opencolorio,pybind11,libraw,ffmpeg,freetype,opencv,gif,openjpeg,webp] ^
                  openmesh ^
                  ceres[suitesparse] ^
                  cuda ^
                  tbb ^
                  assimp ^
                  pcl ^
                  clp ^
                  libe57format ^
                  vcpkg-tool-swig ^
                  cctag ^
                  popsift ^
                  --triplet x64-windows-release
        ```

     - Install them from a ready-to-use precompiled archive.

         This will save all the compilation time as well as some disk space as there will not be any build artifact, but this requires to use the same CUDA version as the one that was used to generate the archive to be able to build AliceVision later on.

         The archive can be downloaded from [our vcpkg fork's release page](https://github.com/alicevision/vcpkg/releases), with the [latest released archive](https://github.com/alicevision/vcpkg/releases/download/2025.07.08/aliceVisionDeps-2025.07.08.zip) built with CUDA 12.5.0.

         It should be unzipped in `<VCPKG_INSTALL_DIRECTORY>`.

3. **Build AliceVision**
    ```bash
    # With VCPKG_ROOT being the path to the root of vcpkg installation
    cd /path/to/aliceVision/
    mkdir build && cd build

    # Windows: Visual 2022 + Powershell
    cmake .. -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT"\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows-release -G "Visual Studio 17 2022" -A x64 -T host=x64

    # Windows: Visual 2022
    cmake .. -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows-release -G "Visual Studio 17 2022" -A x64 -T host=x64
    ```

    This generates a "aliceVision.sln" solution inside the build folder that you can open in Visual Studio to launch the build. Do not forget to switch the build type to "Release".


### Building the project with embedded dependencies (recommended on Linux)

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


### Building the project using external dependencies

In order to build the library with existing versions of the dependencies (e.g. system-installed libraries or user-built libraries), and thus reduce the compilation time and favour the modularization, the paths where to find such libraries can be given at cmake command line. In particular:

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


## CMake Options

* GEOGRAM  
  `-DGEOGRAM_INSTALL_PREFIX:PATH=path/to/geogram/install`

* OPENIMAGEIO  
  `-DOPENIMAGEIO_LIBRARY_DIR_HINTS:PATH=/path/to/oiio/install/lib/`
  `-DOPENIMAGEIO_INCLUDE_DIR:PATH=/path/to/oiio/install/include/`

* `BOOST_NO_CXX11` (default `OFF`)  
  If your Boost binaries are compiled without C++11 support, you need to set this option to avoid compilation errors.
  This is most likely to be the case if you use the system packages to install boost.

* `ALICEVISION_USE_OPENMP` (default `ON`)  
  Use OpenMP parallelization (huge impact on performances).  
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
  `-DOPENGV_DIR:PATH=/path/to/opengv/install/` (where the `include` and `lib` folders can be found)  
  We recommend: `git clone https://github.com/alicevision/opengv.git --branch=cmake_fix_install`

* `ALICEVISION_USE_ALEMBIC` (default `AUTO`)  
  Build with Alembic file format support (required version >= 1.7).  
  `-DAlembic_DIR:PATH=/path/to/alembic/install/lib/cmake/Alembic/` (where AlembicConfig.cmake can be found)  
  With old Alembic versions (<1.6), you need to set many variables: `ALEMBIC_ROOT`, `ALEMBIC_HDF5_ROOT`, `ALEMBIC_ILMBASE_ROOT`, `ALEMBIC_OPENEXR_ROOT`.

* `ALICEVISION_USE_CUDA` (default: `ON`)  
  Enable build with CUDA (for feature extraction and depth map computation).  
  `-DCUDA_TOOLKIT_ROOT_DIR:PATH=/usr/local/cuda-9.1` (adjust the path to your CUDA installation)

* `ALICEVISION_USE_POPSIFT` (default: `AUTO`)  
  Enable GPU SIFT implementation.  
  `-DPopSift_DIR:PATH=/path/to/popsift/install/lib/cmake/PopSift` (where PopSiftConfig.cmake can be found)

* `ALICEVISION_USE_UNCERTAINTYTE` (default: `AUTO`)  
  Enable Uncertainty computation.  
  `-DUNCERTAINTYTE_DIR:PATH=/path/to/uncertaintyTE/install/` (where the `include` and `lib` folders can be found)
  `-DMAGMA_ROOT:PATH=/path/to/magma/install/` (where the `include` and `lib` folders can be found)

* `ALICEVISION_USE_OPENCV` (default: `OFF`)  
  Build with OpenCV.  
  `-DOpenCV_DIR:PATH=/path/to/opencv/install/share/OpenCV/` (where OpenCVConfig.cmake can be found)

* `ALICEVISION_USE_ONNX_GPU` (default: `ON`)  
  Enable the use of CUDA for ONNX. On some Windows systems, this may cause errors and this flag should be set to `OFF`.

* `ALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE` (default: `ON`)  
  By default, aliceVision requires Ceres built with SuiteSparse to ensure best performances but you can make SuiteSparse optional with this flag.

* `BUILD_SHARED_LIBS` (default `ON`)  
  Build AliceVision as shared libraries (instead of static libraries).

* `ALICEVISION_BUILD_SOFTWARE` (default `ON`)  
  Build AliceVision command line tools.

* `ALICEVISION_BUILD_TESTS` (default `OFF`)  
  Build AliceVision unit tests.

* `ALICEVISION_BUILD_DOC` (default `AUTO`)  
  Build AliceVision documentation.

* `ALICEVISION_BUILD_COVERAGE` (default `OFF`)  
  Enable code coverage generation (gcc only).

* `ALICEVISION_BUILD_SWIG_BINDING` (default `OFF`)  
  Build AliceVision's Python binding with SWIG.  
  AliceVision's Python binding requires Python to be installed on the system, as well as NumPy.
  For a better compatibility with Meshroom, we advise to use the same Python version as the one used to run Meshroom as well as NumPy 1.26.

* `ALICEVISION_INSTALL_MESHROOM_PLUGIN` (default `ON`)  
  Copy Meshroom nodes and templates in the installation directory.


### CMake options to build specific parts of AliceVision

* `ALICEVISION_BUILD_SFM` (default `ON`)  
  Build the SfM part of AliceVision. If set to `OFF`, all the following options will be disabled: 
  `ALICEVISION_BUILD_MVS`, `ALICEVISION_BUILD_HDR`, `ALICEVISION_BUILD_SEGMENTATION`, `ALICEVISION_BUILD_PHOTOMETRICSTEREO`, 
  `ALICEVISION_BUILD_PANORAMA`, `ALICEVISION_BUILD_LIDAR`.

* `ALICEVISION_BUILD_MVS` (default `ON`)  
  Build the MVS part of AliceVision.

* `ALICEVISION_BUILD_HDR` (default `ON`)  
  Build the HDR part of AliceVision.

* `ALICEVISION_BUILD_SEGMENTATION` (default `ON`)  
  Build the ONNX-based segmentation part of AliceVision.

* `ALICEVISION_BUILD_PHOTOMETRICSTEREO` (default `ON`)  
  Build the Photometric Stereo part of AliceVision.

* `ALICEVISION_BUILD_PANORAMA` (default `ON`)  
  Build the Panorama part of AliceVision.

* `ALICEVISION_BUILD_LIDAR` (default `AUTO`)  
  Build the LiDAR part of AliceVision.


## Compilation

### Linux compilation

#### Setup the required external library.

* `sudo apt-get install libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev`

* If you want to be able to see the view graph SVG logs:
  `sudo apt-get install graphviz`

#### Clone and configure the project:

```bash
 git clone --recursive https://github.com/alicevision/AliceVision.git
 mkdir build && cd build
 cmake -DCMAKE_BUILD_TYPE=Release . ../AliceVision
```

If you want to enable the build of the unit tests:
```bash
cmake -DCMAKE_BUILD_TYPE=Release -DALICEVISION_BUILD_TESTS=ON ../AliceVision
```

In order to use the MOSEK 6 back-end for the linear programming aliceVision module:

- Check that you have an up-to-date MOSEK licence, otherwise the aliceVision MOSEK unit test will fail.

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

#### Compile the project

```bash
make
```

For a multi-core compilation (replace `NBcore` with the number of threads):
```bash
make -j NBcore
```

Launch the unit tests (if built during the compilation step):
```bash
make test
```


### Windows compilation

* Checkout the project
  `git clone --recursive https://github.com/alicevision/aliceVision.git`.
* Open cmake-gui.
  * Fill the source path with the AliceVision path.
  * Fill the build path with a new directory.
  * Select your Visual Studio IDE and click configure and then generate.
* Open the .sln solution created in your build directory.
  * Change the target to Release.
  * Compile the libraries and binaries samples.


### Mac OSX compilation

```bash
git clone --recursive https://github.com/alicevision/AliceVision.git
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "Xcode" ../AliceVision
```

If you want to enable the build of the unit tests:
```bash
cmake -DCMAKE_BUILD_TYPE=Release \
      -DALICEVISION_BUILD_TESTS=ON \
      -G "Xcode" \
      ../AliceVision
xcodebuild -configuration Release
```


## Using AliceVision as a third party library dependency in CMake

AliceVision can be used as a third party library once it has been installed.
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

### Docker images

A docker image can be built using the Ubuntu or Rocky Linux Dockerfiles.
The Dockerfiles are based on `nvidia/cuda` images (https://hub.docker.com/r/nvidia/cuda/).

To generate the Docker image, just run:
```
./docker/build-rocky.sh
```
or
```
./docker/build-ubuntu.sh
```

To do it manually, parameters `ROCKY_VERSION`/`UBUNTU_VERSION` and `CUDA_TAG` should be passed to choose the OS and CUDA versions.
For example, the first line of the commands below shows the example to build a Docker for a Rocky 9 with Cuda 12.1.0, while the second line is for Ubuntu 22.04 with Cuda 12.1.0:

```
docker build --build-arg ROCKY_VERSION=9 --build-arg CUDA_TAG=12.1.0 --tag alicevision:rocky9-cuda12.1.0 -f Dockerfile_rocky .
docker build --build-arg UBUNTU_VERSION=22.04 --build-arg CUDA_TAG=12.1.0 --build-arg NPROC=8 --tag alicevision:ubuntu22.04-cuda12.1.0 -f Dockerfile_ubuntu .
```

In order to run the image, [nvidia docker](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)) is needed.

```
docker run -it --runtime=nvidia alicevision:rocky9-cuda12.1.0
```

To retrieve the generated files:

```
# Create an instance of the image, copy the files and remove the temporary docker instance.
CID=$(docker create alicevision:rocky9-cuda12.1.0) && docker cp ${CID}:/opt/AliceVision_install . && docker cp ${CID}:/opt/AliceVision_bundle . && docker rm ${CID}
```

## Environment variable

You must set the `ALICEVISION_ROOT` environment variable to point to your installation directory, regardless of how you installed it.


## Using AliceVision with Meshroom

AliceVision provides nodes and templates meant to be used with [Meshroom](https://github.com/alicevision/Meshroom).

To install the plugin, build AliceVision with `ALICEVISION_INSTALL_MESHROOM_PLUGIN=ON` (enabled by default) and set the `MESHROOM_NODES_PATH` and `MESHROOM_PIPELINE_TEMPLATES_PATH` environment variables for Meshroom to detect it.
- On Windows:
    ```
    set MESHROOM_NODES_PATH=%ALICEVISION_ROOT%/share/meshroom;%MESHROOM_NODES_PATH%
    set MESHROOM_PIPELINE_TEMPLATES_PATH=%ALICEVISION_ROOT%/share/meshroom;%MESHROOM_PIPELINE_TEMPLATES_PATH%
    ```
- On Linux:
    ```
    export MESHROOM_NODES_PATH=$ALICEVISION_ROOT/share/meshroom:$MESHROOM_NODES_PATH
    export MESHROOM_PIPELINE_TEMPLATES_PATH=$ALICEVISION_ROOT/share/meshroom:$MESHROOM_PIPELINE_TEMPLATES_PATH
    ```

To use AliceVision in Meshroom to the best of its abilities, we recommend building with the following flags:
* `ALICEVISION_USE_OPENCV=ON`
* `ALICEVISION_BUILD_SWIG_BINDING=ON`
* `ALICEVISION_USE_POPSIFT=ON`
* `ALICEVISION_USE_CCTAG=ON`
* `ALICEVISION_INSTALL_MESHROOM_PLUGIN=ON`

### Environment variables to set for Meshroom

Meshroom relies on specific files provided by AliceVision:

* Sensor database: a text database of sensor width per camera model.
Provided in AliceVision source tree: {ALICEVISION_REPOSITORY}/src/aliceVision/sensorDB/cameraSensors.db
* Voctree (optional): for larger datasets (>200 images), greatly improves image matching performances.
It can be downloaded [here](https://gitlab.com/alicevision/trainedVocabularyTreeData/raw/master/vlfeat_K80L3.SIFT.tree).
* Sphere detection model (optional): for the automated sphere detection in stereo photometry.
It can be downloaded [here](https://gitlab.com/alicevision/SphereDetectionModel/-/raw/main/sphereDetection_Mask-RCNN.onnx).
* Semantic segmentation model (optional): for the semantic segmentation of objects.
It can be downloaded [here](https://gitlab.com/alicevision/semanticSegmentationModel/-/raw/main/fcn_resnet50.onnx).
* Color chart detection models (optional): for the detection of color charts.
It can be downloaded [here](https://gitlab.com/alicevision/ColorchartDetectionModel).

Environment variables need to be set for Meshroom to find those files:
```
ALICEVISION_SENSOR_DB=/path/to/database
ALICEVISION_VOCTREE=/path/to/voctree
ALICEVISION_SPHERE_DETECTION_MODEL=/path/to/detection/model
ALICEVISION_SEMANTIC_SEGMENTATION_MODEL=/path/to/segmentation/model
ALICEVISION_COLORCHARTDETECTION_MODEL_FOLDER=/path/to/ColorChartDetectionModel
```

If these variables are not set, Meshroom will expect those files to be located in `{ALICEVISION_ROOT}/share/aliceVision`.
