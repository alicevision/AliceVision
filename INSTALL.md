
AliceVision
===========


Build instructions
------------------

Required tools:
* Cmake 
* Git
* c/c++ compiler (gcc or visual studio or clang)

Getting the sources:

```bash
git clone --recursive https://github.com/alicevision/aliceVision.git
```

or

```bash
git clone https://github.com/alicevision/aliceVision.git
cd aliceVision
git submodule init
git submodule update
```

As aliceVision use some C++11 features you must have a c++11 ready compiler:
- Visual studio >= 2013
- GCC >= 4.7


Building using external dependencies
--------------------------

AliceVision source tree contains some of the mandatory dependencies that are needed to build the library, and which will be built together with the libray. In order to build the library with existing versions of the dependencies (e.g. system installed libraries or user built libraries), and thus reduce the compilation time and favour the modularization, the paths where to find such libraries can be given at cmake command line. In particular:

* For Ceres solver library, `Ceres_DIR` can be passed pointing to where CeresConfig.cmake can be found.
  e.g. `-DCeres_DIR:PATH=/path/to/ceres/install/share/Ceres/`

* For FLANN library, `FLANN_INCLUDE_DIR_HINTS` can be passed pointing to the include directory, e.g.
  `-DFLANN_INCLUDE_DIR_HINTS:PATH=/path/to/flann/1.8.4/include/`

* For Eigen library, `EIGEN_INCLUDE_DIR_HINTS` can be passed pointing to the include directory, e.g.
  `-DEIGEN_INCLUDE_DIR_HINTS:PATH=/usr/local/include/eigen3`

At the end of the cmake process, a report shows for each library which version (internal/external) will be used in the building process, e.g.:

```
-- EIGEN: 3.2.4 (external)
-- CERES: 1.10.0 (external)
-- FLANN: 1.8.4 (external)
-- LIBTIFF: 4.0.4 (external)
-- LIBPNG: 1.6.18 (external)
-- LIBJPEG (external)
-- CLP: 1.15.11 (internal)
-- COINUTILS: 2.9.3 (internal)
-- OSI: 0.106.10 (internal)
-- LEMON: 1.3 (internal)
```


CMake Options
--------------------------

* `BOOST_NO_CXX11` (default `OFF`)
  If your Boost binaries are compiled without C++11 support, you need to set this option to avoid compilation errors.
  This is most likely to be the case if you use the system packages to install boost.

* `ALICEVISION_USE_OPENMP` (default `ON`)
  Use OpenMP parallelization (huge impact on performances)

* `ALICEVISION_USE_CCTAG` (default `ON`)
  Build with CCTag markers support.
  `-DCCTag_DIR:PATH=/path/to/cctag/install/lib/cmake/CCTag` (where CCTagConfig.cmake can be found)

* `ALICEVISION_USE_OPENGV` (default `AUTO`)
  Build with openGV for multi-cameras localization.
  `-DOPENGV_DIR:PATH=/path/to/opengv/install/` (where "include" and "lib" folders can be found)
  We recommend: `git clone https://github.com/alicevision/opengv.git --branch=cmake_fix_install`

* `ALICEVISION_USE_ALEMBIC` (default `AUTO`)
  Build with Alembic file format support (required version >= 1.7).
  `-DAlembic_DIR:PATH=/path/to/alembic/install/lib/cmake/Alembic/` (where AlembicConfig.cmake can be found)
   
* `ALICEVISION_USE_OIIO` (default: `AUTO`)
  Build code depending on OpenImageIO
  
* `ALICEVISION_USE_OPENMP` (default: `AUTO`)
  Enable OpenMP parallelization

* `ALICEVISION_USE_CCTAG` (default: `AUTO`)
  Enable CCTAG markers
  
* `ALICEVISION_USE_POPSIFT` (default: `AUTO`)
  Enable GPU SIFT implementation
  
* `ALICEVISION_USE_OPENGV` (default: `AUTO`)
  Enable use of OpenGV algorithms
  
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



General informations for aliceVision SfM pipelines
--------------------------

AliceVision can export graphs as graphviz .dot files and render them as SVG files.
If you want consider this graph visualization feature, please consider to install Graphviz.


Linux compilation
-----------------

### Setup the required external library.

* `sudo apt-get install libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev`

* If you want see the view graph svg logs
  `sudo apt-get install graphviz`

### Clone and configure the project:

```bash
 git clone --recursive https://github.com/alicevision/aliceVision.git
 mkdir aliceVision_Build
 cd aliceVision_Build
 cmake -DCMAKE_BUILD_TYPE=RELEASE . ../aliceVision/src/
```

If you want enable unit tests and examples to the build:
```bash
cmake -DCMAKE_BUILD_TYPE=RELEASE -DALICEVISION_BUILD_TESTS=ON -DALICEVISION_BUILD_EXAMPLES=ON . ../aliceVision/src/
```

In order to use the MOSEK 6 back-end for the linear programming aliceVision module:

- Check that you have an up-to-date MOSEK licence, else aliceVision MOSEK unit test will fail.

- Then:

  ```bash
  cmake -DCMAKE_BUILD_TYPE=RELEASE \
        -DMOSEK_SEARCH_HEADER="~/Documents/Lib/mosek/6/tools/platform/linux64x86/h" \
        -DMOSEK_SEARCH_LIB="~/Documents/Lib/mosek/6/tools/platform/linux64x86/bin" \
        . ../aliceVision/src/
  ```

If you want to have an IDE openable project with codeblocks:

```bash
cmake -G "CodeBlocks - Unix Makefiles" -DCMAKE_BUILD_TYPE=RELEASE . ../aliceVision/src/
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

Have fun with the samples
```bash
cd aliceVision_Samples
```
-------------------


Windows compilation
-------------------

* Checkout the project
  `git clone --recursive https://github.com/alicevision/aliceVision.git`
* Open cmake-gui
  * Fill the source path with the src aliceVision path.
  * Fill the build path with a new directory
  * Select your Visual Studio IDE and click configure and then generate
* Open the .sln solution created in your build directory.
  * Change the target to Release.
  * Compile the libraries and binaries samples.

-------------------


Mac OSX compilation
-------------------
```bash
git clone --recursive https://github.com/alicevision/aliceVision.git
mkdir aliceVision_Build
cd aliceVision_Build
cmake -DCMAKE_BUILD_TYPE=RELEASE -G "Xcode" . ../aliceVision/src/
```
If you want enable unit tests and examples to the build:
```bash
cmake -DCMAKE_BUILD_TYPE=RELEASE \
      -DALICEVISION_BUILD_TESTS=ON \
      -DALICEVISION_BUILD_EXAMPLES=ON \
      -G "Xcode" \
      . ../aliceVision/src/
xcodebuild -configuration Release
```
--------------------


Using openCV sample
--------------------

Add `-DAliceVision_USE_OPENCV=ON` to your cmake command line and set `OpenCV_DIR` variable to your openCV install directory where the OpenCVConfigure.cmake file can be found, i.e.: 
`-DOpenCV_DIR="/home/user/Dev/github/itseez/opencv_Build/install/share/OpenCV/" -DAliceVision_USE_OPENCV=ON`

------------------------------------------------------------


Using AliceVision as a third party library dependency in cmake
-------------------------------------------------------------

AliceVision can be used as a third party once it have been installed.
Because it can use its own Ceres version, it is better to install it locally and not in system files.
So please consider using the `CMAKE_INSTALL_PREFIX` cmake variable to specify a local installation directory.

Here the syntax to add the variable to the cmake command line (use absolute path), e.g.: 
```bash
-DCMAKE_INSTALL_PREFIX:STRING="/home/user/Dev/github/aliceVision_Build/aliceVision_install"
```

Perform `make` and `make install`

Once the library has been installed, go to your project that want use AliceVision as an external library and add:
```cmake
FIND_PACKAGE(AliceVision REQUIRED)
INCLUDE_DIRECTORIES(${ALICEVISION_INCLUDE_DIRS})
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main ${ALICEVISION_LIBRARIES})
```

Specify to CMake where AliceVision have been installed by using the cmake `AliceVision_DIR` variable that must point to: `-DAliceVision_DIR:STRING="YourInstallPath"/share/aliceVision/cmake`

A message will be displayed if AliceVision is found or not at the cmake configure step.

