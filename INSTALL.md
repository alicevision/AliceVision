=====================================
OpenMVG (open Multiple View Geometry)
=====================================

------------------


Build instructions
------------------

Required tools:
* Cmake 
* Git
* c/c++ compiler (gcc or visual studio or clang)

Getting the sources:

```bash
git clone --recursive https://github.com/openMVG/openMVG.git
```

or

```bash
git clone https://github.com/openMVG/openMVG.git
cd openMVG
git submodule init
git submodule update
```

As openMVG use some C++11 features you must have a c++11 ready compiler:
- Visual studio >= 2013
- GCC >= 4.7

--------------------------


CMake Options
--------------------------

* OpenMVG_USE_BOOST (default ON)
  Use Boost library (enable modules like localization/voctree and other features and optimizations.

* BOOST_NO_CXX11 (default OFF)
  If your Boost binaries are compiled without C++11 support, you need to set this option to avoid compilation errors.
  This is most likely to be the case if you use the system packages to install boost.

* OpenMVG_USE_OPENMP (default ON)
  Use OpenMP parallelization (huge impact on performances)

* OpenMVG_USE_CCTAG (default ON)
  Build with CCTag markers support.
  `-DCCTag_DIR=/path/to/cctag/install/lib/cmake/CCTag` (where CCTagConfig.cmake can be found)

* OpenMVG_USE_OPENGV (default OFF)
  Build with openGV for multi-cameras localization.
  `-DOPENGV_DIR=/path/to/opengv/install/` (where "include" and "lib" folders can be found)
  We recommend: `git clone https://github.com/poparteu/opengv.git --branch=cmake_fix_install`

* OpenMVG_USE_ALEMBIC (default OFF)
  Build with Alembic file format support.
  `-DAlembic_DIR=/path/to/alembic/install/lib/cmake/Alembic/` (where AlembicConfig.cmake can be found)
  With old Alembic versions (<1.6), you need to set many variables: ALEMBIC_ROOT, ALEMBIC_HDF5_ROOT, ALEMBIC_ILMBASE_ROOT, ALEMBIC_OPENEXR_ROOT.

* OpenMVG_USE_OPENCV (default: OFF): Build with openCV
  `-DOpenCV_DIR=/path/to/opencv/install/share/OpenCV/` (where OpenCVConfig.cmake can be found)

* OPENMVG_REQUIRE_CERES_WITH_SUITESPARSE (default: ON)
  By default, openMVG requires Ceres builded with SuiteSparse to ensure best performances but you can make SuiteSparse optional with this flag.

* OpenMVG_BUILD_SHARED (default OFF)
  Build OpenMVG as shared libs (instead of static libs)

* OpenMVG_BUILD_TESTS (default OFF)
  Build OpenMVG tests

* OpenMVG_BUILD_DOC (default ON)
  Build OpenMVG documentation

* OpenMVG_BUILD_EXAMPLES (default ON)
  Build OpenMVG samples applications (openMVG softwares are still builded)

* OpenMVG_BUILD_OPENGL_EXAMPLES (default OFF)
  Build OpenMVG openGL examples

* OpenMVG_BUILD_COVERAGE (default OFF)
  Enable code coverage generation (gcc only)


--------


General informations for openMVG SfM pipelines
--------------------------

OpenMVG can export graphs as graphviz .dot files and render them as SVG files.
If you want consider this graph visualization feature, please consider to install Graphviz.

-----------------


Linux compilation
-----------------

### Setup the required external library.

* `sudo apt-get install libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev`

* If you want see the view graph svg logs
  `sudo apt-get install graphviz`

### Clone and configure the project:

```bash
 git clone --recursive https://github.com/openMVG/openMVG.git
 mkdir openMVG_Build
 cd openMVG_Build
 cmake -DCMAKE_BUILD_TYPE=RELEASE . ../openMVG/src/
```

If you want enable unit tests and examples to the build:
```bash
cmake -DCMAKE_BUILD_TYPE=RELEASE -DOpenMVG_BUILD_TESTS=ON -DOpenMVG_BUILD_EXAMPLES=ON . ../openMVG/src/
```

In order to use the MOSEK 6 back-end for the linear programming openMVG module:

- Check that you have an up-to-date MOSEK licence, else openMVG MOSEK unit test will fail.

- Then:

  ```bash
  cmake -DCMAKE_BUILD_TYPE=RELEASE \
        -DMOSEK_SEARCH_HEADER="~/Documents/Lib/mosek/6/tools/platform/linux64x86/h" \
        -DMOSEK_SEARCH_LIB="~/Documents/Lib/mosek/6/tools/platform/linux64x86/bin" \
        . ../openMVG/src/
  ```

If you want to have an IDE openable project with codeblocks:

```bash
cmake -G "CodeBlocks - Unix Makefiles" -DCMAKE_BUILD_TYPE=RELEASE . ../openMVG/src/
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
cd openMVG_Samples
```
-------------------


Windows compilation
-------------------

* Checkout the project
  `git clone --recursive https://github.com/openMVG/openMVG.git`
* Open cmake-gui
  * Fill the source path with the src openMVG path.
  * Fill the build path with a new directory
  * Select your Visual Studio IDE and click configure and then generate
* Open the .sln solution created in your build directory.
  * Change the target to Release.
  * Compile the libraries and binaries samples.

-------------------


Mac OSX compilation
-------------------
```bash
git clone --recursive https://github.com/openMVG/openMVG.git
mkdir openMVG_Build
cd openMVG_Build
cmake -DCMAKE_BUILD_TYPE=RELEASE -G "Xcode" . ../openMVG/src/
```
If you want enable unit tests and examples to the build:
```bash
cmake -DCMAKE_BUILD_TYPE=RELEASE \
      -DOpenMVG_BUILD_TESTS=ON \
      -DOpenMVG_BUILD_EXAMPLES=ON \
      -G "Xcode" \
      . ../openMVG/src/
xcodebuild -configuration Release
```
--------------------


Using openCV sample
--------------------

Add `-DOpenMVG_USE_OPENCV=ON` to your cmake command line and set `OpenCV_DIR` variable to your openCV install directory where the OpenCVConfigure.cmake file can be found, i.e.: 
`-DOpenCV_DIR="/home/user/Dev/github/itseez/opencv_Build/install/share/OpenCV/" -DOpenMVG_USE_OPENCV=ON`

------------------------------------------------------------


Using OpenMVG as a third party library dependency in cmake
-------------------------------------------------------------

OpenMVG can be used as a third party once it have been installed.
Because it can use its own Ceres version, it is better to install it locally and not in system files.
So please consider using the `CMAKE_INSTALL_PREFIX` cmake variable to specify a local installation directory.

Here the syntax to add the variable to the cmake command line (use absolute path), e.g.: 
```bash
-DCMAKE_INSTALL_PREFIX:STRING="/home/user/Dev/github/openMVG_Build/openMVG_install"
```

Perform `make` and `make install`

Once the library has been installed, go to your project that want use OpenMVG as an external library and add:
```cmake
FIND_PACKAGE(OpenMVG REQUIRED)
INCLUDE_DIRECTORIES(${OPENMVG_INCLUDE_DIRS})
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main ${OPENMVG_LIBRARIES})
```

Specify to CMake where OpenMVG have been installed by using the cmake `OpenMVG_DIR` variable that must point to: `-DOpenMVG_DIR:STRING="YourInstallPath"/share/openMVG/cmake`

A message will be displayed if OpenMVG is found or not at the cmake configure step.

