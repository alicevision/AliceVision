Build instructions
------------------

Required tools:
* CMake >= 3.4 
* Git
* C/C++ compiler (gcc or visual studio or clang) with C++11 support.

### Dependencies

Required dependencies:
* zlib
* boost >= 1.61
* Geogram (https://gforge.inria.fr/frs/?group_id=5833)
* OpenImageIO >= 1.8
* OpenEXR
* CUDA >= 7.0

Optional:
* OpenMP

### CMake Options
* `ALICEVISION_BUILD_SHARED` (default `OFF`) Build AliceVision as shared libs (instead of static libs)
* `ALICEVISION_USE_OPENMP` (default: `ON`) Enable OpenMP parallelization
* `ALICEVISION_USE_CUDA` (default: `ON`) Enable build with CUDA (for depth map computation)

### Linux/OSX compilation

#### Clone and configure the project:
```bash
 git clone https://github.com/alicevision/CMPMVS.git
 mkdir cmpmvs_build
 cd cmpmvs_build
 cmake -DCMAKE_BUILD_TYPE=RELEASE . ../CMPMVS/
```

In case the dependencies are not installed in common system directories you can use some cmake variables to give hints 
where to find the libraries:
* `-DGEOGRAM_INSTALL_PREFIX:PATH=path/to/geogram/install` for Geogram
* `-DOPENIMAGEIO_LIBRARY_DIR_HINTS:PATH=/path/to/oiio/install/lib/` 
and `-DOPENIMAGEIO_INCLUDE_DIR:PATH=/path/to/oiio/install/include/` for OpenImageIO

**OSX**: if you are compiling with clang shipped with XCode, please note that OpenMP is not supported and you need to
disable OpenMP passing `-DALICEVISION_USE_OPENMP:BOOL=OFF`.

#### Compile the project

```bash
make
```

For a multi-core compilation (Replace NBcore with the number of threads)
```bash
make -j NBcore
```
