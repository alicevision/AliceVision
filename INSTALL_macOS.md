# Building the project on macOS

Make sure to read the whole document at least *once* before compiling the project! If you find any bugs in the build infrastructure, consider opening a bug report on the [GitHub Issue page](https://github.com/alicevision/AliceVision/issues).

## Platform Support

AliceVision is supported for both `arm64` and `x86_64` based Macs. Cross-compiling between the two architectures is supported (see following remarks and the build instructions for cross-compilations below).

Note that support for `x86_64` based Macs might be removed at some point in the future, as Apple is slowly reaching EOL for Intel Macs.

Any CUDA-related functionality is disabled and unavailable on macOS. NVIDIA has long stopped shipping the CUDA toolkit for macOS and using a Mac compatible with one of the last suppported cards is too unlikely to even try hacking in Apple+CUDA support into the project. This currently implies the following:

- PopSIFT is completely unavailable and attempting to build it will cause a CMake error.
- CCTag won't use its CUDA implementation and instead uses the CPU backend.
- ONNX Runtime with GPU support is unavailable.
- The DepthMap library from AliceVision (and therefore all its dependants) are unavailable due to them being implemented only in CUDA. This might change in the future if new DepthMap backends are added.

## Building from Source

Building on macOS has a few important remarks:

- It is *highly* recommended to build with `ALICEVISION_BUILD_DEPENDENCIES=ON`. This ensures that all required dependencies are available and are fulfilling API and ABI requirements. Relying on dependencies provided by a package mangager is not supported by the project (see the following note).

- It is *highly* recommended to have a clean build environment when compiling AliceVision for the host architecture (only the required dependencies and tools should be installed). Due to the amount of dependencies involved we cannot guarantee that package managers are always shipping the required components and that the versions available are API and ABI compatible[^1]. While it might be harmless for one user, we cannot test every possible combination and must therefore declare a build with package-manager dependencies _unsupported_.

- It is *mandatory* to have a clean build system when you are cross-compiling from `arm64` to `x86_64` and vice-versa. We simply cannot force any subprojects and dependencies to not look for libraries provided by your package manager[^2], which *will* cause linker errors in the build process[^3].

- It is *mandatory* to have a clean build system if you want to compile a package that is redistributable (including all dependencies). This ensures that there are no external dependencies which one subproject silently pulled in. The danger in this case stems from the fact that this could go completely unnoticed, because on the build machine all dependencies are available and the package therefore 'just works'™[^4].

- Besides some external build tools, the project can build *all* required dependencies from source.

- As CMake generator (`-G` flag on the CLI) we only guarantee support for "Unix Makefiles" and "Ninja". Other generators might work, but that is not guaranteed (Xcode is known to do funny things and does *not* work with embedded dependencies!). You *always* need to have the `make` program on your `PATH` though (see below). Ninja appears to be the fastest as of writing.

- By default, the project optimizes the build for the CPU architecture of the build machine. This is usually desirable to ensure proper usage of SIMD instructions. If you need broder support, especially for redistribution, consider specifiying the target architecture when configuring the project with CMake: If you want to disable optimization completely, pass `TARGET_ARCHITECTURE=none`. In the same manner you can explicitly set the architecture you want to optimize for[^5]. You can enable verbose output of this process by passing `OFA_VERBOSE=ON` on the CMake CLI. Only change this if you know what you are doing: Tampering with these options will produce binaries that in the worst case will fault at runtime and in the best case are many times slower than expected.

- For some other Apple-specific CMake options see this section: [CMake Options for Apple](#cmake-options-for-apple).

## Required external tools

These differ slightly depending on the target architecture you want to build for. Note that if you are cross-compiling, you still need the additional tools for the target architecture. As these are only build-time tools, they don't have to be compiled *for* the target architecture, they only need to be available on your `$PATH` on the build machine.

### arm64 (Apple Silicon Macs, M-CPUs)

- [x] A working C/C++ compiler with C++20 support (e.g., Xcode/Xcode Command Line Tools)[^6]
- [x] cmake >= 3.25 but < 4.0 (some dependencies do not support CMake 4)
- [x] make (included in Xcode/Xcode Command Line Tools)
- [x] autoconf (Homebrew, MacPorts, Nix)
- [x] automake (Homebrew, MacPorts, Nix)
- [x] pkgconfig (Homebrew, MacPorts, Nix)
- [x] gettext (Homebrew, MacPorts, Nix)
- [x] m4 (Homebrew, MacPorts, Nix)
- [x] BISON[^7] (Homebrew, MacPorts, Nix)
- [x] NumPy[^8] (Homebrew, pip)

### x86_64 (Intel Macs, Core-i-CPUs)

- [x] ALL FROM ABOVE
- [x] nasm[^9] (Homwbrew, MacPorts, Nix)

## Build Instructions for native compilation

1. Create a build directory (in-source builds are unsupported):

`mkdir build && cd build`

2. Configure the project from the build directory:
  ```bash
  cmake \
    -DCMAKE_BUILD_TYPE=<Release|MinSizeRel|Debug|RelWithDebInfo> \
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> \  # Omit this if you want a system-wide install
    -DALICEVISION_BUILD_DEPENDENCIES=ON \  # Mandatory for Apple targets
    -DAV_BUILD_DEPENDENCIES_PARALLEL=<NB_THREADS> \  # Setting this to 0 will use all threads available
    <OTHER_CMAKE_OPTIONS>... \  # To enable or disable other options
    <PATH_TO_ALICEVISION_SOURCE_DIR>  # Can be relative and will usually just be '..'
  ```

3. Start build:

You might see *a lot* of warning messages, especially from the embedded dependencies' build process. This is expected and as long as no errors occur, you shouldn't care.

`cmake --build <PATH_TO_BUILDDIR>    # Note the missing --parellel option: Omit it to avoid build issues. This is handled internally by AV_BUILD_DEPENDENCIES_PARALLEL.`

4. Install project:

`(sudo) cmake --install <PATH_TO_BUILDDIR>    # Use sudo for a system-wide install`

[OPTIONAL: 5. Create a bundle]

This target creates a self-contained bundle (i.e., a folder containing a `lib` and a `bin` folder, with no external dependencies besides any system libraries/Frameworks)[^9]. Any additional required resources (e.g., `share` folder) must be copied manually. This is mainly useful if you want to create a redistributable bundle, especially for use in Meshroom.

`(sudo) cmake --build <PATH_TO_BUILDDIR> --target darwin-bundle    # Use sudo if the bundle should be created system-wide`

## Build Instructions for cross compilation

Enabling cross-compilation is done by setting `CMAKE_OSX_ARCHITECTURES` on the CMake CLI to *either* `arm64` *or* `x86_64`. Compiling universal binaries is *not* suppported at this point.
When cross-compiling, the Optimize-For-Architecture logic will set some reasonable defaults:
- For `arm64`: The default target architecture will be `apple-m1`, making the resulting binaries compatible with all Apple Silicon Macs.
- For `x86_64`: The default target architecture will be `skylake`, making the resulting binaries compatible with all Intel Macs not older than 2015.
If you want to target a different machine, consider setting `TARGET_ARCHITECTURE` on the CMake CLI to a supported value[^5].

1. Create a build directory (in-source builds are unsupported):

`mkdir build-cross && cd build-cross`

2. Configure the project from the build directory:
  ```bash
  cmake \
    -DCMAKE_BUILD_TYPE=<Release|MinSizeRel|Debug|RelWithDebInfo> \
    -DCMAKE_OSX_ARCHITECTURES=<arm64|x86_64> \  # Sets the target architecture
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> \  # Omit this if you want a system-wide install
    -DALICEVISION_BUILD_DEPENDENCIES=ON \  # Mandatory for Apple targets
    -DAV_BUILD_DEPENDENCIES_PARALLEL=<NB_THREADS> \  # Setting this to 0 will use all threads available
    <OTHER_CMAKE_OPTIONS>... \  # To enable or disable other options
    <PATH_TO_ALICEVISION_SOURCE_DIR>  # Can be relative and will usually just be '..'
  ```

3. Start build:

You might see *a lot* of warning messages, especially from the embedded dependencies' build process. This is expected and as long as no errors occur, you shouldn't care.

`cmake --build <PATH_TO_BUILDDIR>    # Note the missing --parallel option: Omit it to avoid build issues. This is handled internally by AV_BUILD_DEPENDENCIES_PARALLEL.`

4. Install project:

`(sudo) cmake --install <PATH_TO_BUILDDIR>    # Use sudo for a system-wide install`

[OPTIONAL: 5. Create a bundle]

This target creates a self-contained bundle (i.e., a folder containing a `lib` and a `bin` folder, with no external dependencies besides any system libraries/Frameworks)[^10]. It also copies known support folders into the bundle (such as the AliceVision headers, the `share/aliceVision` folder and the `share/meshroom` folder). Any additional assets must be copied manually.

`(sudo) cmake --build <PATH_TO_BUILDDIR> --target darwin-bundle    # Use sudo if the bundle should be created system-wide`

## NOTE: Running the freshly-built binaries and XprotectService

If you have built any of the software targets or tests, there is a solid chance that you also want to run them... but you might notice that for some binaries the first startup can be *very* (and I mean *very*) slow. It might seem like your machine is doing nothing. You need to be partient here. AliceVision has a lot of dependencies and each time a new Mach-O file (that is, each binary and each dependent .framework/.dylib) is loaded the very first time by the dynamic linker (`dyld`), it performs a malware scan. You can see this happening in the Activity Monitor, where a process called "XprotectService" will be running.
Depending on your machine, this could take *up to several minutes* when a binary is launched the first time. So just wait for it to eventually finish and all subsequent launches will be near-immediate.

## CMake Options for Apple

These are some influential CMake options specific to Apple:

| Option    | Description | Available Values | Default Value |
| --------- | ----------- | ---------------- | ------------- |
| `CMAKE_OSX_ARCHITECTURES` | Sets the target architecture to compile for | Either `arm64` or `x86_64` | `${CMAKE_HOST_SYSTEM_PROCESSOR}` |
| `ALICEVISION_USE_RPATH` | Whether to use @rpath instead of absolute paths for resolving dependencies (highly recommended) | `ON` / `OFF` | `ON` |
| `BUILD_APPLE_FRAMEWORKS` | Whether to build Framework bundles instead of plain dynamic libraries | `ON` / `OFF` | `ON` |
| `AV_ONNX_APPLE_ARCH` | What architecture to download for the ONNX Runtime (only active if `AV_BUILD_ONNXRUNTIME=ON`) | Either `arm64` or `x86_64` | `${CMAKE_OSX_ARCHITECTURES}` |
| `AV_BUILD_OPENMP` | Whether to build an embedded OpenMP (only active if `ALICEVISION_BUILD_DEPENDENCIES=ON`, highly recommended when using AppleClang) | `ON` / `OFF` | `ON` |
| `AV_BUILD_LAPACK` | Whether to build an embedded BLAS/LAPACK (not recommended, Apple provides it through `Accelerate.framework`, requires a Fortran compiler to be available on `$PATH`) | `ON` / `OFF` | `OFF` |
| `AV_BUILD_SUITESPARSE` | Whether to build an embedded Suitesparse (not recommended, Apple provides an equivalent for Sparse Solvers through `Accelerate.framework`, will massively increase final bundle size) | `ON` / `OFF` | `OFF` |
| `AV_BUILD_ZLIB` | Whether to build an embedded zlib (might be needed for older versions of macOS, especially when redestributing) |  `ON` / `OFF` | `OFF` |
| `AV_BUILD_PCL` | Whether to build an embedded PointCloudLibrary (only required if you plan to build an embedded USD and use the `aliceVision_exportUSD` software) | `ON` / `OFF` | `OFF` |
| `AV_BUILD_USD` | Whether to build an embedded UniversalSceneDescription library (only required if you plan to use the `aliceVision_exportUSD` software) | `ON` / `OFF` | `OFF` |
| `ALICEVISION_REQUIRE_CERES_WITH_ACCELERATESPARSE` | Whether to require the Ceres dependency to be built with `AccelerateSparse`/`Accelerate.framework` (highly recommended to match SuiteSparse speeds) | `ON` / `OFF` | `ON` |
| `ALICEVISION_BUNDLE_PREFIX` | Where to place the bundle created by `make darwin-bundle` | Any path | `${CMAKE_INSTALL_PREFIX}/bundle` |

[^1]: If you see linker errors, this should be the very first thing to check! Do the headers match the package of the library that is linked in? Are any non-project dependencies included from package manager directories (e.g., `/opt/homebrew`, `/usr/local`, `/opt/local`)? If so, clean your *whole* build folder, remove the offending packages and try again.

[^2]: Take a look at `CMAKE_IGNORE_PATH` for example: While we can pass that to dependencies, it only affects `find_package()` calls that use `CONFIG` mode. If a subproject provides its own `Find-X.cmake` module, there is no way for us to exclude certain prefixes that could cause trouble.

[^3]: If CMake would actually check for architecture compatibility in the configure step, we could just emit a nice and clear error to give some hint to the user. But as this is not the case, CMake will happily accept *any* architecture and only the final link step will tell that there was an architecture mismatch (if you can make out the one line in between the thousand lines of messages).

[^4]: When being transferred to a different machine there are essentially three ways this could go:
      (1) The other machine has all and compatible dependencies in the right location: It works. Lucky you.
      (2) The other machine has all but non-compatible dependencies in the right location: It might work, if they are ABI and API compatible. If not, the worst case would be a SIGSEGV or a SIGABRT and you have no idea by what or why they were caused. Maybe `dyld` will complain about missing libraries, if it cannot find the correct version (similar to (3)).
      (3) The other machine is missing dependencies or they are in the wrong location: This might be the best case scenario, because the error will be relatively clear: You will see something like `dyld: Library not loaded: <LIBRARY_NAME>. Referenced from: <SOME_BINARY>. Reason: tried <SOME_PATHS_WITH_ERROR_REASON>`.

[^5]: For a list of supported values see: [Supported Architectures](src/cmake/OFA/SupportedArchitectures.md).

[^6]: We expect a GNU-compatible compiler, so AppleClang, LLVM Clang, GCC and any LLVM derivatives with a compatible CLI interface should work. Cross-compiling between different architectures is only supported with AppleClang and LLVM Clang.

[^7]: This is only required if you intend to build SWIG from source. It requires a BISON newer than the one provided by Apple and therefore the external binary must be on your `$PATH` *first*. Look at the documentation of your shell on how to do this.

[^8]: This is only required if you plan to build the AliceVision SWIG bindings.

[^9]: Alternatively you can also use `yasm`. These are mainly required by libVPX and ffmpeg.

[^10]: The underlying Python script performs the following steps:
      (1) Extracts all required dependencies and available rpaths
      (2) Recursively checks if all dependencies can be resolved (and if the architectures match) on a per-file basis
      (3) Copies all input files and resolved dependencies into the respective folders
