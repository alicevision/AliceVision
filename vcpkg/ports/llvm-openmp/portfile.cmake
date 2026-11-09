# Standalone build of the LLVM OpenMP runtime: libomp.dll and its import library libomp.lib.
#
# AliceVision is compiled with MSVC's /openmp:llvm, which links a default library named libomp.lib.
# The toolset's libomp.lib imports libomp140.x86_64.dll, which vc_redist does not install and which
# Visual Studio only ships in its debug_nonredist folder: it cannot be redistributed with AliceVision.
# AliceVision links this build of the same runtime instead (see the OpenMP section of src/CMakeLists.txt).
#
# Keep the version in sync with DEP_OPENMP_VERSION in src/cmake/DependenciesVersions.cmake.

vcpkg_check_linkage(ONLY_DYNAMIC_LIBRARY)

vcpkg_download_distfile(ARCHIVE
    URLS "https://github.com/llvm/llvm-project/releases/download/llvmorg-${VERSION}/llvm-project-${VERSION}.src.tar.xz"
    FILENAME "llvm-project-${VERSION}.src.tar.xz"
    SHA512 3557a955d55471671ae2f7b9c809affd59a29a6fb1e70a2a5d040dc1c6376246deb0635be8ca36cae09112981760e9afb128c822e5554bd722589fb8dee3f0df
)

# LLVM only releases the whole monorepo: extract the runtime and the shared CMake modules it uses, nothing else.
set(SOURCE_ROOT "${CURRENT_BUILDTREES_DIR}/src/llvm-project-${VERSION}.src")
file(REMOVE_RECURSE "${SOURCE_ROOT}")
file(MAKE_DIRECTORY "${CURRENT_BUILDTREES_DIR}/src")
vcpkg_execute_required_process(
    COMMAND "${CMAKE_COMMAND}" -E tar xf "${ARCHIVE}"
            "llvm-project-${VERSION}.src/openmp"
            "llvm-project-${VERSION}.src/cmake"
    WORKING_DIRECTORY "${CURRENT_BUILDTREES_DIR}/src"
    LOGNAME "extract-${TARGET_TRIPLET}"
)

# Generates the message catalog and the export definitions
vcpkg_find_acquire_program(PYTHON3)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_ROOT}/openmp"
    OPTIONS
        -DOPENMP_STANDALONE_BUILD=ON
        -DLIBOMP_ENABLE_SHARED=ON
        # libomp.dll, not the libomp140.<arch>.dll name of the Visual Studio build
        -DOPENMP_MSVC_NAME_SCHEME=OFF
        -DOPENMP_ENABLE_LIBOMPTARGET=OFF
        -DOPENMP_ENABLE_OMPT_TOOLS=OFF
        -DLIBOMP_OMPD_SUPPORT=OFF
        "-DPython3_EXECUTABLE=${PYTHON3}"
)

vcpkg_cmake_install()
vcpkg_copy_pdbs()

# On Windows, the runtime is also installed as libiomp5md (Intel OpenMP compatibility copies): not needed
foreach(_dir "" "/debug")
    file(REMOVE
        "${CURRENT_PACKAGES_DIR}${_dir}/bin/libiomp5md.dll"
        "${CURRENT_PACKAGES_DIR}${_dir}/lib/libiomp5md.lib")
endforeach()

# Move the headers out of include/: an LLVM omp.h there would shadow the toolset's omp.h for every
# other port and project using this tree (code built with /openmp:llvm uses the toolset's omp.h).
file(GLOB _headers "${CURRENT_PACKAGES_DIR}/include/*.h")
file(COPY ${_headers} DESTINATION "${CURRENT_PACKAGES_DIR}/include/${PORT}")
file(REMOVE ${_headers})
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
vcpkg_install_copyright(FILE_LIST "${SOURCE_ROOT}/openmp/LICENSE.TXT")
