# Repackage a prebuilt, relocatable CPython instead of building it from source.
# Linux uses the same python-build-standalone release (docker/install-python-standalone.sh),
# so the dependencies, AliceVision and the bundle all run on identical interpreter binaries.
# Keep the release in sync with docker/install-python-standalone.sh.
set(PBS_RELEASE "20261001")
set(PBS_ARCHIVE "cpython-${VERSION}+${PBS_RELEASE}-x86_64-pc-windows-msvc-install_only.tar.gz")

vcpkg_check_linkage(ONLY_DYNAMIC_LIBRARY)

string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" PYTHON_VERSION "${VERSION}")
set(PYTHON_VERSION_MAJOR "${CMAKE_MATCH_1}")
set(PYTHON_VERSION_MINOR "${CMAKE_MATCH_2}")
set(PYTHON_HAS_EXTENSIONS ON)
set(PYTHON_LIB_NAME "python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")

vcpkg_download_distfile(ARCHIVE
    URLS "https://github.com/astral-sh/python-build-standalone/releases/download/${PBS_RELEASE}/cpython-${VERSION}%2B${PBS_RELEASE}-x86_64-pc-windows-msvc-install_only.tar.gz"
    FILENAME "${PBS_ARCHIVE}"
    SHA512 f87fbd1b5604bbc172dc1091bef78b434a73d74e051a1c309ebf8e5ac6d6cf36e7edd31eb34b2a43e0c81b3c5b08d7c0d7be07023b0c51e23e384e9e47021c06
)

vcpkg_extract_source_archive(SOURCE_PATH ARCHIVE "${ARCHIVE}")

# Full relocatable interpreter (python.exe, Lib, DLLs, Include, libs) in the location used by the
# upstream port, so it can be used as-is for venvs and copied as-is into the bundle.
file(COPY "${SOURCE_PATH}/" DESTINATION "${CURRENT_PACKAGES_DIR}/tools/${PORT}"
    PATTERN "*.pdb" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
)

# Development files in the standard vcpkg layout, as installed by the upstream port.
file(COPY "${SOURCE_PATH}/include/" DESTINATION "${CURRENT_PACKAGES_DIR}/include/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
file(INSTALL "${SOURCE_PATH}/libs/${PYTHON_LIB_NAME}.lib" "${SOURCE_PATH}/libs/python${PYTHON_VERSION_MAJOR}.lib"
    DESTINATION "${CURRENT_PACKAGES_DIR}/lib")
file(INSTALL "${SOURCE_PATH}/${PYTHON_LIB_NAME}.dll" "${SOURCE_PATH}/python${PYTHON_VERSION_MAJOR}.dll"
    DESTINATION "${CURRENT_PACKAGES_DIR}/bin")

# Release-only package: no debug interpreter is provided by python-build-standalone.
set(VCPKG_POLICY_MISMATCHED_NUMBER_OF_BINARIES enabled)
set(VCPKG_POLICY_ALLOW_EXES_IN_BIN enabled)
set(VCPKG_POLICY_DLLS_WITHOUT_EXPORTS enabled)
set(VCPKG_POLICY_SKIP_MISPLACED_CMAKE_FILES_CHECK enabled)

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE.txt")

file(READ "${CMAKE_CURRENT_LIST_DIR}/usage" usage)
file(READ "${CMAKE_CURRENT_LIST_DIR}/usage.win" usage_extra)
file(WRITE "${CURRENT_PACKAGES_DIR}/share/${PORT}/usage" "${usage}\n${usage_extra}")

function(_generate_finder)
    cmake_parse_arguments(PythonFinder "NO_OVERRIDE;SUPPORTS_ARTIFACTS_PREFIX" "DIRECTORY;PREFIX" "" ${ARGN})
    configure_file(
        "${CMAKE_CURRENT_LIST_DIR}/vcpkg-cmake-wrapper.cmake"
        "${CURRENT_PACKAGES_DIR}/share/${PythonFinder_DIRECTORY}/vcpkg-cmake-wrapper.cmake"
        @ONLY
    )
endfunction()

_generate_finder(DIRECTORY "python" PREFIX "Python" SUPPORTS_ARTIFACTS_PREFIX)
_generate_finder(DIRECTORY "python3" PREFIX "Python3" SUPPORTS_ARTIFACTS_PREFIX)
_generate_finder(DIRECTORY "pythoninterp" PREFIX "PYTHON" NO_OVERRIDE)

configure_file("${CMAKE_CURRENT_LIST_DIR}/vcpkg-port-config.cmake" "${CURRENT_PACKAGES_DIR}/share/${PORT}/vcpkg-port-config.cmake" @ONLY)
