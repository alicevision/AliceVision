cmake_minimum_required(VERSION 3.30)

# Perform bundle fixup on all executables of an install directory
# and generates a standalone bundle with all required runtime dependencies.
#
# This script accepts the following parameters (pass via -D on the cmake -P command line):
#   - CMAKE_INSTALL_PREFIX      : install target path (required)
#   - BUNDLE_INSTALL_PREFIX     : bundle installation path (required)
#   - BUNDLE_LIBS_PATHS         : additional paths (semicolon-separated) to look for runtime dependencies
#   - BUNDLE_PYTHON_ROOT        : relocatable Python runtime to embed as <bundle>/python (optional)
#   - BUNDLE_PYTHON_REFERENCE   : Python executable AliceVision was built with, checked against BUNDLE_PYTHON_ROOT (optional)
#   - CMAKE_INSTALL_BINDIR      : relative bin dir  (default: bin)
#   - CMAKE_INSTALL_LIBDIR      : relative lib dir  (default: lib)
#   - CMAKE_INSTALL_INCLUDEDIR  : relative include dir (default: include)
#   - CMAKE_INSTALL_DATADIR     : relative data dir (default: share)

# ─── Validate required parameters ────────────────────────────────────────────

foreach(_required_var CMAKE_INSTALL_PREFIX BUNDLE_INSTALL_PREFIX)
    if(NOT ${_required_var})
        message(FATAL_ERROR "MakeBundle.cmake: ${_required_var} is not set. "
                            "Pass it with -D${_required_var}=<path>")
    endif()
endforeach()

# ─── Replicate GNUInstallDirs for -P script mode ──────────────────────────────
# include(GNUInstallDirs) cannot resolve lib vs lib64 when invoked via cmake -P,
# so we compute the FULL_ variants manually from whatever the caller passes in.

if(NOT CMAKE_INSTALL_BINDIR)
    set(CMAKE_INSTALL_BINDIR "bin")
endif()

if(NOT CMAKE_INSTALL_LIBDIR)
    # Attempt to auto-detect lib64 (matches GNUInstallDirs heuristic)
    if(EXISTS "/etc/debian_version")
        # Debian/Ubuntu multiarch – keep plain lib; the actual multiarch tuple
        # would need more work, but lib is the safe default.
        set(CMAKE_INSTALL_LIBDIR "lib")
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 8 AND EXISTS "/usr/lib64")
        set(CMAKE_INSTALL_LIBDIR "lib64")
    else()
        set(CMAKE_INSTALL_LIBDIR "lib")
    endif()
endif()

if(NOT CMAKE_INSTALL_INCLUDEDIR)
    set(CMAKE_INSTALL_INCLUDEDIR "include")
endif()

if(NOT CMAKE_INSTALL_DATADIR)
    set(CMAKE_INSTALL_DATADIR "share")
endif()

# Build absolute paths
if(NOT CMAKE_INSTALL_FULL_BINDIR)
    set(CMAKE_INSTALL_FULL_BINDIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}")
endif()
if(NOT CMAKE_INSTALL_FULL_LIBDIR)
    set(CMAKE_INSTALL_FULL_LIBDIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif()
if(NOT CMAKE_INSTALL_FULL_INCLUDEDIR)
    set(CMAKE_INSTALL_FULL_INCLUDEDIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}")
endif()
if(NOT CMAKE_INSTALL_FULL_DATADIR)
    set(CMAKE_INSTALL_FULL_DATADIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}")
endif()

# ─── Blacklist (AppImage excludelist) ────────────────────────────────────────
# https://github.com/AppImage/AppImages/blob/master/excludelist

set(LINUX_OS_LIB_BLACKLIST
    ld-linux
    ld-linux-x86-64
    libanl
    libBrokenLocale
    libcidn
    libcrypt
    libc
    libdl
    libm
    libmvec
    libnsl
    libnss_compat
    libnss_db
    libnss_dns
    libnss_files
    libnss_hesiod
    libnss_nisplus
    libnss_nis
    libpthread
    libresolv
    librt
    libthread_db
    libutil
    libstdc++
    libGL
    libdrm
    libglapi
    libX11
    libgio-2.0
    libasound
    libgdk_pixbuf-2.0
    libfontconfig
    libthai
    libfreetype
    libharfbuzz
    libcom_err
    libexpat
    libgcc_s
    libglib-2.0
    libgpg-error
    libICE
    libkeyutils
    libp11-kit
    libSM
    libusb-1.0
    libuuid
    libz
    libgobject-2.0
    libpangoft2-1.0
    libpangocairo-1.0
    libpango-1.0
    libjack
)

# ─── Platform overrides ───────────────────────────────────────────────────────

function(gp_resolve_item_override context item exepath dirs resolved_item_var resolved_var)
  # avoid log flood for those system libraries with non-absolute path
  if(item MATCHES "^(api-ms-win-)[^/]+dll"
     OR item MATCHES "^libomp[0-9]*\\.x86_64\\.dll$"
     OR item MATCHES "^(cublas|cublasLt|cufft|curand|cusolver|cusparse|nvrtc|nvJitLink)[0-9_]*\\.dll$"
     OR item MATCHES "^npp[a-z]+[0-9_]*\\.dll$"
     OR item MATCHES "^cudart[0-9_]*\\.dll$")
    # resolve item with fake absolute system path to keep them identified as system libs
    # By doing this, fixup_bundle:
    #   - won't complain about those libraries
    #   - won't embed them in the bundle
    # Note: libomp*.x86_64.dll is the LLVM OpenMP runtime (/openmp:llvm).
    # CUDA runtime DLLs (cuBLAS, cuFFT, NPP, etc.) require CUDA to be installed
    # on the target machine and must not be bundled.
    set(${resolved_item_var} "$ENV{SystemRoot}/system/${item}" PARENT_SCOPE)
    set(${resolved_var} TRUE PARENT_SCOPE)
  endif()
endfunction()

if(UNIX)
    function(gp_resolved_file_type_override resolved_file type_var)
        # Embed all "system" libs that are NOT on the AppImage blacklist.
        if("${${type_var}}" STREQUAL "system")
            get_filename_component(basename "${resolved_file}" NAME_WE)
            if(NOT basename IN_LIST LINUX_OS_LIB_BLACKLIST)
                set(${type_var} "embedded" PARENT_SCOPE)
            endif()
        endif()
    endfunction()
endif()

# ─── Diagnostics ─────────────────────────────────────────────────────────────

include(BundleUtilities)

message(STATUS "=== MakeBundle ===")
message(STATUS "  CMAKE_INSTALL_PREFIX    : ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  BUNDLE_INSTALL_PREFIX   : ${BUNDLE_INSTALL_PREFIX}")
message(STATUS "  CMAKE_INSTALL_BINDIR    : ${CMAKE_INSTALL_BINDIR}")
message(STATUS "  CMAKE_INSTALL_LIBDIR    : ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "  CMAKE_INSTALL_INCLUDEDIR: ${CMAKE_INSTALL_INCLUDEDIR}")
message(STATUS "  CMAKE_INSTALL_DATADIR   : ${CMAKE_INSTALL_DATADIR}")
message(STATUS "  CMAKE_INSTALL_FULL_BINDIR  : ${CMAKE_INSTALL_FULL_BINDIR}")
message(STATUS "  CMAKE_INSTALL_FULL_LIBDIR  : ${CMAKE_INSTALL_FULL_LIBDIR}")
message(STATUS "  CMAKE_INSTALL_FULL_INCLUDEDIR : ${CMAKE_INSTALL_FULL_INCLUDEDIR}")
message(STATUS "  CMAKE_INSTALL_FULL_DATADIR : ${CMAKE_INSTALL_FULL_DATADIR}")
message(STATUS "  BUNDLE_LIBS_PATHS       : ${BUNDLE_LIBS_PATHS}")
message(STATUS "  BUNDLE_PYTHON_ROOT      : ${BUNDLE_PYTHON_ROOT}")
message(STATUS "  BUNDLE_PYTHON_REFERENCE : ${BUNDLE_PYTHON_REFERENCE}")

# ─── Dependency lookup paths ─────────────────────────────────────────────────

if(WIN32)
    # On Windows, DLLs live next to the executables
    set(LIBS_LOOKUPS_PATHS "${CMAKE_INSTALL_FULL_BINDIR}")
else()
    set(LIBS_LOOKUPS_PATHS "${CMAKE_INSTALL_FULL_LIBDIR}")
    # GNUInstallDirs cannot distinguish lib vs lib64 in -P mode;
    # always probe the "64" variant as well (harmless if absent).
    list(APPEND LIBS_LOOKUPS_PATHS "${CMAKE_INSTALL_FULL_LIBDIR}64")
endif()

if(BUNDLE_LIBS_PATHS)
    list(APPEND LIBS_LOOKUPS_PATHS ${BUNDLE_LIBS_PATHS})
endif()
message(STATUS "  LIBS_LOOKUPS_PATHS      : ${LIBS_LOOKUPS_PATHS}")

# ─── Guard: bin dir must exist ───────────────────────────────────────────────

if(NOT EXISTS "${CMAKE_INSTALL_FULL_BINDIR}")
    message(FATAL_ERROR "MakeBundle.cmake: bin directory does not exist: "
                        "${CMAKE_INSTALL_FULL_BINDIR}")
endif()

# ─── Copy install tree into bundle ───────────────────────────────────────────

foreach(_dir
        "${CMAKE_INSTALL_FULL_BINDIR}"
        "${CMAKE_INSTALL_PREFIX}/lib"
        "${CMAKE_INSTALL_PREFIX}/lib64"
        "${CMAKE_INSTALL_FULL_INCLUDEDIR}"
        "${CMAKE_INSTALL_FULL_DATADIR}")
    if(EXISTS "${_dir}")
        file(COPY "${_dir}"
             DESTINATION "${BUNDLE_INSTALL_PREFIX}"
             USE_SOURCE_PERMISSIONS)
    else()
        message(STATUS "  Skipping non-existent directory: ${_dir}")
    endif()
endforeach()

# ─── fixup_bundle ────────────────────────────────────────────────────────────

get_bundle_all_executables("${BUNDLE_INSTALL_PREFIX}" BUNDLE_APPS)

if(NOT BUNDLE_APPS)
    message(FATAL_ERROR "MakeBundle.cmake: no executables found under "
                        "${BUNDLE_INSTALL_PREFIX}")
endif()

# fixup_bundle uses the first app as anchor; it fixes up all others automatically.
list(GET BUNDLE_APPS 0 MAIN_APP)
message(STATUS "  Main app for fixup_bundle: ${MAIN_APP}")
fixup_bundle("${MAIN_APP}" "" "${LIBS_LOOKUPS_PATHS}")

# ─── Move stray libs that fixup_bundle dropped into bin/ back into lib/ ──────

if(UNIX)
    set(_bundle_bindir "${BUNDLE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}")
    set(_bundle_libdir "${BUNDLE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")

    file(GLOB _LIBS_TO_MOVE "${_bundle_bindir}/lib*.so*")
    if(_LIBS_TO_MOVE)
        message(STATUS "  Moving ${CMAKE_LIST_LENGTH} stray libs from bin/ to lib/")
        file(COPY ${_LIBS_TO_MOVE}
             DESTINATION "${_bundle_libdir}"
             USE_SOURCE_PERMISSIONS)
        file(REMOVE ${_LIBS_TO_MOVE})
    endif()
endif()

# ─── Copy USD plugin directory from lookup paths ─────────────────────────────
# plugInfo.json and plugin subdirs must sit alongside the DLLs for the USD resolver.

if (WIN32)
    set(_bundle_bindir "${BUNDLE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}")
    foreach(_lookup_path IN LISTS BUNDLE_LIBS_PATHS)
        set(_usd_src "${_lookup_path}/usd")
        if(EXISTS "${_usd_src}")
            message(STATUS "  Copying USD plugin dir: ${_usd_src} -> ${_bundle_bindir}/usd")
            file(COPY "${_usd_src}" DESTINATION "${_bundle_bindir}" USE_SOURCE_PERMISSIONS)
            break()
        endif()
    endforeach()
endif(WIN32)

# ─── Embed the Python runtime ────────────────────────────────────────────────
# Copied after fixup_bundle, which must not rewrite the interpreter and its modules.
# The runtime must be the one AliceVision was built against (same build, not only same version).

if(BUNDLE_PYTHON_ROOT)
    if(WIN32)
        set(_python_exe_rel "python.exe")
    else()
        set(_python_exe_rel "bin/python3")
    endif()
    if(NOT EXISTS "${BUNDLE_PYTHON_ROOT}/${_python_exe_rel}")
        message(FATAL_ERROR "MakeBundle.cmake: no interpreter in BUNDLE_PYTHON_ROOT: "
                            "${BUNDLE_PYTHON_ROOT}/${_python_exe_rel}")
    endif()

    set(_python_check "import sys, importlib.util; print(sys.version); print(sys.base_prefix); print(importlib.util.find_spec('numpy') is None)")

    if(BUNDLE_PYTHON_REFERENCE)
        execute_process(COMMAND "${BUNDLE_PYTHON_REFERENCE}" -c "${_python_check}"
                        OUTPUT_VARIABLE _ref_out OUTPUT_STRIP_TRAILING_WHITESPACE
                        COMMAND_ERROR_IS_FATAL ANY)
        string(REPLACE "\n" ";" _ref_out "${_ref_out}")
        list(GET _ref_out 0 _ref_version)
        list(GET _ref_out 1 _ref_base_prefix)
        file(REAL_PATH "${_ref_base_prefix}" _ref_base_prefix)
        file(REAL_PATH "${BUNDLE_PYTHON_ROOT}" _bundle_python_root)
        if(NOT _ref_base_prefix STREQUAL _bundle_python_root)
            message(FATAL_ERROR "MakeBundle.cmake: AliceVision was built against the Python in ${_ref_base_prefix} "
                                "(${BUNDLE_PYTHON_REFERENCE}), but the bundle would embed ${_bundle_python_root}")
        endif()
    endif()

    message(STATUS "  Copying Python runtime: ${BUNDLE_PYTHON_ROOT} -> ${BUNDLE_INSTALL_PREFIX}/python")
    file(REMOVE_RECURSE "${BUNDLE_INSTALL_PREFIX}/python")
    file(MAKE_DIRECTORY "${BUNDLE_INSTALL_PREFIX}/python")
    file(COPY "${BUNDLE_PYTHON_ROOT}/"
         DESTINATION "${BUNDLE_INSTALL_PREFIX}/python"
         USE_SOURCE_PERMISSIONS
         PATTERN "__pycache__" EXCLUDE
         PATTERN "*.pdb" EXCLUDE)

    # The relocated copy must start in isolated mode and stay pristine (packages belong in venvs).
    execute_process(COMMAND "${BUNDLE_INSTALL_PREFIX}/python/${_python_exe_rel}" -I -B -c "${_python_check}"
                    OUTPUT_VARIABLE _bundle_out OUTPUT_STRIP_TRAILING_WHITESPACE
                    COMMAND_ERROR_IS_FATAL ANY)
    string(REPLACE "\n" ";" _bundle_out "${_bundle_out}")
    list(GET _bundle_out 0 _bundle_version)
    list(GET _bundle_out 2 _bundle_no_numpy)
    message(STATUS "  Bundled Python: ${_bundle_version}")
    if(BUNDLE_PYTHON_REFERENCE AND NOT _bundle_version STREQUAL _ref_version)
        message(FATAL_ERROR "MakeBundle.cmake: bundled Python '${_bundle_version}' differs from the build Python '${_ref_version}'")
    endif()
    if(NOT _bundle_no_numpy STREQUAL "True")
        message(FATAL_ERROR "MakeBundle.cmake: the Python runtime ${BUNDLE_PYTHON_ROOT} is not pristine (numpy is installed in it)")
    endif()

    # On Windows the Python DLL is also resolved into bin/ for the extension modules:
    # it must be the very same file as the interpreter's.
    if(WIN32)
        file(GLOB _bundle_python_dlls RELATIVE "${BUNDLE_INSTALL_PREFIX}/python" "${BUNDLE_INSTALL_PREFIX}/python/python3*.dll")
        foreach(_dll IN LISTS _bundle_python_dlls)
            set(_bin_dll "${BUNDLE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/${_dll}")
            if(EXISTS "${_bin_dll}")
                file(SHA256 "${_bin_dll}" _bin_dll_hash)
                file(SHA256 "${BUNDLE_INSTALL_PREFIX}/python/${_dll}" _python_dll_hash)
                if(NOT _bin_dll_hash STREQUAL _python_dll_hash)
                    message(FATAL_ERROR "MakeBundle.cmake: ${_bin_dll} differs from the bundled interpreter's ${_dll}")
                endif()
            endif()
        endforeach()
    endif()
endif()

message(STATUS "=== Bundle done: ${BUNDLE_INSTALL_PREFIX} ===")