cmake_minimum_required(VERSION 3.30)

# Perform bundle fixup on all executables of an install directory
# and generates a standalone bundle with all required runtime dependencies.
#
# This script accepts the following parameters (pass via -D on the cmake -P command line):
#   - CMAKE_INSTALL_PREFIX      : install target path (required)
#   - BUNDLE_INSTALL_PREFIX     : bundle installation path (required)
#   - BUNDLE_LIBS_PATHS         : additional paths (semicolon-separated) to look for runtime dependencies
#   - CMAKE_INSTALL_BINDIR      : relative bin dir  (default: bin)
#   - CMAKE_INSTALL_LIBDIR      : relative lib dir  (default: lib)
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
    # Suppress errors for Windows API sets (api-ms-win-*.dll) — these are
    # virtual DLLs provided by the OS and must never be bundled.
    if(item MATCHES "^api-ms-win-[^/]+\\.dll$")
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
message(STATUS "  CMAKE_INSTALL_DATADIR   : ${CMAKE_INSTALL_DATADIR}")
message(STATUS "  CMAKE_INSTALL_FULL_BINDIR  : ${CMAKE_INSTALL_FULL_BINDIR}")
message(STATUS "  CMAKE_INSTALL_FULL_LIBDIR  : ${CMAKE_INSTALL_FULL_LIBDIR}")
message(STATUS "  CMAKE_INSTALL_FULL_DATADIR : ${CMAKE_INSTALL_FULL_DATADIR}")
message(STATUS "  BUNDLE_LIBS_PATHS       : ${BUNDLE_LIBS_PATHS}")

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
        "${CMAKE_INSTALL_FULL_LIBDIR}"
        "${CMAKE_INSTALL_FULL_DATADIR}")
    if(IS_SYMLINK "${_dir}")
        # Symlink directory (e.g. lib64 -> lib): file(COPY) would reproduce the
        # dangling symlink without its target. Copy the real directory instead,
        # then recreate the symlink so both names are valid in the bundle.
        get_filename_component(_link_name "${_dir}" NAME)
        get_filename_component(_real      "${_dir}" REALPATH)
        get_filename_component(_real_name "${_real}" NAME)
        if(EXISTS "${_real}")
            file(COPY "${_real}"
                 DESTINATION "${BUNDLE_INSTALL_PREFIX}"
                 USE_SOURCE_PERMISSIONS)
        endif()
        if(NOT EXISTS "${BUNDLE_INSTALL_PREFIX}/${_link_name}")
            file(CREATE_LINK "${_real_name}"
                 "${BUNDLE_INSTALL_PREFIX}/${_link_name}" SYMBOLIC)
            message(STATUS "  Created symlink: ${_link_name} -> ${_real_name}")
        endif()
    elseif(EXISTS "${_dir}")
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
        list(LENGTH _LIBS_TO_MOVE _n_libs)
        message(STATUS "  Moving ${_n_libs} stray libs from bin/ to lib/")
        # Move with RENAME (not COPY): it relocates symlinks as-is instead of
        # dereferencing them. fixup_bundle may leave a dangling symlink in bin/
        # (its target left in lib/), which file(COPY) cannot duplicate.
        foreach(_lib IN LISTS _LIBS_TO_MOVE)
            get_filename_component(_lib_name "${_lib}" NAME)
            file(RENAME "${_lib}" "${_bundle_libdir}/${_lib_name}")
        endforeach()
    endif()
endif()

message(STATUS "=== Bundle done: ${BUNDLE_INSTALL_PREFIX} ===")