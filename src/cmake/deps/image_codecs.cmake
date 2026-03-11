# =============================================================================
# deps/image_codecs.cmake — Image format libraries
#
# Dependencies: zlib (ZLIB_TARGET / ZLIB_CMAKE_FLAGS)
# Provides:     TIFF_CMAKE_FLAGS, PNG_CMAKE_FLAGS, JPEG_CMAKE_FLAGS,
#               LIBRAW_CMAKE_FLAGS, OPENEXR_CMAKE_FLAGS, ILMBASE_CMAKE_FLAGS
# =============================================================================

# ── libtiff ───────────────────────────────────────────────────────────────────

if(AV_BUILD_TIFF)
    set(TIFF_TARGET tiff)

    # libtiff uses autoconf, not CMake.
    ExternalProject_Add(${TIFF_TARGET}
        URL              ${DEP_TIFF_URL}
        URL_HASH         ${DEP_TIFF_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/tiff
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/tiff
        BINARY_DIR       ${BUILD_DIR}/tiff_build
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            <SOURCE_DIR>/configure
                --prefix=<INSTALL_DIR>
                --disable-tests
                --disable-docs
                --disable-tools
        # libtiff uses autoconf — make is the only available build tool here
        BUILD_COMMAND   make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND make install
        DEPENDS         ${ZLIB_TARGET}
    )

    set(TIFF_CMAKE_FLAGS
        -DTIFF_LIBRARY=${CMAKE_INSTALL_PREFIX}/lib/libtiff${CMAKE_SHARED_LIBRARY_SUFFIX}
        -DTIFF_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
    )
    av_register_dep(${TIFF_TARGET})
endif()

# ── libpng ────────────────────────────────────────────────────────────────────

if(AV_BUILD_PNG)
    set(PNG_TARGET png)

    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
        set(_png_neon OFF)
    else()
        set(_png_neon off)
    endif()

    av_add_cmake_dep(
        TARGET     ${PNG_TARGET}
        SOURCE_DIR png
        URL        ${DEP_PNG_URL}
        URL_HASH   ${DEP_PNG_HASH}
        EXTRA_CMAKE_FLAGS
            ${ZLIB_CMAKE_FLAGS}
            -DPNG_ARM_NEON=${_png_neon}
        DEPENDS    ${ZLIB_TARGET}
    )
    unset(_png_neon)

    set(PNG_CMAKE_FLAGS
        -DPNG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libpng${CMAKE_SHARED_LIBRARY_SUFFIX}
        -DPNG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
    )
endif()

# ── libjpeg-turbo ─────────────────────────────────────────────────────────────

if(AV_BUILD_JPEG)
    set(JPEG_TARGET turbojpeg)

    av_add_cmake_dep(
        TARGET     ${JPEG_TARGET}
        SOURCE_DIR turbojpeg
        URL        ${DEP_JPEG_URL}
        URL_HASH   ${DEP_JPEG_HASH}
        EXTRA_CMAKE_FLAGS ${ZLIB_CMAKE_FLAGS}
        DEPENDS    ${ZLIB_TARGET}
    )

    set(JPEG_CMAKE_FLAGS
        -DJPEG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libjpeg${CMAKE_SHARED_LIBRARY_SUFFIX}
        -DJPEG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
    )
endif()

# ── libraw ────────────────────────────────────────────────────────────────────

if(AV_BUILD_LIBRAW)
    set(LIBRAW_TARGET libraw)

    # Step 1 — fetch the unofficial CMake build-system overlay (no build/install)
    ExternalProject_Add(libraw_cmake
        GIT_REPOSITORY  ${DEP_LIBRAW_CMAKE_GIT_REPO}
        GIT_TAG         ${DEP_LIBRAW_CMAKE_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/libraw_cmake
        BINARY_DIR      ${BUILD_DIR}/libraw_cmake_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND     ""
        INSTALL_COMMAND   ""
    )

    # Step 2 — build libraw sources using the overlay CMakeLists.txt
    ExternalProject_Add(${LIBRAW_TARGET}
        GIT_REPOSITORY  ${DEP_LIBRAW_GIT_REPO}
        GIT_TAG         ${DEP_LIBRAW_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/libraw
        BINARY_DIR      ${CMAKE_CURRENT_BINARY_DIR}/libraw
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            cp <SOURCE_DIR>_cmake/CMakeLists.txt . &&
            cp -rf <SOURCE_DIR>_cmake/cmake . &&
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DENABLE_OPENMP=${AV_USE_OPENMP}
            -DENABLE_LCMS=ON
            -DENABLE_EXAMPLES=OFF
            ${ZLIB_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DINSTALL_CMAKE_MODULE_PATH:PATH=<INSTALL_DIR>/cmake
            <SOURCE_DIR>
        # libraw uses the cmake overlay — generator-agnostic build
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS libraw_cmake ${ZLIB_TARGET}
    )

    set(LIBRAW_CMAKE_FLAGS
        -DLIBRAW_PATH=${CMAKE_INSTALL_PREFIX}
        -DPC_LIBRAW_INCLUDEDIR=${CMAKE_INSTALL_PREFIX}/include
        -DPC_LIBRAW_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib
        -DPC_LIBRAW_R_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib
    )
    av_register_dep(${LIBRAW_TARGET})
endif()

# ── openexr ───────────────────────────────────────────────────────────────────

if(AV_BUILD_OPENEXR)
    set(OPENEXR_TARGET openexr)

    av_add_cmake_dep(
        TARGET     ${OPENEXR_TARGET}
        SOURCE_DIR openexr
        URL        ${DEP_OPENEXR_URL}
        URL_HASH   ${DEP_OPENEXR_HASH}
        EXTRA_CMAKE_FLAGS
            -DOPENEXR_BUILD_PYTHON_LIBS:BOOL=OFF
            -DBUILD_TESTING:BOOL=OFF
            -DOPENEXR_INSTALL_EXAMPLES:BOOL=OFF
            -DOPENEXR_BUILD_TOOLS:BOOL=OFF
            ${ZLIB_CMAKE_FLAGS}
        DEPENDS ${ZLIB_TARGET}
    )

    set(ILMBASE_CMAKE_FLAGS
        -DILMBASE_ROOT=${CMAKE_INSTALL_PREFIX}
        -DILMBASE_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include/OpenEXR
    )
    set(OPENEXR_CMAKE_FLAGS
        ${ILMBASE_CMAKE_FLAGS}
        -DOPENEXR_ROOT=${CMAKE_INSTALL_PREFIX}
        -DOPENEXR_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include
    )
endif()
