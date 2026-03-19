# =============================================================================
# deps/tiff.cmake
# Dependencies: zlib
# Provides:     TIFF_TARGET, TIFF_CMAKE_FLAGS
#
# =============================================================================

if(AV_BUILD_TIFF)
    set(TIFF_TARGET tiff)

    av_add_cmake_dep(
        TARGET         ${TIFF_TARGET}
        SOURCE_DIR     tiff
        GIT_REPOSITORY ${DEP_TIFF_GIT_REPO}
        GIT_TAG        ${DEP_TIFF_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            ${ZLIB_CMAKE_FLAGS}
            -Dtiff-tools:BOOL=OFF
            -Dtiff-tests:BOOL=OFF
            -Dtiff-contrib:BOOL=OFF
            -Dtiff-docs:BOOL=OFF
        DEPENDS        ${ZLIB_TARGET}
    )

    set(TIFF_CMAKE_FLAGS
        -DTIFF_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libtiff${CMAKE_SHARED_LIBRARY_SUFFIX}
        -DTIFF_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
    )
    av_register_dep(${TIFF_TARGET})
endif()
