# =============================================================================
# deps/zlib.cmake
# Dependencies: (none)
# Provides:     ZLIB_TARGET, ZLIB_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_ZLIB)
    set(ZLIB_TARGET zlib)

    av_add_cmake_dep(
        TARGET     ${ZLIB_TARGET}
        SOURCE_DIR zlib
        URL        ${DEP_ZLIB_URL}
        URL_HASH   ${DEP_ZLIB_HASH}
        EXTRA_CMAKE_FLAGS
            -DZLIB_BUILD_STATIC:BOOL=OFF
    )

    set(ZLIB_CMAKE_FLAGS -DZLIB_ROOT=${CMAKE_INSTALL_PREFIX})
endif()
