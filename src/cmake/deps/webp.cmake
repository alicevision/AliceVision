# =============================================================================
# deps/webp.cmake
# Dependencies: (none)
# Provides:     WEBP_TARGET, WEBP_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_WEBP)
    set(WEBP_TARGET webp)

    av_add_cmake_dep(
        TARGET     ${WEBP_TARGET}
        SOURCE_DIR webp
        URL        ${DEP_WEBP_URL}
        URL_HASH   ${DEP_WEBP_HASH}
        EXTRA_CMAKE_FLAGS
            -DWEBP_BUILD_ANIM_UTILS:BOOL=OFF
            -DWEBP_BUILD_CWEBP:BOOL=OFF
            -DWEBP_BUILD_DWEBP:BOOL=OFF
            -DWEBP_BUILD_GIF2WEBP:BOOL=OFF
            -DWEBP_BUILD_IMG2WEBP:BOOL=OFF
            -DWEBP_BUILD_VWEBP:BOOL=OFF
            -DWEBP_BUILD_WEBPINFO:BOOL=OFF
            -DWEBP_BUILD_WEBPMUX:BOOL=OFF
            -DWEBP_BUILD_EXTRAS:BOOL=OFF
    )

    set(WEBP_CMAKE_FLAGS -DWebP_ROOT=${CMAKE_INSTALL_PREFIX})
endif()
