# =============================================================================
# deps/jpeg.cmake
# Dependencies: zlib
# Provides:     JPEG_TARGET, JPEG_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_JPEG)
    set(JPEG_TARGET turbojpeg)

    av_add_cmake_dep(
        TARGET     ${JPEG_TARGET}
        SOURCE_DIR turbojpeg
        URL        ${DEP_JPEG_URL}
        URL_HASH   ${DEP_JPEG_HASH}
        EXTRA_CMAKE_FLAGS
            ${ZLIB_CMAKE_FLAGS}
            -DENABLE_STATIC:BOOL=OFF
        DEPENDS    ${ZLIB_TARGET}
    )

    set(JPEG_CMAKE_FLAGS
        -DJPEG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libjpeg${CMAKE_SHARED_LIBRARY_SUFFIX}
        -DJPEG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
    )
endif()
