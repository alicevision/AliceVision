# =============================================================================
# deps/ffmpeg.cmake
# Dependencies: vpx (optional)
# Provides:     FFMPEG_TARGET, FFMPEG_CMAKE_FLAGS
#
# NOTE: FFmpeg uses a custom configure script — make is the only build tool.
# =============================================================================

if(AV_BUILD_FFMPEG)
    set(FFMPEG_TARGET ffmpeg)

    ExternalProject_Add(${FFMPEG_TARGET}
        URL              ${DEP_FFMPEG_URL}
        URL_HASH         ${DEP_FFMPEG_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/ffmpeg
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            <SOURCE_DIR>/configure
                --prefix=<INSTALL_DIR>
                --extra-cflags="-I<INSTALL_DIR>/include"
                --extra-ldflags="-L<INSTALL_DIR>/${CMAKE_INSTALL_LIBDIR}"
                --enable-shared
                --disable-static
                --disable-gpl
                --enable-nonfree
                --enable-libvpx
        BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${VPX_TARGET}
    )

    set(FFMPEG_CMAKE_FLAGS -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX};${CMAKE_PREFIX_PATH})
    av_register_dep(${FFMPEG_TARGET})
endif()
