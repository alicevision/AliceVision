# =============================================================================
# deps/video.cmake — Video encoding/decoding libraries
#
# Dependencies: (none mandatory — vpx is an optional sub-dep of ffmpeg)
# Provides:     FFMPEG_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_FFMPEG)

    # ── libvpx (optional codec for FFmpeg) ────────────────────────────────────

    if(AV_BUILD_VPX)
        set(VPX_TARGET libvpx)

        # libvpx uses a custom configure script, not CMake.
        ExternalProject_Add(${VPX_TARGET}
            GIT_REPOSITORY  ${DEP_VPX_GIT_REPO}
            GIT_TAG         ${DEP_VPX_GIT_TAG}
            GIT_PROGRESS    OFF
            PREFIX          ${BUILD_DIR}
            BUILD_IN_SOURCE 0
            BUILD_ALWAYS    0
            UPDATE_COMMAND  ""
            INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
            CONFIGURE_COMMAND
                <SOURCE_DIR>/configure
                    --prefix=<INSTALL_DIR>
                    --enable-shared
                    --disable-static
                    --disable-examples
            # libvpx uses a custom configure script — make is the only build tool
            BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        )
        av_register_dep(${VPX_TARGET})
    endif()

    # ── ffmpeg ────────────────────────────────────────────────────────────────

    set(FFMPEG_TARGET ffmpeg)

    # FFmpeg uses its own configure script, not CMake.
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
                --extra-ldflags="-L<INSTALL_DIR>/lib"
                --enable-shared
                --disable-static
                --disable-gpl
                --enable-nonfree
                --enable-libvpx
        # FFmpeg uses a custom configure script — make is the only build tool
        BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${VPX_TARGET}
    )

    set(FFMPEG_CMAKE_FLAGS -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX};${CMAKE_PREFIX_PATH})
    av_register_dep(${FFMPEG_TARGET})

endif() # AV_BUILD_FFMPEG
